#!/usr/bin/env python
import roslib
import sys
import rospy, tf, tf2_ros
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import ros_numpy
import pyrealsense2 as rs2

class arucoVisualServoing:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
    rospy.init_node('targetVisualServoing', anonymous=True)
    self.refL = rospy.get_param('~refIDLeft',801)
    self.refR = rospy.get_param('~refIDRight',802)
    self.refs = [801, 802]
    arucoType = rospy.get_param('~codeType','DICT_4X4_1000')
    self.bridge = CvBridge()
    self.rgb_sub = rospy.Subscriber("cam_d1/color/image_raw",Image,self.rgbCallback)
    init_data = rospy.wait_for_message("cam_d1/color/image_raw",Image)
    self.rgbCallback(init_data)
    self.depth_sub = rospy.Subscriber("cam_d1/aligned_depth_to_color/image_raw",Image,self.grayCallback)
    init_data = rospy.wait_for_message("cam_d1/aligned_depth_to_color/image_raw",Image)
    self.grayCallback(init_data)
    # depth camera info for transformation
    #self.info_sub = rospy.Subscriber("/cam_d1/aligned_depth_to_color/camera_info", CameraInfo, self.imageDepthInfoCallback)
    init_data = rospy.wait_for_message("/cam_d1/aligned_depth_to_color/camera_info", CameraInfo)
    self.intrinsics = rs2.intrinsics()
    self.intrinsics.width = init_data.width
    self.intrinsics.height = init_data.height
    self.intrinsics.ppx = init_data.K[2]
    self.intrinsics.ppy = init_data.K[5]
    self.intrinsics.fx = init_data.K[0]
    self.intrinsics.fy = init_data.K[4]
    if init_data.distortion_model == 'plumb_bob':
        self.intrinsics.model = rs2.distortion.brown_conrady
    elif init_data.distortion_model == 'equidistant':
        self.intrinsics.model = rs2.distortion.kannala_brandt4
    self.intrinsics.coeffs = [i for i in init_data.D]
    #self.pc_sub = rospy.Subscriber("cam_d1/depth/color/points",PointCloud2,self.ptCloudCallback)
    #init_data = rospy.wait_for_message("cam_d1/depth/color/points",PointCloud2)
    #self.ptCloudCallback(init_data)
    self.vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    
    # control
    self.kp = [0.6,0.6,0.6]
    self.kd = [0.05,0.05,0.05]      
    self.vlim = [0.1,0.1,0.1]
    self.errX = 0.
    self.errY = 0.
    self.errAng = 0.
    self.errXOld = 0.
    self.errYOld = 0.
    self.errAngOld = 0.
    
    ARUCO_DICT = {
	    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
	    #"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	    #"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	    #"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	    #"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }
    # target detection
    self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[arucoType])
    self.arucoParams = cv2.aruco.DetectorParameters_create()
    
    self.matchFlag = False
    self.controlValid = False
    
    # target position (of the middle point between ID1 and ID2, from robot base_link)
    tgtX = rospy.get_param('~desX',1.85)
    tgtY = rospy.get_param('~desY',0.)
    self.pos_tgt = [tgtX, tgtY]
    self.ang_tgt = rospy.get_param('~desTh',0.0)
    self.relPosL = np.array([0.,0.])
    self.relPosR = np.array([0.,0.])
    
    ca = np.cos(self.ang_tgt)
    sa = np.sin(self.ang_tgt)
    # tgtToObj ^ -1 = R^T * (T^-1)
    self.objToTgt = np.matmul(np.array([[ca,sa,0.],[-sa,ca,0.],[0.,0.,1.]]), \
                np.array([[1.,0.,-self.pos_tgt[0]],[0.,1.,-self.pos_tgt[1]],[0.,0.,1.]]))
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
    
    # target tracking
    self.rate = 10
    self.rosRate = rospy.Rate(self.rate)
  
  # get relative position of a point given its pixel coordinates
  def getRelativePos(self, pixCoord, meshSize = 3):
    # using the center pixel coordinates of a detected target.
    # generate a 3x3 mask around the center pixel
    # deproject the aligned depth frame.

    assert (meshSize%2 > 0)
    pixCoord = np.round(pixCoord)
    #xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.ptCloud)
    x = np.linspace(pixCoord[0]-(meshSize-1)/2, pixCoord[0]+(meshSize-1)/2, meshSize, dtype=np.int32)
    y = np.linspace(pixCoord[1]-(meshSize-1)/2, pixCoord[1]+(meshSize-1)/2, meshSize, dtype=np.int32)

    # deproject the depth image
    pts = np.zeros([meshSize*meshSize,3])
    n = 0
    for xv in x:
        for yv in y:
            pts[n,:] = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xv, yv], self.Dframe[yv, xv])
            n += 1
    relPos = np.sum(pts,0)/(meshSize**2)
    relPosT = np.array([relPos[2], -relPos[0]])*0.001
    return relPosT
        
  def getTransformFromCoords(self):
        # position of the center of the target surface w.r.t. the camera center.
        # center is expressed w.r.t. the camera frame (x front, y left),  convert from mm to m.
        center = np.mean(self.relPos,0)
        # angle of the feature, i.e. the angle of the target surface with the camera plane
        x = np.vstack([-self.relPos[:,1], np.ones(self.relPos.shape[0])]).T
        # y = m*x + c
        m, c = np.linalg.lstsq(x, self.relPos[:,0])[0]
        
        angle = np.arctan2(m,1)
        print(center[0], center[1], angle)
        ca = np.cos(angle)
        sa = np.sin(angle)
        #print(center, angle)
        # transform from camera to object
        camToObject = np.array([[ca,-sa,center[0]],[sa,ca,center[1]],[0.,0.,1.]])
        
        try:
            # transform from base link to depth camera
            baseToCamTF = self.tfBuffer.lookup_transform('base_link','cam_d1_aligned_depth_to_color_frame',rospy.Time()).transform
            tx = baseToCamTF.translation.x
            ty = baseToCamTF.translation.y
            th = tf.transformations.euler_from_quaternion(\
                                (baseToCamTF.rotation.x, \
                                 baseToCamTF.rotation.y, \
                                 baseToCamTF.rotation.z, \
                                 baseToCamTF.rotation.w)
                                )[2]
            ct = np.cos(th)
            st = np.sin(th)
            baseToCam = np.array([[ct,-st,tx],[st,ct,ty],[0.,0.,1.]])
            baseToTgt = np.matmul(np.matmul(baseToCam, camToObject), self.objToTgt)
            self.errAng = np.arctan2(baseToTgt[1,0],baseToTgt[0,0])
            self.errX = baseToTgt[0,2]
            self.errY = baseToTgt[1,2]
            print(self.errX, self.errY, self.errAng)
            print('===')
            self.controlValid = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('transform exceptions')
            pass
    
  def control(self):
    pub_msg = Twist()
    
    steer = self.kp[2]*self.errAng \
            + self.kd[2]*(self.errAng-self.errAngOld)*self.rate
    pub_msg.angular.z = steer/(abs(steer)+1e-3)*min(abs(steer), self.vlim[2])

    xvel = self.kp[0]*self.errX \
            + self.kd[0]*(self.errX-self.errXOld)*self.rate
    pub_msg.linear.x = xvel/(abs(xvel)+1e-3)*min(abs(xvel),self.vlim[0])

    yvel = self.kp[1]*self.errY \
            + self.kd[1]*(self.errY-self.errYOld)*self.rate
    pub_msg.linear.y = yvel/(abs(yvel)+1e-3)*min(abs(yvel), self.vlim[1])
    
    self.errAngOld = self.errAng
    self.errXOld = self.errX
    self.errYOld = self.errY
    
    self.vel_pub.publish(pub_msg)
    
  def rgbCallback(self,data):
    try:
      self.bgrFrame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def grayCallback(self,data):
    try:
      data.encoding = "mono16"
      self.Dframe = self.bridge.imgmsg_to_cv2(data, "mono16")
      #print(np.mean(self.Dframe))
    except CvBridgeError as e:
      print(e)
    
    #cv_image = cv2.Sobel(cv_image,cv2.CV_32F,1,0,ksize=3)
    #cv2.normalize(cv_image, cv_image, 0, 255, cv2.NORM_MINMAX)
    #cv2.imshow("Depth window", self.Dframe/10000.)
    #cv2.waitKey(1)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)
        
  def run(self):
    try:
        while not rospy.is_shutdown():
            (corners, ids, rejected) = cv2.aruco.detectMarkers(self.bgrFrame, self.arucoDict, parameters=self.arucoParams)
            #print(corners)
            #print(ids)
            #print(rejected)
            #print('======================')
            # markers detected.
            if len(corners) > 0:
                ids = ids.flatten()
                # propagate through the detected markers to determine the left and right reference markers
                self.relPos = np.empty((0,2),dtype=float)
                for (markerCorner, markerID) in zip(corners, ids):
                    if markerID in self.refs:
                        (topLeft, topRight, bottomRight, bottomLeft) = markerCorner[0]
                        coord = np.array([topLeft[0]+topRight[0]+bottomRight[0]+bottomLeft[0], \
                                          topLeft[1]+topRight[1]+bottomRight[1]+bottomLeft[1]])*0.25
                        self.relPos = np.append(self.relPos, np.array([self.getRelativePos(coord)]),0)
                        
                # both codes detected
                #print(Ldetected, Rdetected)
                if self.relPos.shape[0] > 1:
                    self.getTransformFromCoords()
                    self.control()
                else:
                    self.vel_pub.publish(Twist())
                    print('Unable to see both markers, aborting...')
                    break
            #if not self.matchFlag:
            #    self.matchRGBfeature(self.bgrFrame)
            #    self.vel_pub.publish(Twist())
            #else:
            #    self.trackRGBfeature(self.bgrFrame)
            #    self.getTransformAndError(self.extractObjectMask())
            #    self.control()
                if abs(self.errX) <= 0.005 and abs(self.errY)<=0.005 and abs(self.errAng)<=0.005 and self.controlValid:
                    print(self.errX, self.errY, self.errAng)
                    self.vel_pub.publish(Twist())
                    print('Precision tolerance achieved, aborting...')
                    break
            self.rosRate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        self.vel_pub.publish(Twist())
        #cv2.destroyAllWindows()

if __name__ == '__main__':
    vs = arucoVisualServoing()
    vs.run()
