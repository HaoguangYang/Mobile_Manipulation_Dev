#!/usr/bin/env python
import roslib
import sys
import rospy, tf, tf2_ros
import math
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import ros_numpy
import pyrealsense2 as rs2
import matplotlib.pyplot as plt

class targetVisualServoing:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
    rospy.init_node('targetVisualServoing', anonymous=True)
    
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
    self.kp = [0.5,0.5,2.0]
    self.kd = [0.025,0.025,0.02]      
    self.vlim = [0.05,0.05,0.05]
    self.errX = 0.
    self.errY = 0.
    self.errAng = 0.
    self.errXOld = 0.
    self.errYOld = 0.
    self.errAngOld = 0.
    
    # target detection
    #self.featureExtractor = cv2.AKAZE_create()
    #self.featureExtractor = cv2.xfeatures2d.SIFT_create()
    self.featureExtractor = cv2.xfeatures2d.SURF_create(400)
    #self.featureExtractor = cv2.ORB_create()
    #index_params= dict(algorithm = 6, #FLANN_INDEX_LSH,
    #               table_number = 12, # 6
    #               key_size = 20,     # 12
    #               multi_probe_level = 2) #1
    index_params = dict(algorithm = 0, #FLANN_INDEX_KDTREE
                        trees = 5)
    search_params = dict(checks=50)
    self.matcher = cv2.FlannBasedMatcher(index_params,search_params)
    #self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    self.matchFlag = False
    
    # load target features
    im_target = cv2.imread('/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/visualTargets/steelStand.jpg')
    (rows,cols,channels) = im_target.shape
    #scale = 0.1
    #im_target = cv2.resize(im_target, (int(cols*scale), int(rows*scale)), interpolation = cv2.INTER_CUBIC)
    self.im_target = cv2.normalize(im_target, im_target, 0, 255, norm_type=cv2.NORM_MINMAX)
    self.kp_tgt, self.des_tgt = self.featureExtractor.detectAndCompute(self.im_target, None)
    h,w,c = self.im_target.shape
    self.tgt_coords = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    self.tgt_area = h*w

    # target position (of target object from robot base_link)
    self.pos_tgt = [1.800, -0.100]  # need to be read in from file.
    self.ang_tgt = 0.
    
    ca = np.cos(self.ang_tgt)
    sa = np.sin(self.ang_tgt)
    # tgtToObj ^ -1 = R^T * (T^-1)
    self.objToTgt = np.matmul(np.array([[ca,sa,0.],[-sa,ca,0.],[0.,0.,1.]]), \
                np.array([[1.,0.,-self.pos_tgt[0]],[0.,1.,-self.pos_tgt[1]],[0.,0.,1.]]))
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
    
    # target tracking
    self.trackerObj = cv2.TrackerKCF_create
    self.track_window = (0,0,0,0)
    # tracking loop termination criteria: 5 iters max or target moved 1 pixel
    self.track_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5, 1 )
    
    self.rate = 10
    self.rosRate = rospy.Rate(self.rate)
  
  def matchRGBfeature(self,frame):
    frame = cv2.normalize(frame, frame, 0, 255, norm_type=cv2.NORM_MINMAX)
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #frame[:,:,2] = 111*np.ones(frame.shape[0:2])
    #kp_f = self.featureExtractor.detect(frame, None)
    kp_f, des_f = self.featureExtractor.detectAndCompute(frame, None)
    matches = self.matcher.knnMatch(self.des_tgt, des_f, k=2)
    # Consider only good matches (top on the list)
    goodMatch = []
    for match in matches:
        if len(match) > 1:
            if match[0].distance < 0.8*match[1].distance:
                goodMatch.append(match[0])
        elif len(match)>0:
            goodMatch.append(match[0])
    
    matchesMask = None
    bbox = []
    img_disp = frame
    
    if len(goodMatch)>5:
        src_pts = np.float32([ self.kp_tgt[m.queryIdx].pt for m in goodMatch ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp_f[m.trainIdx].pt for m in goodMatch ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 2.0)
        matchesMask1 = mask.ravel().tolist()

        #print(M)
        if M is not None:
            Msub = abs(M[0:2,0:2])
            # how close is the rotational part of M to diagonal, since we are moving two-dimensionally
            # reference: https://math.stackexchange.com/questions/1392491/measure-of-how-much-diagonal-a-matrix-is
            j = np.array([1,1])
            r = np.array([1,2])
            r2 = np.array([1,4])
            n = np.sum(Msub)
            sx = np.dot(np.dot(r,Msub),j)
            sy = np.dot(np.dot(j,Msub),r)
            sxx = np.dot(np.dot(r2,Msub),j)
            syy = np.dot(np.dot(j, Msub),r2)
            sxy = np.dot(np.dot(r, Msub),r)
            r = (n*sxy - sx*sy)/np.sqrt((n*sxx-sx**2)*(n*syy-sy**2))
            print(r)
            if M[0,0]>0 and M[1,1]>0 and r>0.6:
                dst = cv2.perspectiveTransform(self.tgt_coords,M)
                dst_area = -0.5*(dst[0,0,0]*dst[1,0,1] - dst[1,0,0]*dst[0,0,1] \
                              + dst[1,0,0]*dst[2,0,1] - dst[2,0,0]*dst[1,0,1] \
                              + dst[2,0,0]*dst[3,0,1] - dst[3,0,0]*dst[2,0,1] \
                              + dst[3,0,0]*dst[0,0,1] - dst[0,0,0]*dst[3,0,1])
                #print(dst_area)
                if dst_area > 0.4*self.tgt_area and dst_area < 3.0*self.tgt_area:
                    matchesMask = matchesMask1
                    bbox = [np.int32(dst)]
                    img_disp = cv2.polylines(frame, bbox, True, 255, 3, cv2.LINE_AA)
                    bbox = np.array(bbox)
                    #print(bbox)
                    c = max(np.min(bbox[:,:,:,0]),0)
                    r = max(np.min(bbox[:,:,:,1]),0)
                    w = min(np.max(bbox[:,:,:,0]),frame.shape[1]-1)-c+1
                    h = min(np.max(bbox[:,:,:,1]),frame.shape[0]-1)-r+1
                    print(bbox)
                    print([c,r,w,h])
                    self.track_window = (c,r,w,h)
                    self.tracker = self.trackerObj()
                    print(self.track_window)
                    self.tracker.init(frame, self.track_window)
                    #print('Found!')
                    self.matchFlag = True
                    
    #img = cv2.drawKeypoints(cv_image, kp, None, (0,0,255), 4)
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

    img_disp = cv2.drawMatches(self.im_target, self.kp_tgt, img_disp, kp_f, goodMatch, None, **draw_params)
    
    #cv2.imshow("Image window", img_disp)
    #cv2.waitKey(1)
    
  def trackRGBfeature(self,frame):
    # use the bounding box generated in the match step and track the matched part
    # does not perform further matching unless the tracking is lost.
    # updates bounding box
    ret_ok, track_window = self.tracker.update(frame)
    self.track_window = (int(track_window[0]), int(track_window[1]), int(track_window[2]), int(track_window[3]))
    if ret_ok:
        corner1 = (self.track_window[0], self.track_window[1])
        corner2 = (self.track_window[0]+self.track_window[2], self.track_window[1]+self.track_window[3])
        cv2.rectangle(frame, corner1, corner2, (255,0,0), 2, 1)
    else:
        self.matchFlag = False
    #cv2.imshow("Tracking Results", frame)
    #cv2.waitKey(1)
    
  def extractObjectMask(self):
    # using the depth image, extract the foreground object in the bounding box using Otsu
    # generates a mask of which pixels belongs to the foreground object
    DframeROI = self.Dframe[self.track_window[1]:self.track_window[1]+self.track_window[3],\
                            self.track_window[0]:self.track_window[0]+self.track_window[2]]
    DframeROI = np.clip(DframeROI*256.0/5000.0, 0., 255.).astype('uint8')
    ret, th = cv2.threshold(DframeROI, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #print(th)
    mask = np.zeros(self.Dframe.shape[0:2], dtype='uint8')
    if th is not None:
        mask[self.track_window[1]:self.track_window[1]+self.track_window[3],\
             self.track_window[0]:self.track_window[0]+self.track_window[2]] = \
             255-th
    else:
        print(ret,th)
        self.vel_pub.publish(Twist())
    cv2.imshow("Depth Mask", mask)
    cv2.waitKey(1)
    return mask
  
  def getTransformAndError(self, mask):
    # using the mask of the foreground object and the corresponding depths,
    # determine the relative position using PCA on the X and Z (depth) coordinates of the pixels in the mask
    # generates a tf.
    #xyz_array = ros_numpy.point_cloud2.get_xyz_points(self.ptCloud)
    x, y = np.nonzero(mask)
    if len(x)>10:
        pts = np.zeros([len(x),3])
        n = 0
        for idx in np.array([x,y]).transpose():
            pts[n,:] = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [idx[1], idx[0]], self.Dframe[idx[0], idx[1]])
            n += 1
        
        #print(np.mean(pts,0))
        #plt.scatter(pts[:,0], pts[:,2])
        #plt.show()
        # perform PCA on the points (Nx2 numpy array)
        mean, eigvec = cv2.PCACompute(pts[:,[0,2]], mean=None)
        # position of the center of the target surface w.r.t. the camera center.
        # center is expressed w.r.t. the camera frame (x front, y left),  convert from mm to m.
        center = [0.001*mean[0,1], -0.001*mean[0,0]]
        # angle of the feature, i.e. the angle of the target surface with the camera plane
        angle = np.arctan2(eigvec[0,0], -eigvec[0,1])
        angle_pi = angle/np.pi
        # filtering the angle readings as it affects the transform drastically...
        if angle_pi>0.25 and angle_pi<0.75:
            # swapped x with y axis
            angle_pi -= 0.5
        elif angle_pi<-0.25 and angle_pi>-0.75:
            angle_pi += 0.5
        elif angle_pi < -0.75:
            # x axis reverted
            angle_pi += 1.
        elif angle_pi > 0.75:
            angle_pi -= 1.
        angle = angle_pi * np.pi
        #    angle += np.pi/2
        #else:
        #    angle -= np.pi/2
        #angle = angle - np.pi/2
        #if angle <= -np.pi/2:
        #    angle = angle + np.pi
        
        #print(center[0], center[1], angle)
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
            #print('===')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    else:
        self.vel_pub.publish(Twist())
    
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
      
  def ptCloudCallback(self,data):
    self.ptCloud = data
    
  def run(self):
    try:
        while not rospy.is_shutdown():
            if not self.matchFlag:
                self.matchRGBfeature(self.bgrFrame)
                self.vel_pub.publish(Twist())
            else:
                self.trackRGBfeature(self.bgrFrame)
                self.getTransformAndError(self.extractObjectMask())
                self.control()
                if abs(self.errX) <= 0.05 and abs(self.errY)<=0.05:
                    self.vel_pub.publish(Twist())
                    break
            self.rosRate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        self.vel_pub.publish(Twist())
        cv2.destroyAllWindows()

if __name__ == '__main__':
    vs = targetVisualServoing()
    vs.run()
