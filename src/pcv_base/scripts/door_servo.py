#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
#from move_base_msgs.msg import MoveBaseActionGoal
import numpy as np
from tf import transformations as ts
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
import numpy as np
import math
import os
import sys

class ScanVisualServo():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

        self.vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.door_flag = False
        self.dpos = [0,0,0]
        self.range = [-1.1,1.1]  
        self.pos=[] 

        # Traj following with points
        self.kp = [1.5,1.5,5.0]
        self.kd = [0.6,0.6,0.5]      
        self.vlim = [0.3,0.3,0.3]
        self.cp = [0,0,0]
        
        locName = rospy.get_param('~location')
        print locName
        direction = rospy.get_param('~direction')
        
        self.controller_rate = 20 
        pathName = '/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/traj/'+locName+'.txt'
        if not (os.path.exists(pathName)):
            sys.exit('ERROR: '+pathName+' Does Not Exist! Aborting...')

        self.waypts = np.loadtxt(pathName)
        print direction
        if direction == 1:                                  # Going out of the door, turn 180 degrees of the headings in the traj files.
            self.waypts[:,[5,6]] = self.waypts[:,[6,5]]     # swapping cos and sin values
            self.waypts[:,6] = -self.waypts[:,6]            # make cos(a+90)=-sin(a).
            self.waypts = self.waypts[::-1,:]               # reverse order

        self.waypt_lim = 0.05
        self.lat_lim = 0.01
        self.str_lim = 0.05
        
        self.state = 0; # 0, 1, 2 = ["idle", "align", "crossing"]

    def scan_cb(self, msg):
        amin = msg.angle_min
        dth = msg.angle_increment
        min_id = int((self.range[0]-amin)/dth)
        max_id = int((self.range[1]-amin)/dth)
        mid_id = (min_id + max_id)/2
        self.door_flag = True        
        dists = msg.ranges[min_id:max_id]
        self.pos = np.zeros([2,len(dists)]) 
        for i,d in enumerate(dists):
            self.pos[0,i] = d*np.cos(self.range[0]+dth*i)
            self.pos[1,i] = d*np.sin(self.range[0]+dth*i)
        leftClosestInd = np.argmin(dists[:mid_id])
        rightClosestInd = np.argmin(dists[mid_id:])
        self.leftTh = self.pos[:,leftClosestInd]
        self.rightTh = self.pos[:,rightClosestInd]
    
    def isSideClear(self, dist):
        clear = True
        robotWidth = 0.31
        dist_offset = -0.28
        for pts in np.array(self.pos).transpose():
            if pts[0] < (dist + dist_offset) and np.abs(pts[1]) < robotWidth:
                clear = False
                break
        return clear
                
    def amcl_cb(self,msg):
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat) 
        # current position vector   
        self.cp = [msg.pose.pose.position.x, msg.pose.pose.position.y, eul[2]]
        #print(self.cp)

    def controller_loop(self):
        cmd_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size = 1)
        rt = rospy.Rate(self.controller_rate)
        
        waypt_i = 0
        px_err = 0.
        py_err = 0.
        psteer_err = 0.
        quat = [self.waypts[waypt_i,3],self.waypts[waypt_i,4],
                        self.waypts[waypt_i,5],self.waypts[waypt_i,6]]
        eul = ts.euler_from_quaternion(quat)
        self.waypt_xd =  self.waypts[waypt_i,0]
        self.waypt_yd =  self.waypts[waypt_i,1]
        self.waypt_thd =  eul[2]
        print(self.waypt_xd)
        heading_correction_flag = True
        waypt_i += 1
        #timer for reaching next waypoint
        timeStamp = rospy.get_time()
        while not rospy.is_shutdown():
            # compute distance between present pos estimate and latest waypt update
            #if self.state == 0 or self.state == 2:
            #hist, edges = np.histogram(self.pos[1,:], bins = np.linspace(0,1,11))
            #if (max(hist)>0.3*np.size(self.pos,1) and np.linalg.norm(self.leftTh - self.rightTh)>0.6):
            #    self.state = 1      # door detected.
            self.dis_err = np.sqrt((self.cp[0]-self.waypt_xd)**2+(self.cp[1]-self.waypt_yd)**2)
            ###########################
            # The pd control
            pub_msg = Twist()
            ###### Steer pd
            #steer_err = self.waypt_thd - self.cp[2]
            
            alpha = math.atan2(self.waypt_yd-self.cp[1],self.waypt_xd-self.cp[0])
            if heading_correction_flag:
                steer_err = alpha - self.cp[2]
            else:
                steer_err = self.waypt_thd - self.cp[2]
            
            ori = self.cp[2]-alpha
            x_err = math.cos(ori)*self.dis_err
            y_err = -math.sin(ori)*self.dis_err

            #elif self.state == 1:
            #    self.dis_err = ((self.leftTh + self.rightTh)*0.5)
            #    diff = self.rightTh - self.leftTh
            #    steer_err = np.atan2(diff[1], diff[0])
            #    x_err = self.dis_err[0]        # did not consider the robot's shape... only suitable for small rotational error.
            #    y_err = self.dis_err[1]                
            #    if np.linalg.norm(self.dis_err) < 0.1:
            #        self.state = 2            

            # waypoint updates
            if(self.dis_err < self.waypt_lim and y_err < self.lat_lim and steer_err < self.str_lim ):
                if (waypt_i <= (len(self.waypts) - 1)):
                    #quat = [self.waypts[waypt_i,3],self.waypts[waypt_i,4],
                    #        self.waypts[waypt_i,5],self.waypts[waypt_i,6]]
                    #eul = ts.euler_from_quaternion(quat)
                    self.waypt_xd =  self.waypts[waypt_i,0]
                    self.waypt_yd =  self.waypts[waypt_i,1]
                    #self.waypt_thd =  eul[2]
                    self.waypt_thd = math.atan2(self.waypts[waypt_i,1] - self.waypts[waypt_i-1,1], self.waypts[waypt_i,0]-self.waypts[waypt_i-1,0])
                    waypt_i += 1
                    heading_correction_flag = True
                    timeStamp = rospy.get_time()
                else:
                    print('End')
                    cmd_pub.publish(Twist())
                    break                
             
            if (steer_err > math.pi):
                steer_err = -2*math.pi + steer_err
            elif (steer_err < -math.pi):
                steer_err = 2*math.pi + steer_err
            
            # controller
            if heading_correction_flag:         # alignment phase
                steer = self.kp[2]*steer_err \
                + self.kd[2]*(steer_err-psteer_err)
                pub_msg.angular.z = steer/abs(steer)*min(abs(steer), self.vlim[2])
                pub_msg.linear.x = 0.
                pub_msg.linear.y = 0.
                
                if steer_err < self.str_lim:
                    # with encoder-based AMCL only, need to verify that the pose 
                    # is actually matching with what the LiDAR currently sees, as 
                    # there is a big deadzone where AMCL is not updating.
                    
                    #isClear = self.isSideClear(self.dis_err)
                    #if isClear:
                    
                    # if using fused odometry, the updating threshold of AMCL can
                    # be reduced, hence double-checking is not necessary.
                    heading_correction_flag = False
                    
                    #else:
                    #    print('NEED HELP - WAYPTS OR NAV MAY BE SKEWED!')
                
            else:
                steer = self.kp[2]*steer_err \
                + self.kd[2]*(steer_err-psteer_err)
                pub_msg.angular.z = steer/abs(steer)*min(abs(steer), self.vlim[2])
                ######## x pd
                xvel = self.kp[0]*x_err \
                + self.kd[0]*(x_err-px_err)
                pub_msg.linear.x = xvel/abs(xvel)*min(abs(xvel),self.vlim[0])
                ######## y pd
                yvel = self.kp[1]*y_err \
                + self.kd[1]*(y_err-py_err)
                pub_msg.linear.y = yvel/abs(yvel)*min(abs(yvel), self.vlim[1])
                # Update previous errors
            px_err = x_err
            py_err = y_err
            psteer_err = steer_err
            #print(self.dis_err)
            #print('d: ', self.waypt_xd,self.waypt_yd, self.waypt_thd)
            #print('cp: ', self.cp)
            #print('cmd: ', pub_msg.linear.x,pub_msg.linear.y,pub_msg.angular.z)
            #print('ori: ', ori)
            #print('-'*20)
            # if haven't reached a goal for too long, exit and set stuck flag.
            if (rospy.get_time() - timeStamp > 20):
                cmd_pub.publish(Twist())
                sys.exit(-1)
            else:
                cmd_pub.publish(pub_msg)

            rt.sleep()
        
        for i in range(10):        
            cmd_pub.publish(Twist())
            time.sleep(0.2)
        
    def main(self):
        while not self.door_flag:
            continue
        
        plt.plot(-self.pos[1,:],self.pos[0,:],'o')
        plt.show()
        

if __name__=="__main__":
    rospy.init_node('LaserDoor')
    svs = ScanVisualServo()
    svs.controller_loop()
    #svs.main()
