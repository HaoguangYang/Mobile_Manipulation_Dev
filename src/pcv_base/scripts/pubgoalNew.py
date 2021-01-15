#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
#from move_base_msgs.msg import MoveBaseActionGoal
import numpy as np
from tf import transformations as ts
import time
import os
import sys
from std_msgs.msg import Header, Byte
from move_base_msgs.msg import MoveBaseActionResult

class PubNavGoal():
    def __init__(self):
        rospy.init_node('trajpub')
        locName = rospy.get_param('~location', 'traj')
        self.timeout = rospy.get_param('~timeout', '300')
        self.gpub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
        self.ena_pub = rospy.Publisher('/mobile_base_controller/control_mode', Byte, queue_size = 1)
        pathName = rospy.get_param('~waypt_file_path', '/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/traj/'+locName+'.txt')
        if not (os.path.exists(pathName)):
            sys.exit('ERROR: '+pathName+' Does Not Exist! Aborting...')
        self.traj = np.loadtxt(pathName)
        if self.traj.ndim<2:                # in case of only one line in the traj...
            self.traj = np.expand_dims(self.traj,0)
            
        self.startLine = rospy.get_param('~startLn','1') - 1
        self.endLine = rospy.get_param('~endLn', str(self.traj.shape[0])) - 1
        assert (self.startLine >= 0)
        assert (self.endLine < self.traj.shape[0])
        self.isMirror = rospy.get_param('~isMirror','0')
        if self.isMirror == 1:                                         # Mirroring the map along Y axis
            self.traj[:,0] = -self.traj[:,0]
            self.traj[:,[5,6]] = self.traj[:,[6,5]]
        if rospy.get_param('~dir','0') == 0:
            self.direction = 1      # in normal direction
        else:
            self.direction = -1     # in reversed direction
            self.traj[:,[5,6]] = self.traj[:,[6,5]]     # swapping cos and sin values
            self.traj[:,6] = -self.traj[:,6]            # make cos(a+90)=-sin(a).
        
        self.status = 0
        self.last_loc = PoseWithCovarianceStamped()
        
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.status_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber('/pauseAction', Byte, self.pause_cb)
        
        self.pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        print(self.traj)
        self.pause_duration = 0.
        self.pause_time = 0.
        self.pause_status = False

    def status_cb(self,msg):
        self.status = msg.status.status
        print('Status = ', self.status)
        
    def amcl_cb(self,msg):
        self.last_loc = msg
        
    def pause_cb(self,msg):
        if msg.data == 1 and not self.pause_status:
            self.ena_pub.publish(Byte(0))
            self.pause_time = rospy.get_time()
            self.pause_status = True
        elif msg.data == 0 and self.pause_status:
            self.ena_pub.publish(Byte(1))
            self.pause_duration = self.pause_duration + rospy.get_time()-self.pause_time
            self.pause_status = False
 
    def main(self):
        # Publish initial pose
        #ret = input('Is robot in position? (1-Yes, 0-NO)')
        #if ret==0:
        #    rospy.signal_shutdown('Exit')
        
        #ipose = PoseWithCovarianceStamped()
        #ipose.pose.pose.position.x = self.init_pose[0]
        #ipose.pose.pose.position.y = self.init_pose[1]
        #ipose.pose.pose.orientation.z = self.init_pose[5]
        #ipose.pose.pose.orientation.w = self.init_pose[6]
        #self.pose_pub.publish(ipose)
        #time.sleep(1)

        # Start publishing goals from trajectory
        #msg = PoseStamped()
        #self.gpub.publish(msg)
        #self.gpub.publish(msg)
        while not self.gpub.get_num_connections():
            pass
        for i in range(self.startLine, self.endLine + self.direction, self.direction):
            msg = PoseStamped()            
            # Create header
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "map"
            msg.header = h

            # Add position
            msg.pose.position.x =  self.traj[i,0]
            msg.pose.position.y =  self.traj[i,1]
            # Add orientation
            msg.pose.orientation.x =  self.traj[i,3]
            msg.pose.orientation.y =  self.traj[i,4]
            msg.pose.orientation.z =  self.traj[i,5]
            msg.pose.orientation.w =  self.traj[i,6]
            
            self.gpub.publish(msg)
            self.status = 0
            time.sleep(0.5)  
            if i==0:
                self.gpub.publish(msg)
                time.sleep(0.5)
            print(msg)
            
            cond = True
            itim = rospy.get_time()
            while cond and not rospy.is_shutdown():
                if not self.pause_status:
                    dt = rospy.get_time()-itim-self.pause_duration
                cond = self.status!=3 and dt<self.timeout
                continue
            
            if self.status != 3:
                rospy.logerr('Did not succeed, status code: '+str(self.status))
                sys.exit(self.status-3)
                
            #if res.status.status != 3:
            #    break
            if rospy.is_shutdown():
                break
        
        return 0

if __name__ == '__main__':
    pg = PubNavGoal()
    pg.main()
