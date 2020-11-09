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
#from payload import payload

class PubNavGoal():
    def __init__(self):
        rospy.init_node('trajpub')
        locName = rospy.get_param('~location')
        self.gpub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=10)
        #self.gsub = rospy.Subscriber('amcl_pose',, self.amcl_cb);                   
        self.ena_pub = rospy.Publisher('/mobile_base_controller/control_mode', Byte, queue_size = 1)
        pathName = '/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/traj/'+locName+'.txt'
        if not (os.path.exists(pathName)):
            sys.exit('ERROR: '+pathName+' Does Not Exist! Aborting...')
        self.traj = np.loadtxt(pathName)
        if self.traj.ndim<2:                # in case of only one line in the traj...
            self.traj = np.expand_dims(self.traj,0)
        self.lidar_cent = 0
        self.status = 0
        self.last_loc = PoseWithCovarianceStamped()
        
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.status_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
        
        self.pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        #self.init_pose = [ 0.0453101396561, 0.00364243984222, 0.0, 0.0, 0.0, -0.032842760778, 0.999460531019]
        print(self.traj)

    def status_cb(self,msg):
        self.status = msg.status.status
        print('Status = ', self.status)
        
    def amcl_cb(self,msg):
        self.last_loc = msg
 
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
        for i in range(self.traj.shape[0]):
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
            
            self.status = 0
            self.gpub.publish(msg)
            time.sleep(0.5)  
            if i==0:
                self.gpub.publish(msg)
                time.sleep(0.5)            
                
            print(msg)
            
            cond = True
            itim = rospy.get_time()
            while cond and not rospy.is_shutdown():
                dt = rospy.get_time()-itim
                cond = self.status!=3 and dt<20 
                continue
            if (self.status == 3 and self.traj[i,7]>0):
                # do disinfection stuff here.
                loc = self.last_loc
                #payload.turnOnUVC()
                #self.ena_pub.publish(Byte(0))
                time.sleep(self.traj[i,7])
                #payload.turnOffUVC()
                #self.ena_pub.publish(Byte(1))
                #loc.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
                self.pose_pub.publish(loc)
                time.sleep(1)
            if self.status != 3:
                rospy.logerr('Did not succeed, status code: '+str(self.status))
                sys.exit(self.status-3)            
                
            #if res.status.status != 3:
            #    break
            if rospy.is_shutdown():
                #payload.turnOffUVC()
                break
        
        return 0

if __name__ == '__main__':
    pg = PubNavGoal()
    pg.main()
