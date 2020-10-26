#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
#from move_base_msgs.msg import MoveBaseActionGoal
import numpy as np
from tf import transformations as ts
import time
from std_msgs.msg import Header

class PubNavGoal():
    def __init__(self):
        rospy.init_node('trajpub')
        self.gpub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
        #self.gsub = rospy.Subscriber('amcl_pose',, self.amcl_cb);       
        self.traj = np.loadtxt('/home/cartman/Dev/Mobile_Manipulation_Dev/src/wall_follower/resources/ui_speedway.txt')
        self.cp = [0,0];

        print(self.traj)

    def amcl_cb(self, msg):
        self.cp = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])

    def main(self):
        #self.gpub.publish(PoseStamped())
        msg = PoseStamped()
        self.gpub.publish(msg)
        for i in range(self.traj.shape[0]):
            #msg = PoseStamped()
            # Create header
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "map"
            msg.header = h

            # Add position
            msg.pose.position.x =  self.traj[i,0]
            msg.pose.position.y =  self.traj[i,1]
            # Add orientation
            quat = ts.quaternion_from_euler(0,0,self.traj[i,2])
            msg.pose.orientation.x =  quat[0]
            msg.pose.orientation.y =  quat[1]
            msg.pose.orientation.z =  quat[2]
            msg.pose.orientation.w =  quat[3]
            self.gpub.publish(msg)
            print(msg)
            print(self.traj[i,3])
            d=5;
            while (d>0.1 and not rospy.is_shutdown()):
                self.gpub.publish(msg)
                d = np.linalg.norm(self.cp - np.array([self.traj[i,0],self.traj[i,1]]))
                time.sleep(self.traj[i,3])
            if rospy.is_shutdown():
                break


if __name__ == '__main__':
    pg = PubNavGoal()
    pg.main()
