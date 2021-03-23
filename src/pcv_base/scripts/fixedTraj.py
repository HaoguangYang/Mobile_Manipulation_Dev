#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Byte
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
#from move_base_msgs.msg import MoveBaseActionGoal
import tf
import numpy as np
import time

class fixedTrajFixedSpeedProfile():
    def __init__(self):
        rospy.init_node('trajpub')
        self.vpub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
        self.pos = np.array([0.75, 4.8, 0.0])
        #self.pathSpeed = np.array([0.7, 0.7, 0.7, 0.2174, 0.7, 0.4938, 0.7, 0.2174, 0.7])
        #self.pathSpeed = np.array([0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7])        
        #self.traj = np.array([[2.25, 5.0, 0.0],
        #                      [3.75, 5.0, 0.0], 
        #                      [3.75, 3.15, 0.0], 
        #                      [2.25, 3.15, 0.0],
        #                      [3.75, 3.15, 0.0],
        #                      [5.25, 3.15, 0.0],
        #                      [2.25, 3.15, 0.0],
        #                      [0.75, 3.15, 0.0],
        #                      [0.75, 5.0, 0.0]])
        #self.traj = np.array([[2.25, 4.8, 0.0],
        #                      [3.75, 4.8, 0.0], 
        #                      [3.75, 2.95, 0.0], 
        #                      [2.25, 2.95, 0.0],
        #                      [3.75, 2.95, 0.0],
        #                      [5.25, 2.95, 0.0],
        #                      [2.25, 2.95, 0.0],
        #                      [0.75, 2.95, 0.0],
        #                      [0.75, 4.8, 0.0]])
        self.pathSpeed = np.array([0.7, 0.7, 0.7, 0.7])
        self.traj = np.array([[5.25, 4.8, 0.0],
                              [5.25, 2.95, 0.0], 
                              [0.75, 2.95, 0.0], 
                              [0.75, 4.8, 0.0]])
        self.kp = [1.0, 1.0, 1.0]
        self.ki = [0.0, 0.0, 0.0]
        self.sumErr = np.array([0., 0., 0.])
        self.freq = 50.
        self.leg = 0
        self.timeLastWaypoint = rospy.Time.now().to_sec()
        self.timeNextWaypoint = self.timeLastWaypoint + np.linalg.norm(self.traj[self.leg][0:2]-self.pos[0:2])/self.pathSpeed[self.leg]
        #vel = np.array([0.1, 0.1, 0.1, 0.1])
        #legs = np.array([[1.0,0.0],
        #                [0.0,1.0],
        #                [-1.0,0.0],
        #                [0.0,-1.0]])
        #t = np.array([
        #    np.linalg.norm(legs[0])/vel[0],
        #    np.linalg.norm(legs[1])/vel[1],
        #    np.linalg.norm(legs[2])/vel[2],
        #    np.linalg.norm(legs[3])/vel[3]]
        #    )
        #self.velTarget = 1./self.pathTime
        #for i in range(0,np.size(self.traj,0)):
        #    self.velTarget[i] = self.velTarget[i]*np.linalg.norm(self.traj[i][0:2]-self.traj[(i+1)%np.size(self.traj, 0)][0:2])
            #print(self.traj[i][0:2]-self.traj[(i+1)%np.size(self.traj, 0)][0:2])
        #print (self.velTarget)

    def amcl_callback(self,data):
        self.pos[0] = data.pose.pose.position.x
        self.pos[1] = data.pose.pose.position.y
        ori = data.pose.pose.orientation
        eu_ori = tf.transformations.euler_from_quaternion([ori.x,ori.y,ori.z,ori.w])
        self.pos[2] = eu_ori[2]

    def control_fcn(self, err):
        # err: [ex, ey, eorientation] - in Base frame
        #print(err)
        self.vel_x = self.kp[0]*err[0] + self.ki[0]*self.sumErr[0]
        self.vel_y = self.kp[1]*err[1] + self.ki[1]*self.sumErr[1]
        self.omega = self.kp[2]*err[2] + self.ki[2]*self.sumErr[2]
        self.sumErr = self.sumErr + err/self.freq
        
    def main(self):
        #self.gpub.publish(PoseStamped())
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        msg = Twist()
        self.vpub.publish(msg)
        r = rospy.Rate(self.freq)

        self.timeLastWaypoint = rospy.Time.now().to_sec()
        self.timeNextWaypoint = self.timeLastWaypoint + \
                np.linalg.norm(self.traj[self.leg][0:2]-\
                               self.traj[(self.leg-1)%np.size(self.traj, 0)][0:2])/\
                               self.pathSpeed[self.leg]
        while (not rospy.is_shutdown()):
            #msg = PoseStamped()
            # Create header
            #timeNow = rospy.Time.now()
            timeNow = rospy.Time.now().to_sec()
            if timeNow > self.timeNextWaypoint:
                self.leg = (self.leg + 1)%np.size(self.traj,0)
                self.timeLastWaypoint = self.timeNextWaypoint
                self.timeNextWaypoint = self.timeNextWaypoint + \
                        np.linalg.norm(self.traj[self.leg][0:2]-\
                               self.traj[(self.leg-1)%np.size(self.traj, 0)][0:2])/\
                               self.pathSpeed[self.leg]
            w = (timeNow-self.timeLastWaypoint)/(self.timeNextWaypoint-self.timeLastWaypoint)
            #print(self.leg)
            err = w*self.traj[self.leg]+(1-w)*self.traj[(self.leg-1)%np.size(self.traj,0)] - self.pos
            #err = self.traj[(self.leg+1)%np.size(self.traj, 0)] - self.pos
            err_robot = np.array([0.,0.,0.])
            err_robot[0] = err[0]*np.cos(err[2]) + err[1]*np.sin(err[2])
            err_robot[1] = -err[0]*np.sin(err[2]) + err[1]*np.cos(err[2])
            err_robot[2] = err[2]
            self.control_fcn(err_robot)
            #print(self.vel_x, self.vel_y, self.omega)
            msg.linear.x = np.clip(self.vel_x, -0.7, 0.7)
            msg.linear.y = np.clip(self.vel_y, -0.7, 0.7)
            msg.angular.z = np.clip(self.omega, -0.7, 0.7)
            self.vpub.publish(msg)
            self.pos[0] = self.pos[0] + self.vel_x/self.freq*np.cos(err[2]) + self.vel_y/self.freq*np.sin(err[2])
            self.pos[1] = self.pos[1] - self.vel_x/self.freq*np.sin(err[2]) + self.vel_y/self.freq*np.cos(err[2])
            self.pos[2] = self.pos[2] + self.omega/self.freq
            #if (timeNow - timeStart).secs > t[i]:
            #    i = (i + 1)%4
            #    timeStart = timeNow
            
            #msg.linear.x = legs[i][0]/t[i]
            #msg.linear.y = legs[i][1]/t[i]
            #msg.linear.z = 0.
            #msg.angular.x = 0.
            #msg.angular.y = 0.
            #msg.angular.z = 0.

            #self.vpub.publish(msg)
            r.sleep()
        msg = Twist()
        self.vpub.publish(msg)

if __name__ == '__main__':
    vp = fixedTrajFixedSpeedProfile()
    vp.main()
