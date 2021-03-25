#!/usr/bin/env python
import rospy
import warnings
import roslaunch
import numpy as np
from std_msgs.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf import transformations as ts

class RobotBase(object):
    name = "robotBase"
    
    def __init__(self):
        # Prerequisits: Map server launched, robot_pose_ekf and amcl launched.
        rospy.init_node(self.name, anonymous=True)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.__launch_baseNode = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/pcv_node.launch'])
        self.__ena_pub = rospy.Publisher('/mobile_base_controller/control_mode', Byte, queue_size = 1)
        self.__vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
        self.v_lim = [0.5, 0.5, 0.5]    # safety limits
        # Initialize before use.
        self.basePos = np.array([0.,0.,0.])
        self.baseTime = Time()
        #init_data = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        #self.__amcl_cb(init_data)
        self.__ekfPos = np.array([0.,0.,0.])
        self.__ekfTime = Time()
        #init_data = rospy.wait_for_message('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped)
        #self.__ekf_cb(init_data)
        self.paused = False
        while not self.__ena_pub.get_num_connections():
            pass
        
    def execute(self):
        self.__amcl_inst = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__amcl_cb)
        self.__ekf_inst = rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.__ekf_cb)
        self.__watchdog_inst = rospy.Timer(rospy.Duration(1), self.__watchdog_cb)
    
    def shutdown(self):
        self.watchdog_inst.shutdown()
        self.amcl_inst.shutdown()
        self.ekf_inst.shutdown()
        self.launch_baseNode.shutdown()
    
    def evaluate(self):
        pass
        
    def estimate(self):
        pass
    
    def __watchdog_cb(self, event):
        try:
            check_output(["pidof","pcv_base_node"])
        except subprocess.CalledProcessError:
            # thread is not running
            self.__launch_baseNode.shutdown()
            self.__launch_baseNode = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/pcv_node.launch'])
            self.__launch_baseNode.start()
        
    def __amcl_cb(self, msg):
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat) 
        # current position vector   
        self.basePos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, eul[2]])
        self.baseTime = msg.header.stamp
        
    def __ekf_cb(self, msg):
        quat_new = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat_new)
        ekfPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, eul[2]])
        ekfTime = msg.header.stamp
        # if got newer odom data
        if (ekfTime > self.baseTime):
            ekfDt = ekfTime - self.__ekfTime
            vel = (ekfPos - self.__ekfPos)/ekfDt
            dAng = self.__ekfPos[2] - self.basePos[2]
            # velocity of robot in base frame
            vel_b = np.dot(np.array([[np.cos(dAng), -np.sin(dAng)],
                                     [np.sin(dAng), np.cos(dAng)]]),vel[0:2])
            self.basePos = self.basePos + np.array([vel_b[0], vel_b[1], vel[2]])*(ekfTime-self.baseTime)
            self.baseTime = ekfTime
        self.__ekfTime = ekfTime
        self.__ekfPos = ekfPos
        
    def robot_enable(self, state):
        # 0 - Disable
        # 1 - Velocity
        # 2 - Torque
        if (state==0):
            self.__ena_pub.publish(Byte(0))
            self.paused = True
        elif (state==1):
            self.__ena_pub.publish(Byte(1))
            self.paused = False
        elif (state==2):
            self.__ena_pub.publish(Byte(2))
            self.paused = False
        else:
            warnings.warn('Unrecognized robot control mode, setting the mode is neglected!')
        
    def robot_cmd_vel(self, vx, vy, omega):
        cmd = Twist()
        cmd.linear.x = np.clip(vx, -self.v_lim[0], self.v_lim[0])
        cmd.linear.y = np.clip(vy, -self.v_lim[1], self.v_lim[1])
        cmd.angular.z = np.clip(omega, -self.v_lim[2], self.v_lim[2])
        self.__vel_pub.publish(cmd)
        
if __name__ == "__main__":
    base = RobotBase()
    base.__init__()
    base.execute()
    robot_enable(1)
    rospy.spin()
    base.shutdown()
