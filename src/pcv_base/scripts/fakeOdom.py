#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Byte
from pcv_base.msg import electricalStatus

def cmd_vel_cb(data):
    vx = data.linear.x
    vy = data.linear.y
    vth = data.angular.z
    
def ctrl_mode_cb(data):
    mode = data.data

def main():
    x = 0.
    y = 0.
    th = 0.
    vx = 0.
    vy = 0.
    vth = 0.
    mode = 0
    freq = 50.
    odom = Odometry()
    eStatus = electricalStatus()
    rospy.init_node('pcv_base_fake')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, cmd_vel_cb)
    rospy.Subscriber("/mobile_base_controller/control_mode", Byte, ctrl_mode_cb)
    odom_broadcaster = tf.TransformBroadcaster()
    eStatus_pub = rospy.Publisher("electricalStatus", electricalStatus, queue_size=1)

    curTime = rospy.Time.now()

    r = rospy.Rate(freq)

    while not rospy.is_shutdown():
        if mode == 1:
            x += vx/freq
            y += vy/freq
            th += vth/freq
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        else:
            odom.twist.twist = Twist()
        curTime = rospy.Time.now()
        
        odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
        odom_broadcaster.sendTransform(
            (x, y, 0),          # position
            odom_quat,          # orientation
            curTime,            # time stamp
            "odom",             # odom frame name
            "base_link")        # base name
        odom.header.stamp = curTime
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
        odom_pub.publish(odom)
        
        eStatus.stamp = curTime
        eStatus.steer_1_Volt = 42.
        eStatus.steer_2_Volt = 42.
        eStatus.steer_3_Volt = 42.
        eStatus.steer_4_Volt = 42.
        
        eStatus.roll_1_Volt = 42.
        eStatus.roll_2_Volt = 42.
        eStatus.roll_3_Volt = 42.
        eStatus.roll_4_Volt = 42.
        
        eStatus.steer_1_Amp = 0.1
        eStatus.steer_2_Amp = 0.1
        eStatus.steer_3_Amp = 0.1
        eStatus.steer_4_Amp = 0.1
        
        eStatus.roll_1_Amp = 0.1
        eStatus.roll_2_Amp = 0.1
        eStatus.roll_3_Amp = 0.1
        eStatus.roll_4_Amp = 0.1
        
        eStatus_pub.publish(eStatus)
        
        r.sleep()

if __name__ == "__main__":
    main()

