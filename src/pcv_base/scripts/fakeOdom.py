#!/usr/bin/env python
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

x = 0.
y = 0.
th = 0.
vx = 0.
vy = 0.
vth = 0.
mode = 0
freq = 50.
odom = Odometry()

def cmd_vel_cb(data):
    vx = data.linear.x
    vy = data.linear.y
    vth = data.angular.z
    
def ctrl_mode_cb(data):
    mode = data.data

def main():
    rospy.init_node('pcv_base_fake')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, cmd_vel_cb)
    rospy.Subscriber("/mobile_base_controller/control_mode", Byte, ctrl_mode_cb)
    odom_broadcaster = tf.TransformBroadcaster()

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
        odom.child.frame_id = "base_link"
        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
        odom_pub.publish(odom)
        r.sleep()

if __name__ == "__main__":
    main()

