#!/usr/bin/env python
import serial
import sys
import rospy
from std_msgs.msg import Empty

PORT = '/dev/ttyUSB0'
btn_flag = 0        # Flag to indicate if button was pressed
ser = serial.Serial(PORT, 9600, timeout=0.1)
rospy.init_node('traj_button', anonymous=True)
btn_pub = rospy.Publisher('/ov3/button', Empty, queue_size=3)

while not rospy.is_shutdown():
    if ser.in_waiting>0:
        c = ser.read()
        #print(c)
        if len(c)>0 and ord(c)==0:
            btn_flag = (btn_flag+1)%2
            btn_pub.publish(Empty())
