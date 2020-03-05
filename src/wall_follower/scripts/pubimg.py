#!/usr/bin/env python

import rospy
import cv2, numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('pubimg', anonymous=True)
pub = rospy.Publisher('/camimg', Image, queue_size=10)

cap = cv2.VideoCapture(0)
bridge = CvBridge()

while not rospy.is_shutdown():
    _,frame = cap.read()
    rgb_fr = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    cv_img = bridge.cv2_to_imgmsg(rgb_fr, encoding="passthrough")
    pub.publish(cv_img)
    cv2.waitKey(2)
cap.release()





