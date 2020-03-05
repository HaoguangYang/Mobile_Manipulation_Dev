#!/usr/bin/env python
###################################################
# Authors: Mythra Varun 
# This script reads imu raw data and publishes filtered imu data 
# using madgwick filter ..
###################################################

import rospy
from std_msgs.msg import String

# imports for laser scan data
from sensor_msgs.msg import Imu

# imports for math computations
import numpy as np
import math
import time

class ImuFilt():
	def __init__(self, topicnames):
		'''
			@topicnames: list of imut raw topic names to subscribe to
		'''
		
	def
	
