#!/usr/bin/env python
###################################################
# Authors: Mythra Varun 
# This script reads and writes to the serial port. The read data is published to a topic named "ser_read". The data to be written is read from the "ser_wrt" topic. 
###################################################

import rospy
from std_msgs.msg import String

# imports for laser scan data
from sensor_msgs.msg import Imu

# imports for reading map
from nav_msgs.msg import OccupancyGrid,MapMetaData

# imports for reading amcl outputs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

# imports for math computations
import numpy as np
import math
import time

from visualization_msgs.msg import Marker 

import serial
import tf

class serialport_rw:
    
    # init constructor method for the class
    def __init__(self):
        #Serial initialization parameters
        self.port_name = '/dev/ttyACM0'
        self.baud = 115200
        # Serial data buffers
        self.read_buf = ''
        self.wrt_buf = 'A+0000+0000'
        self.wrtflag = False
        
        #Serial loop rate
        self.ser_rate = 333
        self.serflag = False
        # Data descriptors
        self.word_len = 27 # Ex: I1+00021+00025+16482+02002U 
        rospy.init_node('serial_rw',anonymous = True) 
    #    rospy.on_shutdown(self.shtdwn)       
        
# The callback function to write data to serial port
    def wrt_cb(self,data):
        self.wrtflag = True
        self.wrt_buf = data.data
    
        #rospy.loginfo(self.wrt_buf)
# The main serial function    
    def ser_rw(self):
        # initializations

        pub1 = rospy.Publisher('imu1_dat', Imu, queue_size = 100)
        pub2 = rospy.Publisher('imu2_dat', Imu, queue_size = 100)
        pub3 = rospy.Publisher('imu3_dat', Imu, queue_size = 100)
        rospy.Subscriber("ser_wrt", String, self.wrt_cb)
        rt = rospy.Rate(self.ser_rate)
        # imu message 
        imu_msg = Imu()
        # Port setup
        try:
            ser = serial.Serial(self.port_name,self.baud,timeout=0.01)
            #ser.close()
            #ser.open()
            self.serflag = True
            rospy.loginfo('Serial port access successful')
            time.sleep(3);
        except rospy.ROSInterruptException:
            rospy.loginfo('Error in opening serial port')
            rospy.signal_shutdown('Shutting down')
        #index variables
        beg = 0
        en = 0
        rospy.loginfo(self.serflag)
        if self.serflag:
        # Flush the serial port once in the beginning
            ser.read(ser.inWaiting())
            rospy.loginfo('Serial port flushed')
            rospy.loginfo(ser.write('IMU2'))    
       
        # Loop begin 
            while not rospy.is_shutdown():
            #Check if there is data to be written and write data
                if self.wrtflag:
                    ser.write(self.wrt_buf)
                    self.wrtflag = False
            #Check if there is data to be read, if so write
                if ser.inWaiting(): 
                    self.read_buf = self.read_buf + ser.read(ser.inWaiting());
                    if len(self.read_buf) >= self.word_len:
                        beg = self.read_buf.find('I')
                        self.read_buf = self.read_buf[beg:]
                        if (len(self.read_buf)>=self.word_len and self.read_buf[self.word_len-1] == 'U'):
                            if(self.read_buf[1]=='0'):
                                pub1.publish(self.read_buf[0:self.word_len-1]);
                            elif(self.read_buf[1]=='1'):
                                pub2.publish(self.read_buf[0:self.word_len-1]);
                            elif(self.read_buf[1]=='2'):
                                pub3.publish(self.read_buf[0:self.word_len-1]);
                            self.read_buf=self.read_buf[self.word_len:]
                rt.sleep();
            ser.write('IMU0')
    
if __name__ == '__main__':
    try:
        serobj = serialport_rw()
        serobj.ser_rw()
    except rospy.ROSInterruptException:
        print("Error in starting ")
        pass 
