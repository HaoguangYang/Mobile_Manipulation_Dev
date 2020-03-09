#!/usr/bin/env python
###################################################
# Authors: Mythra Varun 
# This script reads and writes to the serial port. The read data is published to a topic named "ser_read". The data to be written is read from the "ser_wrt" topic. 
###################################################

import rospy, serial, time
from std_msgs.msg import String


class serialport_rw:
    
    # init constructor method for the class
    def __init__(self):
        #Serial initialization parameters
        self.port_name = '/dev/ttyACM0'
        self.baud = 115200
        # Serial data buffers
        self.read_buf = ''
        self.wrt_buf = 'A+0000+0000'
        rospy.init_node('serial_rw',anonymous = True)
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.01)
            self.ser.close()
            self.ser.open()
            self.serflag = True
            rospy.loginfo('Serial port access successful')
        except rospy.ROSInterruptException:
            rospy.loginfo('Error in opening serial port')
            rospy.signal_shutdown('Shutting down')      
        #Serial loop rate
        self.ser_rate = 200
        
        
        
# The callback function to write data to serial port
    def wrt_cb(self,data):
        self.wrtflag = True
        self.wrt_buf = data.data
        self.ser.write(self.wrt_buf)
        #rospy.loginfo(self.wrt_buf)
        
# The main serial function    
    def ser_rw(self):
        time.sleep(3);
        # initializations
        
        pub2 = rospy.Publisher('imu2_dat', String, queue_size = 100)
        if self.serflag:
        # Flush the serial port once in the beginning
            self.ser.read(self.ser.inWaiting())
            rospy.loginfo('Serial port flushed')
            rospy.loginfo(self.ser.write('IMU1'))    
        rospy.Subscriber("ser_wrt", String, self.wrt_cb)
        rt = rospy.Rate(self.ser_rate)
        
        while not rospy.is_shutdown():
            #self.ser.write('A+0300+0400')
            self.read_buf = self.read_buf + self.ser.read_until('EN'); 
            if self.ser.inWaiting()>1000:
                print(self.ser.inWaiting())  
            if len(self.read_buf) >= 7:
                beg = self.read_buf.find('I')
                self.read_buf = self.read_buf[beg:]
                if (len(self.read_buf)>=7 and self.read_buf[5:7] == 'EN'):
                    pub2.publish(self.read_buf[1:5])
                    self.read_buf=self.read_buf[7:]
            rt.sleep();
        self.ser.close()  

if __name__ == '__main__':
    try:
        serobj = serialport_rw()
        serobj.ser_rw()
    except rospy.ROSInterruptException:
        print("Error in starting ")
        pass 
