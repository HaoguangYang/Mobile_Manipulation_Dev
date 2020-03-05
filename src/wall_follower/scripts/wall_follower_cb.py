#!/usr/bin/env python

###################################################
# Authors: Mythra Varun 
# This script listens to the Hokuyo lidar laserscan uses it to follow the wall
# The script subscribes to the lidar scan topic which publishes data in sensor_msgs/LaserScan.msg format
# We find the range at the right angle to vehicle which is also the Pi/2 reading of the laserScan data (Since z is upward and x is forward, so left right angle is Pi/2)
# The position of this data in the ranges array is ((3.14159265/2)+2.3561945)/0.0043633231 = 900; where -2.3561945 to 2.3561945 is the minimum and maximum angle of laserScan, and 0.0043633231 is the angle increment 
# The rally car follows the wall on the left
# The car runs PD control at laserscan callback rates.
###################################################

import rospy
from std_msgs.msg import String

# imports for laser scan data
from sensor_msgs.msg import LaserScan

# imports for reading map
from nav_msgs.msg import OccupancyGrid

# imports for reading amcl outputs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

# imports for math computations
import numpy as np
import math

import serial
import tf


class wall_follower:
    
    # init constructor method for the class
    def __init__(self):
    
## PD controller related: 
    # Gains for the pd controller 
        self.kp_gas = 10
        self.kd_gas = 1
        self.kp_steer = 10
        self.kd_steer = 100
    
    # Rate for the pd loop
        self.pd_rate = 100 
    
    # Errors in car gas and steer
        self.gas_err = 0
        self.pgas_err = 0   # previous error
        self.steer_err = 0
        self.psteer_err = 0 # previous error

## Car positions and velocities
    # Car position estimates w.r.t map (in m)
        self.car_x = 0  
        self.car_y = 0.5    # assume initially its placed at 0.5 from wall 
        self.car_px = 0     # car previous positions  
        self.car_py = 0     
        self.car_yaw = 0
        
    #   Car position and orientation from amcl
        self.car_ax = 0
        self.car_ay = 0
        self.car_ayaw = 0    
        
    # velocities to send
        self.steer = 0.0
        self.gasvel = 0.0
    # gas scale - gas value to m/s (2048 -> m/s)
        self.gas_scale = 0.0004
## Problem statement related    
    # For wall following, desired distance from wall (in m)
        self.dist_des = 0.5
    # Left and right wall distances
        self.dist_frm_lwall = 0
        self.dist_frm_rwall = 0
## Communication related
    # For serial initializations
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200      # bits/s
    
    # Scaling for arduino scale
    # The gas and steer computed should be scaled to:
        self.steer_min = -2048      # equivalent to ~-50 deg
        self.steer_max = 2048      # equivalent to ~50 deg
        self.gas_min = -2048
        self.gas_max = 2048  
    #Serial handle
        self.ser_handle = serial.Serial(self.port, self.baudrate)
    # serial init flag
        self.serial_flag = False
    # The serial write string
        self.ser_write = 'A+0000+0000'
    # for loop timing
        self.current_time = 0
 # counter for gas rate
        self.gas_cnt = 0
    
#---------------------------------------------------------------------#
# The callback for laser scan data
    def scanCallback(self, data):
        min_angle = data.angle_min
        angle_inc = data.angle_increment        
        # Compute distance from wall
        # Compute min distance from wall from pi/6 to pi/2
        left_wall_index_beg = int((math.pi/6-min_angle)/angle_inc)
        left_wall_index_end = int((math.pi/2-min_angle)/angle_inc)
        # Set up scale for desired wall distances at different angles
        range_scale = np.linspace(0.5,1\
        ,left_wall_index_end - left_wall_index_beg)
        a = 0
        b = 0
        # Scale the distances to different desired dist.
        ranges = [a*b for a,b in zip(range_scale,data.ranges[left_wall_index_beg:left_wall_index_end])]
        self.dist_frm_lwall = min(ranges)
        # The distance to the wall along -ve x axis (in car frame) 
        rght_wall_index = int((-math.pi/2-min_angle)/angle_inc)
        self.dist_frm_rwall = data.ranges[rght_wall_index]
        print('[At pi/2, At-pi/2 ]',[data.ranges[left_wall_index_end],self.dist_frm_rwall])
        ######################
        # The PD control
        # The steer pd control
        self.steer_err = self.dist_frm_lwall - self.dist_des    # Find steer error
        self.steer = self.kp_steer*self.steer_err \
        + self.kd_steer*(self.steer_err-self.psteer_err)        # The PD
            # Gas pd
        self.gas_err = 20 # gas_error constant, Since we want to run at constant speed 
        self.gasvel = self.kp_gas*self.gas_err \
        + self.kd_gas*(self.gas_err-self.pgas_err)
            # Update previous errors
        self.pgas_err = self.gas_err
        self.psteer_err = self.steer_err
            # Scale the values and write    
            # For steer error we limit the scale to:
            # steer_limit = +/- max_dist_frm_wall*kp_steer = 0.5*5 = 2.5 
            # This angle max is assumed to be 50deg equivalent to 3(steer_limit)
        self.steer = self.map_vel(self.steer, -0.5*self.kp_steer, 0.5*self.kp_steer, self.steer_max, self.steer_min)
        steer_wrt = int(self.steer)      # Steer to be written, integer
        self.gasvel = self.map_vel(self.gasvel, 0, 2048, 0, 2048) # change when controlling gas
        gasvel_wrt = int(self.gasvel)    # gas velocity to be written
            # Have pulsed powering for large angle turns
            # for low velocities needed set 0 every 10 cycles
        if(abs(steer_wrt) > 1800):
            self.gas_cnt = self.gas_cnt+1
            if(self.gas_cnt == 4):
                self.gas_cnt = 0
                gasvel_wrt = 0
            else:
                gasvel_wrt = 400
                # Setup write string and write to serial port
        self.ser_write = "A%+05d%+05d" %(steer_wrt, gasvel_wrt)            
        #rospy.loginfo(self.ser_write)
        if(self.serial_flag):
            self.ser_handle.write(self.ser_write) 
            # Timing
        prev_time = self.current_time    #compute the time for the loop
        self.current_time = rospy.get_time()
        print 'Time between Callbacks:'
        rospy.loginfo(self.current_time - prev_time)
           
#------------------------------------------------------------------------------------
# The mapping function
    def map_vel(self, inp, in_min, in_max, out_min, out_max):
        if(inp < in_min):
            return out_min
        elif(inp > in_max):
            return out_max
        else:
            return((inp-in_min)*(out_max-out_min)/(in_max-in_min)+out_min)
        
#-------------------------------------------------------------------------------------        
# The main wall_following function
    def wall_follow(self):
        # initializations
        rospy.init_node('wall_follow',anonymous = True)
        rospy.Subscriber("scan",LaserScan,self.scanCallback)
         
        # initialize serial
        try:
            self.ser_handle.close()
            self.ser_handle.open()
            self.serial_flag = True     # Set serial flag to identify succesful port opening
        except:
            print('Error opening serial port')
            rospy.signal_shutdown("Sutdown due to serial port error, check serial device");
        rospy.on_shutdown(self.shut_dwn);
        rospy.spin()

    def shut_dwn(self):
        print('Sutting down')
        try:
            print('Setting velocities to zero')
            self.ser_handle.write('A+0000+0000')
            print('Closing serial port')
            self.ser_handle.close()
            print('Done!!')
        except:
            print('Error closing serial port, do it manually')
        
if __name__ == '__main__':
    try:
        car = wall_follower()
        car.wall_follow()
    except rospy.ROSInterruptException:
        pass 

