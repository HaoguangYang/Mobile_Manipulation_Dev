#!/usr/bin/env python
###################################################
# Author: Mythra Varun
# This script listens to the Hokuyo lidar laserscan data to localize using the amcl package. The waypoints are read from a file. They are updated at a rate of 10Hz and update is stopped if waypoint moves beyond 2m distance(self.waypt_lim).
###################################################

import rospy
from std_msgs.msg import String

# imports for laser scan data
from sensor_msgs.msg import LaserScan

# imports for reading map
from nav_msgs.msg import OccupancyGrid,MapMetaData

# imports for reading amcl outputs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Byte, Empty

# imports for math computations
import numpy as np
import math

from visualization_msgs.msg import Marker

import serial, os, signal
import tf

from msgsrv.msg import ctrlParams


class map_follower:

    # init constructor method for the class
    def __init__(self):

    #Car state and limits - Run/ Stop ...
        self.car_state = 0
        self.vlim = 1.0       # 0-1, scale/% of max velocity, velocity limit

## PD controller related:
    # Gains for the pd controller
        self.kp_gas = 180
        self.kd_gas = 200
        self.kp_steer = 0.9
        self.kd_steer = 4.2

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
        self.car_y = 0    # assume initially its placed at 0 from wall
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

    # Was serial successfully init flag
        self.serial_flag = False
    # Should serial be init flag
        self.ser_initq = False
    # The serial write string
        self.ser_write = Twist()

    # for loop timing
        self.current_time = 0
 # counter for gas rate
        self.gas_cnt = 0

 # Waypoint init
 # Array to load waypoints
        self.waypt_buf = np.zeros((2,2))
 # waypoint load rate
        self.waypt_rate = 10 #(Hz)
 # waypoint timing variables
        self.waypt_ptime = 0    #The previous time when waypint was updated
 # waypoint index
        self.waypt_i = 0
 # next desired x and y from waypt
        self.waypt_xd = 0
        self.waypt_yd = 0
 # distance between present pos and desired pos
        self.dis_err = 0
 # limit for waypoint update
        self.waypt_lim = 0.5  #2m limit, if dist between des and present is > waypt_lim then do not update
# Trajectory index
        self.traj_idx = -1
        self.running = False # flag for command publishing or not


#---------------------------------------------------------------------#
# The callback for laser scan data
    def scanCallback(self, data):
        min_angle = data.angle_min
        angle_inc = data.angle_increment
        # Compute distance from wall
        # Compute min distance from wall from 0 to pi/2
        left_wall_index_beg = int((math.pi/6-min_angle)/angle_inc)
        left_wall_index_end = int((math.pi/2-min_angle)/angle_inc)
        #scale wall distances to different max values
        range_scale = np.linspace(0.5,1\
        ,left_wall_index_end - left_wall_index_beg)
        a = 0
        b = 0
        ranges = [a*b for a,b in zip(range_scale,data.ranges[left_wall_index_beg:left_wall_index_end])]
        self.dist_frm_lwall = min(ranges)
        rght_wall_index = int((-math.pi/2-min_angle)/angle_inc)
        self.dist_frm_rwall = data.ranges[rght_wall_index]
#------------------------------------------------------------------------------------
# The mapping function
    def map_vel(self, inp, in_min, in_max, out_min, out_max):
        if(inp < in_min):
            return out_min
        elif(inp > in_max):
            return out_max
        else:
            return((inp-in_min)*(out_max-out_min)/(in_max-in_min)+out_min)
#------------------------------------------------------------------------------------
# The amcl callback
    def amcl_callback(self,data):
        self.car_ax = data.pose.pose.position.x
        self.car_ay = data.pose.pose.position.y
        car_ori = data.pose.pose.orientation
        car_eu_ori = tf.transformations.euler_from_quaternion([car_ori.x,car_ori.y,car_ori.z,car_ori.w])
        self.car_ayaw = car_eu_ori[2]
        rospy.loginfo([self.car_ax, self.car_ay, self.car_ayaw])

#------------------------------------------------------------------------------------
# The
    def ctrlprm_update(self,data):
        # ReSet gains
        self.kp_gas = data.gains[0]
        self.kd_gas = data.gains[1]
        self.kp_steer = data.gains[3]
        self.kd_steer = data.gains[4]
        # Reset state
        self.car_state = data.state
        # velocity %
        self.vlim = data.vel_lim
        print(data)

#-------------------------------------------------------------------------------------
# Waypoint file parser
    def parse_waypts(self):
        '''
            load and parse waypoints from file
        '''
        waypt_fname = '/home/cartman/Mobile_Manipulation_Dev/src/rallyUI/resources/ui_speedway.txt'
        self.trajs = []
        waypt_buf = []
        with open(waypt_fname, 'r') as f:
            for line in f.readlines():
                if line[:-1]=='---':
                    self.trajs.append(np.array(waypt_buf))
                    waypt_buf=[]
                else:
                    # print(float(line[:-1]))
                    waypt_buf.append([float(x) for x in (line[:-1]).split()])
        rospy.loginfo('The waypoints are loaded')
        rospy.loginfo(self.trajs)

    def button_cb(self,emp):
        if not self.running:
            self.traj_idx += 1
            self.waypt_buf = self.trajs[self.traj_idx]
            self.waypt_i = 0
            self.running = True

#-------------------------------------------------------------------------------------
# The main wall_following function
    def map_follow(self):
        # initializations
        rospy.init_node('map_follow',anonymous = True)
        pub = rospy.Publisher('waypts', Marker, queue_size = 100)
        cmd_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size = 1)
        mode_pub = rospy.Publisher('/mobile_base_controller/control_mode', Byte, queue_size = 1)
        # Set control mode 1- vel cont
        while not mode_pub.get_num_connections()    # hold there until the subsecribers are ready
        mode_pub.publish(Byte(data=1))

        #rospy.Subscriber("scan",LaserScan,self.scanCallback)
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        #rospy.Subscriber("map",OccupancyGrid,self.map_callback)
        rospy.Subscriber("/ov3/button", Empty , self.button_cb)
        rospy.Subscriber("control_params", ctrlParams, self.ctrlprm_update)
        rt = rospy.Rate(self.pd_rate)
        # Run controller param
        run_flag = rospy.get_param('runPD_q')
        print('Run Flag is', run_flag)

        self.parse_waypts()

        # for showing waypts on rviz
        wpt_marker = Marker()
        wpt_marker.header.frame_id = "/map"
        wpt_marker.type = wpt_marker.ARROW
        wpt_marker.action = wpt_marker.ADD
        wpt_marker.pose.orientation.x = 0
        wpt_marker.pose.orientation.y = 0
        wpt_marker.pose.orientation.z = 0
        wpt_marker.pose.orientation.w = 1.0
        wpt_marker.pose.position.x = 0
        wpt_marker.pose.position.y = 0
        wpt_marker.pose.position.z = 0
        wpt_marker.scale.x = 1
        wpt_marker.scale.y = 0.1
        wpt_marker.scale.z = 0.1
        wpt_marker.color.a = 0.8
        wpt_marker.color.r = 0.2;
        wpt_marker.color.g = 1.0;
        wpt_marker.color.b = 0.2;
        rospy.loginfo('************** Initialization complete ************')
        while not rospy.is_shutdown() and (run_flag==1):
            # compute distance between present pos estimate and latest waypt update
            self.dis_err = np.sqrt(math.pow((self.car_ax-self.waypt_xd),2)+math.pow((self.car_ay-self.waypt_yd),2))
            #rospy.loginfo(self.dis_err)
            # waypoint updates
            if(rospy.get_time()-self.waypt_ptime  > 0.1 and\
             self.dis_err < self.waypt_lim):
                self.waypt_ptime = rospy.get_time()
                self.waypt_xd =  self.waypt_buf[self.waypt_i,0]
                self.waypt_yd =  self.waypt_buf[self.waypt_i,1]
                #rospy.loginfo([self.waypt_xd,self.waypt_yd, 'wpt'])
                if(self.waypt_i < (len(self.waypt_buf) - 1)):
                    self.waypt_i += 1
                else:
                    self.running = False
                    cmd_pub.publish(Twist())
                 # waypt publishing
                wpt_marker.pose.position.x = self.waypt_xd
                wpt_marker.pose.position.y = self.waypt_yd
                #rospy.loginfo(wpt_marker)
            pub.publish(wpt_marker)
            prev_time = self.current_time    #compute the time for the loop
###########################
            # The pd control
            ###### Steer pd
            alpha = math.atan2(self.waypt_yd-self.car_ay,self.waypt_xd-self.car_ax)
            self.steer_err = alpha - self.car_ayaw

            if (self.steer_err > math.pi):
                self.steer_err = -2*math.pi + self.steer_err
            elif (self.steer_err < -math.pi):
                self.steer_err = 2*math.pi + self.steer_err
                # controller
            self.steer = self.kp_steer*self.steer_err \
            + self.kd_steer*(self.steer_err-self.psteer_err)
            ######## gas pd
            self.gas_err = self.dis_err
            # Controller
            self.gasvel = self.kp_gas*self.gas_err \
            + self.kd_gas*(self.gas_err-self.pgas_err)
            # Update previous errors
            self.pgas_err = self.gas_err
            self.psteer_err = self.steer_err
            # Scale the values and write
            # for steer we limit the scale to:
            # steer_limit = +/- max_dist_frm_wall*kp_steer = 0.5*5 = 2.5 + 0.5(for kd term)
            # This angle max is assumed to 30deg equivalent to 3(steer_limit)
            # self.steer = self.map_vel(self.steer, -math.pi/3, math.pi/3, self.steer_max, self.steer_min)
            # steer_wrt = int(self.steer)      # Steer to be written
            # self.gasvel = self.map_vel(self.gasvel, 0, 2048, 0, 2048) # change when controlling gas
            # gasvel_wrt = int(self.gasvel)*self.vlim    # gas velocity to be written
         # scale gasvel inversely to steer
         # gasvel_wrt = 0.08*(2048/(abs(steer_wrt)+10))*gasvel_wrt + 190
         # for low velocities needed set 0 every 6 cycles
            # if(abs(steer_wrt) > 1800):
            #     self.gas_cnt = self.gas_cnt+1
            #     if(self.gas_cnt == 6):
            #         self.gas_cnt = 0
            #         gasvel_wrt = 0
            #     else:
            #         gasvel_wrt = 400 * self.vlim
            # if(gasvel_wrt > 440):
            #     gasvel_wrt = 440 * self.vlim
            # Setup write string and write to serial port
            if self.car_state == 0:
                self.ser_write = Twist()
            else:
                self.ser_write.linear.x = self.gasvel
                self.ser_write.angular.z = self.steer
                #print(self.vlim)
                #print(self.ser_write)
            if self.running:
                #print('Running')
                cmd_pub.publish(self.ser_write)
            else:
                cmd_pub.publish(Twist())
            rt.sleep()
            run_flag = rospy.get_param('runPD_q')
        if rospy.is_shutdown() or (run_flag==0):
            print('Sutting down')
            try:
                print('Setting velocities to zero')
                cmd_pub.publish(Twist())
                mode_pub.publish(Byte(data=0))
            except:
                print('Error closing serial port, do it manually')
        if (run_flag==0):
            rospy.set_param('runPD_q', 1)
        #    os.kill(os.getppid(), signal.SIGHUP)

if __name__ == '__main__':
    try:
        car = map_follower()
        car.map_follow()
    except rospy.ROSInterruptException:
        pass
