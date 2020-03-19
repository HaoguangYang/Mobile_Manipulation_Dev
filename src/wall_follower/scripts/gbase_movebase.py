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
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path

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

#from msgsrv.msg import ctrlParams

from rospkg import RosPack


class map_follower:

    # init constructor method for the class
    def __init__(self):

    #Car state and limits - Run/ Stop ...
        self.car_state = 0

    # Rate for the final command
        self.cmd_rate = 100

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

    # minimum distance from obstacles. This triggers an emergency stop.
        self.dist_lim = 0.2
        
## Communication related
    # For serial initializations
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200      # bits/s

    # Was serial successfully init flag
        self.serial_flag = False
    # Should serial be init flag
        self.ser_initq = False

    # for loop timing
        self.current_time = 0

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
        
        # if something is too close
        self.intrusion = False
        
        # movebase
        self.viaPts = Path()
        self.pubVia = rospy.Publisher('via_points', Path, queue_size = 1)
        self.mode_pub = rospy.Publisher('/mobile_base_controller/control_mode', Byte, queue_size = 1)
        self.scan_min_ang = -1.
        self.scan_max_ang = 1.
        self.scan_ang_inc = 0.25/180*np.pi
        # barrier of the robot: intrusion within 30cm front of the laser center will induce an emergency shutdown.
        self.shape = 0.3/np.cos(np.arange(self.scan_min_ang, self.scan_max_ang, self.scan_ang_inc))

#---------------------------------------------------------------------#
# The callback for laser scan data
    def scanCallback(self, data):
    	# detect if any obstacle within dist_lim range
        #min_angle = data.angle_min
        #angle_inc = data.angle_increment
        ranges = np.array(data.ranges)
        intrusion = np.sum(np.less(ranges, self.shape))
        if intrusion > 2:
            self.intrusion = True
        else:
            self.intrusion = False

#------------------------------------------------------------------------------------
# The amcl callback
    def amcl_callback(self,data):
        self.car_ax = data.pose.pose.position.x
        self.car_ay = data.pose.pose.position.y
        car_ori = data.pose.pose.orientation
        car_eu_ori = tf.transformations.euler_from_quaternion([car_ori.x,car_ori.y,car_ori.z,car_ori.w])
        self.car_ayaw = car_eu_ori[2]
        rospy.loginfo([self.car_ax, self.car_ay, self.car_ayaw])

#-------------------------------------------------------------------------------------
# Waypoint file parser
    def parse_waypts(self):
        '''
            load and parse waypoints from file
        '''
        waypt_fname = RosPack().get_path('wall_follower')+'/resources/ui_speedway.txt'
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
            self.viaPts.header.seq = self.traj_idx
            self.viaPts.header.stamp = rospy.Time.now()
            self.viaPts.header.frame_id = "map"
            pose = PoseStamped()
            pose.header = self.viaPts.header
            pose.header.seq = 0
            for rows in self.waypt_buf:
                pose.pose = Pose(position = Point(x=rows[0], y=rows[1]),
                                 orientation = tf.transformations.quaternion_from_euler(0, 0, rows[2]))
                self.viaPts.poses.append(pose)
                pose.header.seq += 1
            self.pubVia.publish(self.viaPts)
            self.running = True

#-------------------------------------------------------------------------------------
# The main map_following function
    def map_follow(self):
        # initializations
        rospy.init_node('map_follow',anonymous = True)
        pubUI = rospy.Publisher('waypts', Marker, queue_size = 100)
        # cmd_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size = 1)
        
        # Set control mode 1- vel cont
        while not self.mode_pub.get_num_connections():    # hold there until the subsecribers are ready
            pass
        self.mode_pub.publish(Byte(data=1))

        rospy.Subscriber("scan",LaserScan,self.scanCallback)
        #rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        #rospy.Subscriber("map",OccupancyGrid,self.map_callback)
        rospy.Subscriber("/ov3/button", Empty , self.button_cb)
        #rospy.Subscriber("control_params", ctrlParams, self.ctrlprm_update)
        rate = rospy.Rate(self.cmd_rate)
        # Run controller param
        run_flag=1
        #run_flag = rospy.get_param('runPD_q')
        #print('Run Flag is', run_flag)

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
            #self.dis_err = np.sqrt(math.pow((self.car_ax-self.waypt_xd),2)+math.pow((self.car_ay-self.waypt_yd),2))
            #rospy.loginfo(self.dis_err)
            # waypoint updates
            if(rospy.get_time()-self.waypt_ptime  > 0.1):
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
            pubUI.publish(wpt_marker)
            
            # TODO: make LED on button go off instead...
            #if self.running:
            #    print('Running')
            #    cmd_pub.publish(self.ser_write)
            #else:
            #    cmd_pub.publish(Twist())
            
            rt.sleep()
            run_flag = rospy.get_param('runPD_q')
        if rospy.is_shutdown() or (run_flag==0):
            print('Sutting down')
            try:
                print('Setting velocities to zero')
                cmd_pub.publish(Twist())
                self.mode_pub.publish(Byte(data=0))
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
