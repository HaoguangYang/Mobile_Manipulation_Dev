#!/usr/bin/env python

import os, pickle, time, multiprocessing
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Byte
from sensor_msgs.msg import LaserScan
import serial
from payload import payload
import roslaunch

def status_cb(data):
    status = data.status.status
    if status == 3:
        #susp_time = calc_disinfection_time(scan_cur)
        auto_susp_cmd = 'sudo rtcwake -u -s '+ str(susp_time) +' -m mem'
        os.system(auto_susp_cmd)
        # program resumes here after waking up
        # turn off lamp command to be added
        i = i+1
        #pub_goal.Publish(goals_list[i])
    elif status == 4:
        # call for help procedures
        manual_control = 1

def cmd_vel_cb(data):
    cmd_vel_cur = data

def cmd_vel_tele_cb(data):
    cmd_vel_tele_cur = data
    
def scan_cb(data):
    scan_cur = data.ranges

if __name__=="__main__":
    # start ROS core
    #os.system('roscore &')
    #pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
    #pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 1)
    #sub_cmd = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)
    #sub_cmd_tele = rospy.Subscriber('/cmd_vel_tele', Twist, cmd_vel_tele_cb)
    #sub_stat = rospy.Subscriber('/move_base/result', MoveBaseActionResult, status_cb)
    #sub_scan = rospy.Subscriber('/scan',Scan, scan_cb)    
    os.system('. devel/setup.sh')
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # launch the stack of stuff
    #nav_thread = multiprocessing.Process(target=os.system, args=('roslaunch pcv_base run_nav_only.launch',))
    #nav_thread.daemon = True
    #nav_thread.start()
    #time.sleep(30)      # wait for the stack to start up
    
    payload.initialize()
    
    while not (payload.isReady()):
        if (os.path.exists('/dev/input/js0')):  # if a joystick is plugged in...
            time.sleep(2)
            launch = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/joystick_teleop.launch', './src/pcv_base/launch/pcv_node.launch'])
            launch.start()
            while (os.path.exists('/dev/input/js0')):
                pass
            launch.shutdown()
    # Continue ONLY when the button is pressed
    
    launch = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/run_nav_only.launch'])
    launch.start()
    time.sleep(15)
    launch_baseNode = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/pcv_node.launch'])
    launch_baseNode.start()
    time.sleep(15)
    
    rospy.init_node('admin')

    pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
    init_pose = [ 0.0453101396561, 0.00364243984222, 0.0, 0.0, 0.0, -0.032842760778, 0.999460531019]
    
    # reads keyboard input. considers changing to read button status.
    #ret = input('Is robot in position? (1-Yes, 0-NO)')
    #if ret==0:
    #    rospy.signal_shutdown('Exit')
    
    # initialize position in the map.
    ipose = PoseWithCovarianceStamped()
    ipose.pose.pose.position.x = init_pose[0]
    ipose.pose.pose.position.y = init_pose[1]
    ipose.pose.pose.orientation.z = init_pose[5]
    ipose.pose.pose.orientation.w = init_pose[6]
    pose_pub.publish(ipose)
    time.sleep(1)
    
    # robot starts moving... turn onUVC.
    payload.turnOnUVC()
    s = os.system('rosrun pcv_base pubgoal.py _location:=LivingRoomClean')
    
    if (s == 0):
        s = os.system('rosrun pcv_base door_servo.py _location:=Bedroom1Door _direction:=0')
    
    if (s == 0):
        s = os.system('rosrun pcv_base pubgoal.py _location:=Bedroom1Clean')
    
    if (s == 0):
        s = os.system('rosrun pcv_base pubgoal.py _location:=Bedroom1PostClean')
    
    if (s == 0):
        s = os.system('rosrun pcv_base door_servo.py _location:=Bedroom1Door _direction:=1')

    if (s == 0):
        s = os.system('rosrun pcv_base pubgoal.py _location:=Bedroom1ToBedroom2')
    
    if (s == 0):
        s = os.system('rosrun pcv_base door_servo.py _location:=Bedroom2Door _direction:=0')

    if (s == 0):
        s = os.system('rosrun pcv_base pubgoal.py _location:=Bedroom2Clean')

    if (s == 0):
        s = os.system('rosrun pcv_base pubgoal.py _location:=Bedroom2PostClean')
    
    if (s == 0):
        s = os.system('rosrun pcv_base door_servo.py _location:=Bedroom2Door _direction:=1')

    if (s == 0):
        s = os.system('rosrun pcv_base pubgoal.py _location:=ReturnToKitchen')
    
    if (s == 0):
        print('Done!')
    else:
        print('HELP NEEDED!')
        # help code...
    payload.turnOffUVC()
    payload.setDoneStatus()
    
    #nav_thread.terminate()
    #robot_thread.terminate()
    launch.shutdown()
    launch_baseNode.shutdown()
    payload.payload_thread.terminate()
    
    #rate = rospy.rate(50)
    # Admin node functionalities:
    #   subscribes to: (move_base)-->cmd_vel, (teleop)-->cmd_vel_tele
    #   publishes to:  mobile_base_controller/cmd_vel-->(pcv_base_base)
    #   subscribes to: (move_base)-->move_base/result: 2 - goal updated; 3 - goal reached; 4 - failed
    #   publishes to:  teleop interface
    #   publishes to:  move_base_simple/goal -->(movebase)
    # if no cmd_vel published within 5 seconds after submission of goal, treat it as failed.
    # if failed, call for help.
    # if goal reached, compute disinfection time from scan (maybe we should use the localization data instead, since scan only covers a small area), then compose a suspend command and sent through os.system to suspend the machine for that long.
    
    #i = 0
    
    #pub_goal.Publish(goals_list[i])
    
    #while (1)       # while end of goal list is not reached
    #    if (not manual_control)
    #        pub_cmd.Publish(cmd_vel_cur)
    #    else
    #        pub_cmd.Publish(cmd_vel_tele_cur)
        # else do nothing
    #    rate.sleep()
