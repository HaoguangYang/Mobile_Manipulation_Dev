#!/usr/bin/env python

import os, time, multiprocessing
import subprocess
from subprocess import check_output
import rospy
#from move_base_msgs.msg import MoveBaseActionResult
#from geometry_msgs.msg import Twist

#from std_msgs.msg import Header, Byte
#from sensor_msgs.msg import LaserScan
#import serial
# comment out payload if no Arduino-related payload (i.e. /dev/ttyACM0) exists.
#from Payloads.UVC import payload
from Tasks.uvcDisinfection import Task
import roslaunch

#def status_cb(data):
#    status = data.status.status
#    if status == 3:
        #susp_time = calc_disinfection_time(scan_cur)
#        auto_susp_cmd = 'sudo rtcwake -u -s '+ str(susp_time) +' -m mem'
#        os.system(auto_susp_cmd)
        # program resumes here after waking up
        # turn off lamp command to be added
#        i = i+1
        #pub_goal.Publish(goals_list[i])
#    elif status == 4:
        # call for help procedures
#        manual_control = 1

#def cmd_vel_cb(data):
#    cmd_vel_cur = data

#def cmd_vel_tele_cb(data):
#    cmd_vel_tele_cur = data
    
#def scan_cb(data):
#    scan_cur = data.ranges
    
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch_baseNode = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/pcv_node.launch'])

def checkBaseRunning():
    restartFlag = False
    try:
        while 1:
            try:
                check_output(["pidof","pcv_base_node"])
            except subprocess.CalledProcessError:
                # thread is not running
                restartFlag = True
                launch_baseNode.start()
            time.sleep(1)
            #print('Watchdog active!')
    except:
        if restartFlag:
            launch_baseNode.shutdown()
        pass

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
    
    roslaunch.configure_logging(uuid)
    # launch the stack of stuff
    #nav_thread = multiprocessing.Process(target=os.system, args=('roslaunch pcv_base run_nav_only.launch',))
    #nav_thread.daemon = True
    #nav_thread.start()
    #time.sleep(30)      # wait for the stack to start up
    
    # task is already initialized.
    
    launch_video = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/includes/realsense.launch'])
    launch_video.start()
    
    while (1):
        while not (Task.isReady()):
            if (os.path.exists('/dev/input/js0')):  # if a joystick is plugged in...
                time.sleep(2)
                launch = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/joystick_teleop.launch', './src/pcv_base/launch/pcv_node.launch'])
                launch.start()
                while (os.path.exists('/dev/input/js0')):
                    pass
                launch.shutdown()

        # Continue ONLY when the button is pressed
        if (Task.isReady()):
            task_thread = multiprocessing.Process(target=Task.run, args=(launch_baseNode,))
            task_thread.daemon = True
            task_thread.start()
            time.sleep(20)
            base_watchdog_thread = multiprocessing.Process(target=checkBaseRunning, args=())
            base_watchdog_thread.daemon = True
            base_watchdog_thread.start()
            while (not Task.chkFault()):
                pass
            #payload.turnOffUVC()
            base_watchdog_thread.terminate()
            #payload.setDoneStatus()
            task_thread.terminate()
    
    Task.end()
    launch_video.shutdown()
    
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
