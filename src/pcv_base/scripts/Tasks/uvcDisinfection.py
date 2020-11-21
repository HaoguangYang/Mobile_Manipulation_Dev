#!/usr/bin/env python

import os, time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from Payloads.UVC import payload

class uvcDisinfectionTask():
    def __init__(self):
        payload.initialize()
        
    def run(self, launch_baseNode):
        launch = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/run_nav_only.launch'])
        launch.start()

        rospy.init_node('admin')
        
        pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        while not pose_pub.get_num_connections():
            pass
        #init_pose = [ 0.0453101396561, 0.00364243984222, 0.0, 0.0, 0.0, -0.032842760778, 0.999460531019]
        init_pose = [1.824850559234619141e+00, 2.446069240570068359e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, -6.975731935691269481e-01, 7.165135306564485163e-01]
        # reads keyboard input. considers changing to read button status.
        #ret = input('Is robot in position? (1-Yes, 0-NO)')
        #if ret==0:
        #    rospy.signal_shutdown('Exit')
        
        # initialize position in the map.
        ipose = PoseWithCovarianceStamped()
        ipose.header.frame_id="map"
        ipose.pose.pose.position.x = init_pose[0]
        ipose.pose.pose.position.y = init_pose[1]
        ipose.pose.pose.orientation.z = init_pose[5]
        ipose.pose.pose.orientation.w = init_pose[6]
        pose_pub.publish(ipose)
        
        time.sleep(15)
        launch_baseNode = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/pcv_node.launch'])
        launch_baseNode.start()
        time.sleep(15)

        pose_pub.publish(ipose)
        time.sleep(1)
        
        try:
            # robot starts moving... turn onUVC.
            payload.turnOnUVC()
            s = os.system('rosrun pcv_base pubgoal.py _location:=EntranceToLivingRoom')

            if (s == 0):
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
                s = os.system('rosrun pcv_base pubgoal.py _location:=ReturnToEntrance')
        
            if (s == 0):
                print('Done!')
            else:
                print('HELP NEEDED!')
                message = payload.sms_client.messages \
                          .create(
                                body='FIXME: Robot got stuck, task is INCOMPLETE!',
                                from_=self.sms_from, # this is my twilio number
                                to=self.sms_to # this is my number
                          )
            # help code...
        
        finally:
            payload.turnOffUVC()
            payload.setDoneStatus()
        
            #nav_thread.terminate()
            #robot_thread.terminate()
            launch.shutdown()
            launch_baseNode.shutdown()
        
    def isReady(self):
        return payload.isReady()
    
    def chkFault(self):
        return (payload.isReady() and payload.isOn())
        
    def end(self):
        payload.payload_thread.terminate()
        
Task = uvcDisinfectionTask()
