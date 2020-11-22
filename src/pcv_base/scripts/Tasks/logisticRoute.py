#!/usr/bin/env python

import os, time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from Payloads.Generic import payload
import roslaunch

class Task():
    def __init__(self, uuid):
        payload.initialize()
        self.uuid = uuid
        self.delayStart = 0
        
    def runLogistics(self):
        launch = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/run_nav_only.launch'])
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
        
        time.sleep(self.delayStart)
        #launch_baseNode = roslaunch.parent.ROSLaunchParent(uuid,['./src/pcv_base/launch/pcv_node.launch'])
        #launch_baseNode.start()
        #time.sleep(15)

        #pose_pub.publish(ipose)
        #time.sleep(1)
        
        """
        # THIS SEGMENT SERVES AS A TEMPLATE FOR NOW...
        s = os.system('rosrun pcv_base pubgoal.py _location:=RouteASeg1')

        if (s == 0):
            payload.pause()
            while not(payload.isRunning()):
                pass
            s = os.system('rosrun pcv_base pubgoal.py _location:=RouteASeg2')
       
        if (s == 0):
            payload.pause()
            while not(payload.isRunning()):
                pass
            s = os.system('rosrun pcv_base pubgoal.py _location:=RouteASeg3')
    
        if (s == 0):
            s = os.system('rosrun pcv_base door_servo.py _location:=RouteANarrowPass1 _direction:=0')

        if (s == 0):
            time.sleep(30)
            s = os.system('rosrun pcv_base door_servo.py _location:=RouteANarrowPass1 _direction:=1')
    
        if (s == 0):
            payload.pause()
            while not(payload.isRunning()):
                pass
            s = os.system('rosrun pcv_base pubgoal.py _location:=RouteASeg4')
        """

        if (s == 0):
            print('Done!')
        else:
            print('HELP NEEDED!')
            payload.sendSMS('FIXME: Robot got stuck, task is INCOMPLETE!')
            # help code...
        
        try:
            pass
        finally:
            payload.setDoneStatus()
            #nav_thread.terminate()
            #robot_thread.terminate()
            launch.shutdown()
        
    def isReady(self):
        return payload.isReady()
    
    def chkFault(self):
        return not payload.isReady()

    def start(self):
        self.task_thread = multiprocessing.Process(target=self.runLogistics, args=())
        self.task_thread.daemon = True
        self.task_thread.start()

    def stop(self):
        self.task_thread.terminate()
        
    def end(self):
        payload.payload_thread.terminate()
        
if __name__ == '__main__':
    task = Task()
    task.runLogistics()