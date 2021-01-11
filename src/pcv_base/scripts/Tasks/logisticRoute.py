#!/usr/bin/env python

import os, time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Byte
from Payloads.Generic import payload
import numpy as np
import roslaunch
import multiprocessing
import tokenize
from definedFunctions import *

class Task():
    def __init__(self, uuid, location, init_pos = 1):
        payload.initialize()
        self.uuid = uuid
        self.pathName = '/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/traj/'+location+'.txt'
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/run_nav_only.launch'])
        if not (os.path.exists(self.pathName)):
            sys.exit('ERROR: '+self.pathName+' Does Not Exist! Aborting...')
        self.traj = np.loadtxt(self.pathName)
        if self.traj.ndim<2:                # in case of only one line in the traj...
            self.traj = np.expand_dims(self.traj,0)
        fileObj = open(self.pathName, 'r')
        
        """
        format of waypoint file for Omniveyor tasks:
        x,y,z,quat  # target_label, target_operation_function, target_operation_function_param1, param2, ...
        
        target_label:               identifier of the dispatch point for a specific interaction target
        target_operation_function:  call this function when the robot reaches the waypoint
        params (optional):          passed into the operation function.
        Keywords follow Python conventions.
        
        Special flow control within target_operation_function:
        via:    via point, no action, proceed to next waypoint.
        pause:  pause at the point, until the payload initiates a resume signal,
                    or a timer has elapsed.
        cycle:  cycle mark, no action, return to the start of file.
        end:    ending mark, no action, only recognized at eof.
        
        example:
        1.0, -1.0, 0.,0.,0.707, 0.707   # A, dock_procedure_A, target_image.jpg
        2.0, -2.0, 0.,0.,1.0, 0.0       # None, via
        0.0, 0.0, 0.,0., -1.0, 0.0      # None, cycle
        """
        # Lists for [target, action] pairs
        self.targetActionList = []
        for toktype, tok, start, end, line in tokenize.generate_tokens(fileObj.readline):
            if toktype == tokenize.COMMENT:
                item = [obj.strip() for obj in tok.split(",")]
                assert (len(item)>0)
                self.targetActionList.append(item)
        
        # by default the robot is at the 1st waypoint of the file.
        self.waypt_idx = init_pos-1
        self.lenWaypt = len(self.targetActionList)
        self.isCyclic = self.targetActionList[-1][1]=='cycle'
        
        # last reported location, for recovery from faults
        self.last_loc = PoseWithCovarianceStamped()
        
    def gotoX(self, target_label):
        """
        Go to a target in the waypoint file specified by the target_label
        """
        for i in range(self.waypt_idx, self.waypt_idx+self.lenWaypt):
            idx = i%self.lenWaypt
            if self.targetActionList[idx][0]==target_label:
                break
            return -8   # label not found in the file.
        if (idx>=self.waypt_idx):
            # not crossing the end of file
            s = os.system('rosrun pcv_base pubgoalNew.py _waypt_file_path:='+\
                          self.pathName+' _startLn:='+str(self.waypt_idx)+\
                          ' _endLn:='+str(idx+1)+' _dir:=0')
            if (s==0):
                self.waypt_idx = idx
            return s
            
        elif not self.isCyclic:
            return -9   # waypoint is not reachable in a un-cycleable path.
            
        else:
            s = os.system('rosrun pcv_base pubgoalNew.py _waypt_file_path:='+\
                          self.pathName+' _startLn:='+str(self.waypt_idx+1)+\
                          ' _endLn:='+str(self.lenWaypt)+' _dir:=0')
            if s==0:
                self.waypt_idx = self.lenWaypt - 1
                s = os.system('rosrun pcv_base pubgoalNew.py _waypt_file_path:='+\
                              self.pathName+' _startLn:=1 _endLn:='+str(idx+1)+\
                              ' _dir:=0')
                if s==0:
                    self.waypt_idx = idx
            return s
            
    def gotoNext(self):
        """
        Go to the next target which is not a via point.
        """
        s = 0
        eof = self.waypt_idx+1 == self.lenWaypt
        if not eof:   # go towards the end of the file
            for i in range(self.waypt_idx+1, self.lenWaypt):
                if self.targetActionList[i][1]!='via':
                    break
            print('rosrun pcv_base pubgoalNew.py _waypt_file_path:='+\
                          self.pathName+' _startLn:='+str(self.waypt_idx+1)+\
                          ' _endLn:='+str(i+1)+' _dir:=0')
            s = os.system('rosrun pcv_base pubgoalNew.py _waypt_file_path:='+\
                          self.pathName+' _startLn:='+str(self.waypt_idx+1)+\
                          ' _endLn:='+str(i+1)+' _dir:=0')
            if (s==0):
                self.waypt_idx = i
                eof = self.waypt_idx+1 == self.lenWaypt
                if not eof:
                    return 0
        
        if eof and s==0:   # currently at end of waypt file.
            if not self.isCyclic:
                return 0                        # completion of path
            else:
                self.waypt_idx = 0
                for i in range(self.lenWaypt):
                    if self.targetActionList[i][1]!='via':
                        break
                s = os.system('rosrun pcv_base pubgoalNew.py _waypt_file_path:='+\
                          self.pathName+' _startLn:=1 _endLn:='+str(i+1)+\
                          ' _dir:=0')
                if (s==0):
                    self.waypt_idx = i
        return s
        
    def evaluate(self):
        s = 0
        if self.targetActionList[self.waypt_idx][1] == 'via':
            pass
        elif self.targetActionList[self.waypt_idx][1] == 'cycle':
            pass
        elif self.targetActionList[self.waypt_idx][1] == 'end':
            s = -128            # ending mark
        elif self.targetActionList[self.waypt_idx][1] == 'pause':
            last_loc_rec = self.last_loc
            payload.pause()
            print(payload.isPaused())
            if len(self.targetActionList[self.waypt_idx])>=3:
                try:
                    t = float(self.targetActionList[self.waypt_idx][2])
                    if t>0:
                        time.sleep(t)
                        payload.resume()
                except ValueError:
                    pass
            while payload.isPaused():
                pass
            # anti-drifting re-init
            self.pose_pub.publish(last_loc_rec)
        else:
            cmd = self.targetActionList[self.waypt_idx][1]+"(self, payload, "
            for i in range(2,len(self.targetActionList[self.waypt_idx])-1):
                cmd = cmd + self.targetActionList[self.waypt_idx][i] + ","
            cmd = cmd + self.targetActionList[self.waypt_idx][-1] + ")"
            s = eval(cmd)
        return s
        
    def run(self):
        s = 0
        print('Run Thread Spawned!')
        while payload.isReady():
            #print('In While Loop...')
            if payload.isRunning():
                s = self.gotoNext()
                if s==0:
                    s = self.evaluate()
                    if s==-128:         # ending mark
                        payload.setDoneStatus()
                        break
                else:
                    print('HELP NEEDED!')
                    # payload.sendSMS('FIXME: Robot got stuck, task is INCOMPLETE!')
                    # help code...
        
        # THIS SEGMENT SERVES AS A TEMPLATE FOR NOW...
        """
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

        if (s == 0):
            print('Done!')
        else:
            print('HELP NEEDED!')
            # payload.sendSMS('FIXME: Robot got stuck, task is INCOMPLETE!')
            # help code...
        
        try:
            pass
        finally:
            payload.setDoneStatus()
            #nav_thread.terminate()
            #robot_thread.terminate()
            launch.shutdown()
        """
        
    def start(self):
        self.launch.start()
        
        rospy.init_node('admin')
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
        self.pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        while not self.pose_pub.get_num_connections():
            pass
        #init_pose = [ 0.0453101396561, 0.00364243984222, 0.0, 0.0, 0.0, -0.032842760778, 0.999460531019]
        init_pose = self.traj[self.waypt_idx]
        
        # initialize position in the map.
        ipose = PoseWithCovarianceStamped()
        ipose.header.frame_id="map"
        ipose.pose.pose.position.x = init_pose[0]
        ipose.pose.pose.position.y = init_pose[1]
        ipose.pose.pose.orientation.z = init_pose[5]
        ipose.pose.pose.orientation.w = init_pose[6]
        self.pose_pub.publish(ipose)
        
        while not payload.isReady():
            pass
        
        self.pose_pub.publish(ipose)
        time.sleep(1)
        
        self.run_thread = multiprocessing.Process(target=self.run, args=())
        self.run_thread.daemon = True
        self.run_thread.start()
        
        #def payload_mon(self):
        print('In main process')
        #print(payload.isReady())
        pause_pub = rospy.Publisher('/pauseAction', Byte, queue_size=1)
        while payload.isReady():
            #print(payload.isPaused())
            if payload.isPaused():
                pause_pub.publish(Byte(1))
            else:
                pause_pub.publish(Byte(0))
            time.sleep(0.1)
        
    def amcl_cb(self,msg):
        self.last_loc = msg
        
    def isReady(self):
        return payload.isReady()
    
    def stop(self):
        payload.setDoneStatus()
        self.run_thread.terminate()
        self.launch.shutdown()
        
    def cleanup(self):
        payload.payload_thread.terminate()
        
if __name__ == '__main__':
    task = Task()
    task.start()
    task.run()
    task.stop()
    task.cleanup()
