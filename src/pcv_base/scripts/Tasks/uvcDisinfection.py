#!/usr/bin/env python

import os, time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Byte
from Payloads.UVC import payload
import roslaunch
import numpy as np
import multiprocessing
import tokenize
from definedFunctions import *

class Task():
    def __init__(self, uuid, init_pos = 1):
        payload.initialize()
        self.uuid = uuid
        self.delayStart = 30
        
        self.laserLaunch = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/includes/laser.launch'])
        # remove laser and map server in the launch file below, since they are separately launched.
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/run_nav_only_narrow.launch'])
        
        # route file, combined. Route file is generated based on the "Left"-sided map.
        self.pathName = '/home/cartman/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/traj/disinfectionRoute.txt'
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
        1.0, -1.0, 0.,0.,0.707, 0.707   # A, gotoWaitAndProceed, trajFile.txt
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
        
    def runDisinfection(self):
        s = 0
        print('Run Thread Spawned!')
        payload.turnOnUVC()
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
                    payload.setDoneStatus()
                    print('HELP NEEDED!')
                    # payload.sendSMS('FIXME: Robot got stuck, task is INCOMPLETE!')
                    # help code...
                    break
        payload.turnOffUVC()
        
    def amcl_cb(self,msg):
        self.last_loc = msg
        
    def isReady(self):
        return payload.isReady()
    
    #def chkFault(self):
    #    return not payload.isReady()

    def start(self):
        time.sleep(self.delayStart)
        self.laserLaunch.start()
        init_scan = rospy.wait_for_message("/scan", LaserScan)
        amin = init_scan.angle_min
        dth = init_scan.angle_increment
        wheremax = np.argmax(init_scan.ranges) * dth + amin
        print(init_scan.ranges[wheremax])
        package = 'map_server'
        executable = 'map_server'
        self.isLeft = False
        if wheremax > 0:        # opening is to the left, use the L side map file
            node = roslaunch.core.Node(package, executable, name=package, args='$(find pcv_base)/resources/map/pvil_small_L.yaml')
            self.isLeft = True
        else:
            node = roslaunch.core.Node(package, executable, name=package, args='$(find pcv_base)/resources/map/pvil_small_R.yaml')
        self.mapLaunch = roslaunch.scriptapi.ROSLaunch()
        self.mapLaunch.start()
        self.mapServer = self.mapLaunch.launch(node)
        
        self.launch.start()

        rospy.init_node('admin')
        
        pose_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=1)
        while not pose_pub.get_num_connections():
            pass
        
        ipose = PoseWithCovarianceStamped()
        ipose.header.frame_id="map"
        init_pose = self.traj[self.waypt_idx]
        ipose.pose.pose.position.y = init_pose[1]
        if self.isLeft:
            ipose.pose.pose.position.x = init_pose[0]
            ipose.pose.pose.orientation.z = init_pose[5]
            ipose.pose.pose.orientation.w = init_pose[6]
        else:   # perform mirroring operation
            ipose.pose.pose.position.x = -init_pose[0]
            ipose.pose.pose.orientation.z = init_pose[6]
            ipose.pose.pose.orientation.w = init_pose[5]
        pose_pub.publish(ipose)
        time.sleep(1)
        
        self.task_thread = multiprocessing.Process(target=self.runDisinfection, args=())
        self.task_thread.daemon = True
        self.task_thread.start()
        
        print('In main process')
        pause_pub = rospy.Publisher('/pauseAction', Byte, queue_size=1)
        while payload.isReady():
            #print(payload.isPaused())
            if payload.isPaused():
                pause_pub.publish(Byte(1))
            else:
                pause_pub.publish(Byte(0))
            time.sleep(0.1)

    def stop(self):
        payload.turnOffUVC()
        payload.setDoneStatus()
        self.task_thread.terminate()
        self.launch.shutdown()
        self.laserLaunch.shutdown()
        self.mapLaunch.shutdown()
        
    def cleanup(self):
        payload.payload_thread.terminate()
        
if __name__ == '__main__':
    task = Task()
    task.runDisinfection()
