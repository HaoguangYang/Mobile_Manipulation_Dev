#!/usr/bin/env python
import rospy
import roslaunch
import numpy as np
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from RobotBase import RobotBase

def motionPlanningBase():
    class motionPlanningBaseClass(RobotBase):
        def __init__(self):
            super().__init__()
            self.__movebase_launch = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/includes/navigation.launch'])
            self.__timeout = rospy.get_param('~timeout', 300)
            self.__gpub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=1)
            self.navStatus = 0
            self.__trajTol = [0.3, 0.2]  # Distance, angular error tolerance
            self.__traj = np.array([])
            self.__navIndex = 0
            self.__timer = 0.
            self.__period = 0.
                    
        def execute(self, freq=5.0):
            super().execute()
            self.__movebase_launch.start()
            self.__navStatus_inst = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.__navStatus_cb)
            self.__period = 1./freq
            self.__plan_cycle_inst = rospy.Timer(rospy.Duration(1./freq), self.__cycle)
                    
        def shutdown(self):
            super().shutdown()
            self.__movebase_launch.shutdown()
            self.__navStatus_inst.shutdown()
            self.__plan_cycle_inst.shutdown()
            
        def evaluate(self):
            super().evaluate()
            
        def estimate(self):
            super().estimate()
        
        def __navStatus_cb(self, msg):
            self.navStatus = msg.status.status
            print('Status = ', self.navStatus)
            
        def setVias(self, viaList, mode=''):
            if (mode=='a'):
                appendList = np.array(viaList)
                if appendList.ndim<2:                # in case of only one line in the traj...
                    appendList = np.expand_dims(appendList,0)
                self.__traj = np.append(self.__traj, appendList, 0)
            else:
                self.__traj = np.array(viaList)
                if self.__traj.ndim<2:                # in case of only one line in the traj...
                    self.__traj = np.expand_dims(self.__traj,0)
                self.__navIndex = 0

        def __cycle(self):
            if self.navStatus == 3 or \
               ((self.basePos[0]-self.__traj[self.__navIndex,0])**2 + \
                (self.basePos[1]-self.__traj[self.__navIndex,1])**2 < self.__trajTol[0]**2 and \
                np.abs(self.basePos[2]-self.__traj[self.__navIndex,2]) < self.__trajTol[1]):
                self.__navIndex += 1
                if self.__navIndex >= self.__traj.shape[0]:
                    self.shutdown()
                    # TODO: signal done
                    pass
                else:
                    msg = PoseStamped()            
                    # Create header
                    h = Header()
                    h.stamp = rospy.Time.now()
                    h.frame_id = "map"
                    msg.header = h
                    # Add position
                    msg.pose.position.x =  self.__traj[i,0]
                    msg.pose.position.y =  self.__traj[i,1]
                    # Add orientation
                    msg.pose.orientation.x =  self.__traj[i,3]
                    msg.pose.orientation.y =  self.__traj[i,4]
                    msg.pose.orientation.z =  self.__traj[i,5]
                    msg.pose.orientation.w =  self.__traj[i,6]
                    
                    self.__gpub.publish(msg)
                    self.navStatus = 0
                    self.__timer = 0.
            else:
                self.__timer += self.__period * (!self.paused)
                if self.__timer > self.__timeout:
                    self.shutdown()
                    # TODO: Signal failure
                    pass
    return motionPlanningBaseClass
    
if __name__ == "__main__":
    autoBase = motionPlanningBase()
    autoBase.__init__()
    autoBase.execute()
    rospy.spin()
    autoBase.shutdown()
