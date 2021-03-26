#!/usr/bin/env python
import rospy
import roslaunch
from RobotBase import RobotBase

def keyboardTeleopBase():
    class keyboardTeleopBaseClass(RobotBase):
        def __init__(self):
            super(keyboardTeleopBaseClass,self).__init__()
            self.__kb_node = roslaunch.core.Node('pcv_base', 'teleop_twist_keyboard', name='kb')
            self.__launch = roslaunch.scriptapi.ROSLaunch()
            self.__launch.start()

        def execute(self):
            super(keyboardTeleopBaseClass,self).execute()
            self.__kb_inst = self.__launch.launch(self.__kb_node)
            self.robot_enable(1)   # enable the robot and enters velocity mode
            
        def shutdown(self):
            super(keyboardTeleopBaseClass,self).shutdown()
            self.__kb_inst.stop()
            self.robot_enable(0)
            
        def evaluate(self):
            super(keyboardTeleopBaseClass,self).evaluate()
            
        def estimate(self):
            super(keyboardTeleopBaseClass,self).estimate()
    
    return keyboardTeleopBaseClass
    
if __name__ == "__main__":
    topBase = keyboardTeleopBase()
    topBase.__init__()
    topBase.execute()
    rospy.spin()
    topBase.shutdown()
