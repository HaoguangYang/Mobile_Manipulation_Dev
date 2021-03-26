#!/usr/bin/env python
import rospy
import roslaunch
from sensor_msgs.msg import Joy
from RobotBase import RobotBase

def joystickTeleopBase()
    class joystickTeleopBaseClass(RobotBase):
        def __init__(self):
            super().__init__()
            self.__joystick_node = roslaunch.core.Node('joy', 'joy_node', name='joy')
            self.__launch = roslaunch.scriptapi.ROSLaunch()
            self.__launch.start()
            self.v_lim = [0.5, 0.5, 0.5]
            #rospy.on_shutdown(self.on_kill)

        def execute(self):
            super().execute()
            self.__joystick_inst = self.__launch.launch(self.__joystick_node)
            self.__joy_if_inst = rospy.Subscriber('joy',Joy, self.__joy_cb)
            self.robot_enable(1)   # enable the robot and enters velocity mode
            
        def shutdown(self):
            super().shutdown()
            self.__joystick_inst.stop()
            self.__joy_if_inst.shutdown()
            self.robot_enable(0)
            
        def evaluate(self):
            super().evaluate()
            
        def estimate(self):
            super().estimate()
            
        def __joy_cb(self, msg):
            vel_x = msg.axes[1]*self.v_lim[0]
            vel_y = msg.axes[0]*self.v_lim[1]
            omega = msg.axes[2]*self.v_lim[2]
            self.robot_cmd_vel(vel_x, vel_y, omega)
    
    return joystickTeleopBaseClass
    
if __name__ == "__main__":
    topBase = joystickTeleopBase()
    topBase.__init__()
    topBase.execute()
    rospy.spin()
    topBase.shutdown()
