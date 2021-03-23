#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf import transformations as ts
import time

class RobotBase(object):
    name = "robotBase"
    
    def __init__(self):
        rospy.init_node(self.name, anonymous=True)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.__launch_baseNode = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/pcv_node.launch'])
        
        self.__ena_pub = rospy.Publisher('/mobile_base_controller/control_mode', Byte, queue_size = 1)
        self.__vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
        self.v_lim = [0.5, 0.5, 0.5]
        # TODO: these localization variables need to be initialized before first used.
        self.basePos = np.array([0.,0.,0.])
        self.baseTime = Time()
        self.__ekfPos = np.array([0.,0.,0.])
        self.__ekfTime = Time()
        self.paused = False
        while not self.__ena_pub.get_num_connections():
            pass
        
    def execute(self):
        self.__amcl_inst = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__amcl_cb)
        self.__ekf_inst = rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.__ekf_cb)
        self.__watchdog_inst = rospy.Timer(rospy.Duration(1), self.__watchdog_cb)
    
    def shutdown(self):
        self.watchdog_inst.shutdown()
        self.amcl_inst.shutdown()
        self.ekf_inst.shutdown()
        self.launch_baseNode.shutdown()
    
    def evaluate(self):
        pass
        
    def estimate(self):
        pass
    
    def __watchdog_cb(self, event):
        try:
            check_output(["pidof","pcv_base_node"])
        except subprocess.CalledProcessError:
            # thread is not running
            self.__launch_baseNode.shutdown()
            self.__launch_baseNode = roslaunch.parent.ROSLaunchParent(self.uuid,['./src/pcv_base/launch/pcv_node.launch'])
            self.__launch_baseNode.start()
        
    def __amcl_cb(self, msg):
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat) 
        # current position vector   
        self.basePos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, eul[2]])
        self.baseTime = msg.header.stamp
        
    def __ekf_cb(self, msg):
        quat_new = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat_new)
        ekfPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, eul[2]])
        ekfTime = msg.header.stamp
        # if got newer odom data
        if (ekfTime > self.baseTime):
            ekfDt = ekfTime - self.__ekfTime
            vel = (ekfPos - self.__ekfPos)/ekfDt
            dAng = self.__ekfPos[2] - self.basePos[2]
            # velocity of robot in base frame
            vel_b = np.dot(np.array([[np.cos(dAng), -np.sin(dAng)],
                                     [np.sin(dAng), np.cos(dAng)]]),vel[0:2])
            self.basePos = self.basePos + np.array([vel_b[0], vel_b[1], vel[2]])*(ekfTime-self.baseTime)
            self.baseTime = ekfTime
        self.__ekfTime = ekfTime
        self.__ekfPos = ekfPos
        
    def robot_enable(self, state):
        # 0 - Disable
        # 1 - Velocity
        # 2 - Torque
        if (state==0):
            self.__ena_pub.publish(Byte(0))
            self.paused = True
        elif (state==1):
            self.__ena_pub.publish(Byte(1))
            self.paused = False
        elif (state==2):
            self.__ena_pub.publish(Byte(2))
            self.paused = False
        else:
            warnings.warn('Unrecognized robot control mode, setting the mode is neglected!')
        
    def robot_cmd_vel(self, vx, vy, omega):
        cmd = Twist()
        cmd.linear.x = np.clip(vx, -self.v_lim[0], self.v_lim[0])
        cmd.linear.y = np.clip(vy, -self.v_lim[1], self.v_lim[1])
        cmd.angular.z = np.clip(omega, -self.v_lim[2], self.v_lim[2])
        self.__vel_pub.publish(cmd)

class joystickTeleop(RobotBase):
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

class planningMotion(RobotBase):
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
        self.navStatus_inst.shutdown()
        self.plan_cycle_inst.shutdown()
        
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
                # TODO: Signal failure
                pass

class Payload(base):
    def __init__(self):
        super().__init__()
        self.__payload_if = serial.Serial('/dev/ttyACM0', timeout=2)  # open serial port.
        time.sleep(1)
        self.__payload_if.setDTR(0)
        time.sleep(1)
        """
        sms_cred = ET.parse('/home/cartman/Dev/smsCredentials.xml')
        account_sid = sms_cred.findall('account_sid')[0].get('value')
        auth_token = sms_cred.findall('auth_token')[0].get('value')
        self.sms_client = Client(account_sid, auth_token)
        self.sms_from = sms_cred.findall('from')[0].get('value')
        self.sms_to = sms_cred.findall('to')[0].get('value')
        """
        self.payload_in_queue = ""
        self.payload_out_queue = ""
        
    def execute(self):
        super().execute()
        self.__payload_if_cycle_inst = rospy.Timer(rospy.Duration(1./50.), self.__cycle)
    
    def __cycle(self):
        in_bytes = self.__payload_if.inWaiting()
        out_bytes = len(self.payload_out_queue)
        if in_bytes:
            self.payload_in_queue += self.__payload_if.read(in_bytes)
            # recycle space
            in_size = len(self.payload_in_queue)
            if in_size > 256:
                self.payload_in_queue = self.payload_in_queue[in_size-256:]
        if out_bytes:
            self.__payload_if.write(self.payload_out_queue)
            self.payload_out_queue = ""
            
    def ser_recv(head, tail, size):
        offset_head = len(head)
        offset_tail = len(tail)
        # find beginning of the message
        start = self.payload_in_queue.find(head)
        # extract payload
        data = self.payload_in_queue[start+offset_head:start+offset_head+size]
        # find end of the message
        end = self.payload_in_queue.find(tail, start+offset_head+size)
        # remove message from queue
        self.payload_in_queue = self.payload_in_queue[0:start]+self.payload_in_queue[end+offset_tail:]
        if end!=start+offset_head+size:
            # Size of payload does not match the size provided. Invalidate message
            return ""
        else:
            # return the payload stripped from the in queue as a string.
            return data
    
    def ser_send(msg):
        self.payload_out_queue += msg
    
class StartButton(Payload):
    pass    

class UVCLamp(Payload):
    pass

class Sprayer(Payload):
    pass

class Bernoulli(Payload):
    pass

class WirelessCharging(Payload):
    pass

class PumaArm(Payload):
    pass

def assembleTask(base, payload, task):
    base = eval(base)
    payload = eval(payload)
    task = eval(task)
    return task


if __name__ == "__main__":
    Child().run()
