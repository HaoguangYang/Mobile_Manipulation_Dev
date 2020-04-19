#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Time
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu

class imu_fusion:
    def __init__(self, nIMUs, staleDelay, rate):
        self.nIMUs = nIMUs
        #self.topics = ["imu_"+i for i in range(0,nIMUs)]
        self.stale = [True for i in range(0,nIMUs)]
        self.last_rx = [rospy.Time() for i in range(0,nIMUs)]
        self.orientation = np.zeros(nIMUs,3)
        self.orientation_cov = np.zeros(nIMUs,9)
        self.omega = np.zeros(nIMUs,3)
        self.omega_cov = np.zeros(nIMUs,9)
        self.acc = np.zeros(nIMUs,3)
        self.acc_cov = np.zeros(nIMUs,9)
        #self.in_msg = [Imu() for i in range(0,nIMUs)]
        self.out_msg = Imu()
        self.staleDelay = staleDelay
        self.rate = rospy.Rate(rate)
        self.ref_frame = ''
        self.publisher = rospy.Publisher("imu_out", Imu, queue_size=1)

    def imuMsgCb(self, data, n):
        assert data.header.frame_id == self.ref_frame
        self.stale[n] = False;
        self.last_rx[n] = data.header.stamp;
        orientation_msg = data.orientation
        self.orientation[n] = euler_from_quaternion([orientation_msg.x,
                                                     orientation_msg.y,
                                                     orientation_msg.z,
                                                     orientation_msg.w])
        self.orientation_cov[n] = msg.orientation_covariance
        omega_msg = msg.angular_velocity
        self.omega[n] = [omega_msg.x, omega_msg.y, omega_msg.z]
        self.omega_cov[n] = msg.angular_velocity_covariance
        acc_msg = msg.linear_acceleration
        self.acc[n] = [acc.msg.x, acc.msg.y, acc,msg.z]
        self.acc_cov = msg.linear_acceleration_covariance
    
    def fusion(self):
        timeNow = rospy.Time.now()
        counter = 0
        counter_ori_cov = 0     # if no covariances is available, put -1 at first position.
        counter_omeg_cov = 0
        counter_acc_cov = 0
        ori_out_re = np.zeros(3)
        ori_out_im = np.zeros(3)
        ori_cov_out = np.zeros(9)
        omeg_out = np.zeros(3)
        omeg_cov_out = np.zeros(9)
        acc_out = np.zeros(3)
        acc_cov_out = np.zeros(9)
        latest_time = rospy.Time()
        for i in range(0, self.nIMUs):
            if not (self.stale[i]):
                if ((timeNow - self.last_rx[i]).to_sec()*1000 > self.staleDelay):
                    self.stale[i] = True
                else:
                    if (self.last_rx[i] > latest_time):
                        latest_time = self.last_rx[i]
                    counter += 1
                    ori_out_re += np.cos(self.orientation[i])
                    ori_out_im += np.sin(self.orientation[i])
                    omeg_out += self.omega[i]
                    acc_out += self.acc[i]
                    if self.orientation_cov[i,0] >= 0.:
                        counter_ori_cov += 1
                        ori_cov_out += self.orientation_cov[i]
                    if self.omega_cov[i,0] >= 0.:
                        counter_omeg_cov += 1
                        omeg_cov_out += self.omega_cov[i]
                    if self.acc_cov[i,0] >= 0.:
                        counter_acc_cov += 1
                        acc_cov_out += self.acc_cov[i]
        if counter:
            counter_sq = counter*counter
            ori_out = np.atan2(ori_out_im, ori_out_re)
            omeg_out /= counter_sq
            acc_out /= counter_sq
            if counter_ori_cov:
                ori_cov_out /= (counter_ori_cov*counter_ori_cov)
            else:
                ori_cov_out[0] = -1.        # it has not been modified yet (zeros)
            if counter_omeg_cov:
                omeg_cov_out /= (counter_omeg_cov*counter_omeg_cov)
            else:
                omeg_cov_out[0] = -1.
            if counter_acc_cov:
                acc_cov_out /= (counter_acc_cov*counter_acc_cov)
            else:
                acc_cov_out[0] = -1.
            self.out_msg.header.stamp = latest_time
            q_out = quaternion_from_euler(ori_out)
            self.out_msg.orientation = Quaternion(q_out[0], q_out[1], q_out[2], q_out[3])
            self.out_msg.orientation_covariance = ori_cov_out
            self.out_msg.angular_velocity = Vector3(omeg_out[0], omeg_out[1], omeg_out[2])
            self.out_msg.angular_velocity_covariance = omeg_cov_out
            self.out_msg.linear_acceleration = Vector3(acc_out[0], acc_out[1], acc_out[2])
            self.out_msg.linear_acceleration_covariance = acc_cov_out
            self.publisher.publish(self.out_msg)
    
    def run(self):
    	init = rospy.wait_for_message("imu_0", Imu)
    	self.ref_frame = init.header.frame_id
    	self.out_msg.header.frame_id = self.ref_frame
    	self.subs = [rospy.Subscriber("imu_"+i, Imu, self.imuMsgCb, i) for i in range(0,self.nIMUs)]
        while not rospy.is_shutdown():
            self.fusion()
            rate.sleep()


if __name__ == '__main__':
    try:
        nIMUs = rospy.get_param('~number_of_IMUs')
        staleDelay = rospy.get_param('~stale_delay')
        rate = rospy.get_param('~publish_rate')
        imu_fuse = imu_fusion(nIMUs, staleDelay, rate)
        imu_fuse.run()
    except rospy.ROSInterruptException:
        pass 
