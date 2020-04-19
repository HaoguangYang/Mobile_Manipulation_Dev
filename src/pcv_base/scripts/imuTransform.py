#!/usr/bin/env python
import rospy
import numpy as np
import tf
from std_msgs.msg import Time, Header
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu

class imu_transform:
    def __init__(self, accFrame, gyroFrame, outFrame):
        self.omega_out = np.array([0., 0., 0.])
        self.omega_cov_out = np.zeros([3,3])
        self.omega_dot = np.array([0., 0., 0.])
        self.a_out = np.array([0., 0., 0.])
        self.a_cov_out = np.zeros([3,3])
        self.rot_out = np.array([0., 0., 0.])
        
        self.tf_listener = tf.TransformListener()
        self.out_to_in_trans = np.array([0., 0., 0.])
        self.in_orientation = np.array([0.,0.,0.,1.])
        self.out_to_in_rot = np.zeros([3,3])            # turns to eye(3) or proper format after receiving first tf frame.
        self.out_to_in_rot_q = np.array([0.,0.,0.,1.])
        self.in_to_gyro_rot = np.zeros([3,3])           # turns to eye(3) or proper format after receiving first tf frame.
        self.in_to_gyro_rot_q = np.array([0.,0.,0.,1.])
        
        self.omega_last = np.array([0., 0., 0.])
        self.a_in = np.array([0., 0., 0.])
        self.a_cov_in = np.zeros([3,3])
        self.time_omega_old = Time()
        self.time_acc_old = Time()
        
        # in_frame should be the accelerometer frame. Gyro data shall be transformed to the accel frame accordingly.
        self.gyroFrame = gyroFrame
        self.inFrame = accFrame
        self.outFrame = outFrame
        self.pub_transformed = rospy.Publisher("imu_in_"+outFrame, Imu, queue_size=1)

    def calibrate(self):
        acc = np.zeros(3)
        gyr = np.zeros(3)
        for i in range(0,100):
            In = rospy.wait_for_message("accel_"+self.inFrame, Imu)
            acc[0] += In.linear_acceleration.x
            acc[1] += In.linear_acceleration.y
            acc[2] += In.linear_acceleration.z
        for i in range(0,100):
            gyrIn = rospy.wait_for_message("gyro_"+self.gyroFrame, Imu)
            gyr[0] += In.angular_velocity.x
            gyr[1] += In.angular_velocity.y
            gyr[2] += In.angular_velocity.z
        acc = acc*0.01
        alpha = -np.atan(acc[1]/acc[2])
        beta = np.atan(acc[0]/acc[2])
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        cb = np.cos(beta)
        sb = np.sin(beta)
        self.accCalibMatrix = 9.80665/np.linalg.norm(acc,2)* \
                            np.matmul(np.array([[1,0,0],[0,ca,sa],[0,-sa,ca]]), \
                                      np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
        self.gyrZeroCorrection = -gyr*0.01
        # TODO: add in gyro scaling through launch file

    def getTransform(self):
        try:
            (self.out_to_in_trans, self.out_to_in_rot_q) = self.tf_listener.lookupTransform(self.outFrame, self.inFrame, rospy.Time(0))
            (_, self.in_to_gyro_rot_q) = self.tf_listener.lookupTransform(self.inFrame, self.gyroFrame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.out_to_in_rot = tf.transformations.quaternion_matrix( \
                                 np.matmul(self.out_to_in_rot_q, self.accCalibMatrix))
        self.in_to_gyro_rot = tf.transformations.quaternion_matrix(self.in_to_gyro_rot_q)
        
    def transformOmega(self, omega, omega_cov, rot):
        omega_o = np.matmul(rot, omega)
        omega_cov_o = np.matmul(np.matmul(rot, omega_cov), rot.transpose())
        return (omega_o, omega_cov_o)
        
    def getOmegaDot(self, omega, t):
        self.omega_dt = t - self.time_omega_old
        delta = (self.omega_last + omega) * 0.5 * self.omega_dt
        # small rotation matrix from in_frame(t-1) to in_frame(t). It is inverse symmetric.
        in_dot = np.array([[1., -delta[2], delta[1]],
                           [delta[2], 1., -delta[0]],
                           [-delta[1], delta[0], 1]])
        # apply transformation to omega(t-1)
        omega_old_in_new_frame = np.matmul(-in_dot, self.omega_last)
        self.time_omega_old = t
        self.omega_last = omega
        # iteratively compute orientation of the in_frame.
        in_dot_q = tf.transformations.quaternion_from_matrix(in_dot)
        self.in_orientation = (lambda q: q/np.linalg.norm(q))(
                                tf.transformations.quaternion_multiply(in_dot_q, self.in_orientation))
        return (omega - omega_old_in_new_frame)/self.omega_dt
        
    def getAccel(self, accel, omega, omega_dot, accel_cov, omega_cov):
        trans = self.out_to_in_trans
        rot = self.out_to_in_rot
        # a transformation rules: a0 + omega_dot x r + omega x (omega x r), all expressed within the a0 frame.
        a_out_inFrame = accel + np.cross(omega_dot, trans) + np.cross(omega, np.cross(omega, trans))
        self.a_out = np.matmul(rot, a_out_inFrame)
        # compute covariance: matrix format of cross product
        R = np.array([[0, trans[2], -trans[1]], [-trans[2], 0, trans[0]], [trans[1], -trans[0], 0]])
        # 4 parts are considered: original accel cov, cov of omega_dot operated by R, cov(omega.(omega.r)), and r^2*cov(omega.omega)I(3).
        a_cov = accel_cov + np.matmul(np.matmul(R,omega_cov),R.transpose())*2./(self.omega_dt**2) \
                + (np.matmul(np.matmul(trans,omega_cov),trans.transpose()))**2*omega_cov \
                + trans**2*np.eye(3)*np.sum(omega_cov**2)
        # transform to out_frame
        self.a_cov_out = np.matmul(np.matmul(rot, accel_cov), rot.transpose())
    
    def omegaMsgCb(self, msg):
        time = msg.header.stamp.to_sec()
        msg_omega = msg.angular_velocity
        omega = np.array([msg_omega.x,
                          msg_omega.y,
                          msg_omega.z]) + self.gyrZeroCorrection
        omega_cov = np.reshape(msg.angular_velocity_covariance, [3,3])
        (omega, omega_cov) = self.transformOmega(omega, omega_cov, self.in_to_gyro_rot)
        (self.omega_out, self.omega_cov_out) = self.transformOmega(omega, omega_cov, self.out_to_in_rot)
        omega_dot = self.getOmegaDot(omega, time)
        self.getAccel(self.a_in, omega, omega_dot, self.a_cov_in, omega_cov)
        # transform to out_frame
        out_orientation = tf.transformations.quaternion_multiply(
                                tf.transformations.quaternion_conjugate(self.out_to_in_rot_q),
                                self.in_orientation
                                )
        msg_out = Imu(
                        header = Header(stamp=msg.header.stamp, frame_id=self.outFrame),
                        orientation = Quaternion(out_orientation[0], out_orientation[1], out_orientation[2], out_orientation[3]),
                        orientation_covariance = msg.orientation_covariance,
                        angular_velocity = Vector3(self.omega_out[0], self.omega_out[1], self.omega_out[2]),
                        angular_velocity_covariance = list(self.omega_cov_out.reshape(9)),
                        linear_acceleration = Vector3(self.a_out[0], self.a_out[1], self.a_out[2]),
                        linear_acceleration_covariance = list(self.a_cov_out.reshape(9))
                     )
        self.pub_transformed.publish(msg_out)
        
    def accelMsgCb(self, msg):
        time = msg.header.stamp.to_sec()
        msg_acc = msg.linear_acceleration
        self.a_in = np.array([msg_acc.x,
                              msg_acc.y,
                              msg_acc.z]))
        self.a_cov_in = np.reshape(msg.linear_acceleration_covariance, [3,3])

    def linkImu(self):
        # link subscribed topics to call-back functions.
        self.gyro_sub = rospy.Subscriber("gyro_"+self.gyroFrame, Imu, self.omegaMsgCb)
        self.imu_sub = rospy.Subscriber("accel_"+self.inFrame, Imu, self.accelMsgCb)

    def run(self, rate):
        self.calibrate()
        while not self.out_to_in_rot.any():
            self.getTransform()
        self.linkImu()
        if rate:
            rate = rospy.Rate(rate)
            while not rospy.is_shutdown():
                self.getTransform()
                rate.sleep()
        else:
            rospy.spin()


if __name__ == '__main__':
    try:
        accF = rospy.get_param('~accelerometer_frame')
        gyroF = rospy.get_param('~gyroscope_frame')
        outF = rospy.get_param('~output_frame')
        # TODO: add in gyro scaling through launch file
        # TODO: add in accelerometer scaling through launch file
        # TODO: add in option of using stored calibration
        imu_tf = imu_transform(accF, gyroF, outF)
        isStatic = rospy.get_param('~is_static_tf')
        if isStatic:
            imu_tf.run(0)
        else:
            rate = rospy.get_param('~frame_update_frequency')
            imu_tf.run(rate)
    except rospy.ROSInterruptException:
        pass 
