/*------------------------------includes--------------------------------------*/
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include <signal.h>
#include <string>
#include <unistd.h>
#include <mqueue.h>
#include <map>
#include "../include/motor.h"
#include "../include/CO_message.h"
#include "../include/CAN_utils.h"
#include "../include/RT_utils.h"
#include "../include/vehicle.h"
#include "../include/event.h"
#include "../include/buttons.h"
#include "../include/OTG.h"
#include "../include/pose_t265.h"
#include "../lib/Eigen/Core"
#include "../include/definitions.h"
#include <ctime>
#include <chrono>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>


// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Byte.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"


/* these parameters were for testing sine wave inputs, uncomment next line
   to run the platform in a circular motion and ignore controller inputs */

#define X_freq                  (0.1)       /* [hz] */
#define Y_freq                  (0.1)       /* [hz] */
#define THETA_freq              (0.05)      /* [hz] */

// #define JOYSTICK
// #define CAMERA
// #define MANUAL_ZERO

// Trajectory modes
// #define SPIN
// #define LINE
// #define SQUARE

using std::cout;
using std::endl;

/*------------------------------defines---------------------------------------*/

/*------------------------------structs---------------------------------------*/

/*------------------------static function declarations------------------------*/
static void perform_startup_tasks (void);
static void parse_command_args (int argc, char *argv[]);
static void *control_thread (void *aux);
static void sig_handler (int);
static void sleep_until (struct timespec *ts, long delay);
static int kbhit(void);
/*------------------------static variable declarations------------------------*/
static Vehicle *vehicle;
static CameraT265 *cameraT265;

/* variables for setting operation space velocities (x_dot, y_dot, theta_dot)*/
static double cur_x_dot = 0.;
static double cur_y_dot = 0.;
static double cur_theta_dot = 0.;

/* variables for torque control mode */
static double gx_des_g[3] = {0.};  	// Position desired, global var
static double gxd_des_g[3] = {0.};  // Velocity desired, global var
static double gxdd_des_g[3] = {0.}; // Acceleration desired, global var

/* for recording data */
static bool dumpData = true;
static std::ofstream file;

enum ctrl_mode control_mode = VELOCITY;
// enum ctrl_mode control_mode = TORQUE;

static mqd_t mq_joystick;
static mqd_t mq_tracking;

static const char *mq_name = "/joystick message queue";

/*---------------------------public functions---------------------------------*/
/*
 * The main function for the vehicle program. You can pass a --dump flag to
 * output data to a csv for debugging. This file i/o is performed in the
 * control_thread function if it needs to be changed.
 */


void cmdVelRecvCallback(const geometry_msgs::Twist::ConstPtr& cmdVel)
{
    cur_x_dot = cmdVel->linear.x;
    cur_y_dot = cmdVel->linear.y;
    cur_theta_dot = cmdVel->angular.z;
    //cout<<vel_commands<<endl;
    //if (control_mode == VELOCITY)
        //vehicle->setGlobalVelocity(vel_commands);
}

void cmdTorqXRecvCallback(const geometry_msgs::Pose::ConstPtr& x_des)
{
    gx_des_g[0] = x_des->position.x;
    gx_des_g[1] = x_des->position.y;
    gx_des_g[2] = tf::getYaw(x_des->orientation);
}

void cmdTorqXdRecvCallback(const geometry_msgs::Twist::ConstPtr& xd_des)
{
    gxd_des_g[0] = xd_des->linear.x;
    gxd_des_g[1] = xd_des->linear.y;
    gxd_des_g[2] = xd_des->angular.z;
}

void cmdTorqXddRecvCallback(const geometry_msgs::Accel::ConstPtr& xdd_des)
{
    gxdd_des_g[0] = xdd_des->linear.x;
    gxdd_des_g[1] = xdd_des->linear.y;
    gxdd_des_g[2] = xdd_des->angular.z;
}

void ctrlModeRecvCallback(const std_msgs::Byte::ConstPtr& ctrlMode)
{
    switch (ctrlMode->data){
        case 0:
            if (vehicle->isEnabled())
                vehicle->disable();
            break;
        case 1:
            if (vehicle->getCtrlMode()!=VELOCITY){      // CtrlMode == Torque
                if (vehicle->isEnabled())               // Vehicle Enabled
                    vehicle->disable();                 // First disable vehicle
                vehicle->setCtrlMode(VELOCITY);         // Set CtrlMode to Velocity
                vehicle->enable();                      // Reenable vehicle
            }
            else if (!vehicle->isEnabled())             // CtrlMode == Velocity and vehicle disabled
                vehicle->enable();                      // Enables vehicle
            break;
        case 2:
            if (vehicle->getCtrlMode()!=TORQUE){        // CtrlMode == VELOCITY
                if (vehicle->isEnabled())               // Vehicle Enabled
                    vehicle->disable();                 // First disable vehicle
                vehicle->setCtrlMode(TORQUE);           // Set CtrlMode to TORQUE
                vehicle->enable();                      // Reenable vehicle
            }
            else if (!vehicle->isEnabled())             // CtrlMode == Torque and vehicle disabled
                vehicle->enable();                      // Enables vehicle
            break;
        default:
            ROS_ERROR("Unknown Control Mode!");
            break;
    }
}

int
main (int argc, char *argv[])
{
	// if(argc < 3) {
	//     puts("Usage: sudo ./vehicle <Kp_theta> <Kv_theta>");
	//     exit(0);
	// }f

	cout << "vehicle startup" << endl;
	/* various startup tasks */
	perform_startup_tasks ();

	/* now parse command line args */
	parse_command_args (argc, argv);

	/* Initialize csv file to write to */
	if (dumpData)
	{
		// if (control_mode == TORQUE)
		// {
			file.open("../traces/tuning.csv", std::ios::out);
			file << "t, dt,";
			file << "x, y, theta,";
			file << "x_des, y_des, theta_des,";
			file << "xd, yd, thetad,";
			file << "tq1_des, tq2_des, tq3_des, tq4_des, tq5_des, tq6_des, tq7_des, tq8_des,";
			file << "fx_des, fy_des, th_des,";
			file << "l1, l2, l3, l4, l5, l6, l7, l8, l9,";
			file << "qsteer1, qsteer2, qsteer3, qsteer4,";
			file << "step_x_des, step_y_des, step_theta_des,";
			file << "step_xd_des, step_yd_des, step_thetad_des,";
			file << "x_cam, y_cam, z_cam,";
			file << "th_cam,";
			file << "heading,";
			file << "x_local, y_local, th_local,";
			file << "xd_local, yd_local, thd_local,";
			file << "qd1, qd2, qd3, qd4,";
			file << "Cpinv1, Cpinv2, Cpinv3, Cpinv4, Cpinv5, Cpinv6, Cpinv7, Cpinv8, Cpinv9, Cpinv10,";
			file << "Cpinv11, Cpinv12, Cpinv13, Cpinv14, Cpinv15, Cpinv16, Cpinv17, Cpinv18, Cpinv19,";
			file << "Cpinv20, Cpinv21, Cpinv22, Cpinv23, Cpinv24";
			file << endl;
		// }
	}

	/* Construct the vehicle object */
	vehicle = new Vehicle ();
	pthread_t control;

	/* Initialize vehicle and if successful, launch control thread*/
	if (vehicle->init()){
		cout << "Vehicle Initialization failed!" << endl;
	}
	else
	{
		cout << "Vehicle Initialization succeeded!" << endl;
		vehicle->setCtrlMode(control_mode);
		vehicle->enable();

		// Goal position controls
		// double Kp_x  = 60.0;
		// double Kp_y  = 60.0;
		// double Kp_th = 40.0;

		// double Kv_x  = 12.0;
		// double Kv_y  = 12.0;
		// double Kv_th = 12.0;

		// Trajectory tracking controls
		double Kp_x  = 50.0;
		double Kp_y  = 50.0;
		double Kp_th = 50.0;

		double Kv_x  = 14.0;
		double Kv_y  = 14.0;
		double Kv_th = 20.0;

		Eigen::Array3d Kp;
		Eigen::Array3d Kv;

		Kp << Kp_x, Kp_y, Kp_th;
		Kv << Kv_x, Kv_y, Kv_th;

		vehicle->setKp(Kp);
		vehicle->setKv(Kv);

		cout << "Control gains: " << endl;
		cout << "Kp: " << Kp << endl;
		cout << "Kv: " << Kv << endl;

		#ifdef MANUAL_ZERO
			cout << "Press ENTER once the vehicle is at the desired origin." << endl;
			int c;
			c = getchar();

			#ifdef CAMERA
				/* Construct the camera object*/
				cameraT265 = new CameraT265();

				/* Initialize T265 camera */
				if (cameraT265->init()) {
					cout << "Camera initialization succeeded!" << endl;
				} else {
					cout << "Camera initialization failed!" << endl;
				}
			#endif
			// #ifdef CAMERA
			// 	cameraT265->setOrigin();
			// 	cout << "X offset is: " << cameraT265->getXOffset() << ", Y offset is: " << cameraT265->getYOffset() << endl;
			// #endif
		#endif

		/*Launch control thread */
		launch_rt_thread (control_thread, &control, NULL, MAX_PRIO);
	}

    /* Initialize ROS node:
            Set node name = PCV_Base
            Publishes sensor status to topic = TBD
            Publish rate = 50Hz.
     */
    ros::init(argc, argv, "PCV_Base");
    ros::NodeHandle rosNH;
    ros::Publisher odomPub = rosNH.advertise<nav_msgs::Odometry>("odom", 10);
    //ros::Publisher statusPub = rosNH.advertise<std_msgs::Float64MultiArray>("dump",10);
    ros::Rate pub_rate(50);
    ros::Subscriber cmdSubV = rosNH.subscribe("/mobile_base_controller/cmd_vel", 10, cmdVelRecvCallback);
    ros::Subscriber cmdSubTx = rosNH.subscribe("/mobile_base_controller/cmd_torq_x", 10, cmdTorqXRecvCallback);
    ros::Subscriber cmdSubTxd = rosNH.subscribe("/mobile_base_controller/cmd_torq_xd", 10, cmdTorqXdRecvCallback);
    ros::Subscriber cmdSubTxdd = rosNH.subscribe("/mobile_base_controller/cmd_torq_xdd", 10, cmdTorqXddRecvCallback);
    ros::Subscriber ctrlModeSub = rosNH.subscribe("/mobile_base_controller/control_mode", 10, ctrlModeRecvCallback);

    tf::TransformBroadcaster odomBroadcaster;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    ros::Time current_time;
    odom_trans.transform.translation.z = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    Eigen::Vector3d gx       = Eigen::Vector3d::Zero();
    Eigen::Vector3d gxd      = Eigen::Vector3d::Zero();
    const float sin_PI_4      = -sqrt(2.)*0.5;
    const float cos_PI_4      = sqrt(2.)*0.5;

  //test

  ros::Duration(1.0).sleep();
  
	/* main loop - receive events from controller */
	while (ros::ok())
	{

    #ifdef BUMPER_SENSORS
		if (vehicle->isInitialized()){
			//"SAFETY" -- stop vehicle and program if bumper is hit
			if (vehicle->isBumperHit()) {
				cout << "Bumper hit: " << vehicle->getBumperState() << endl; // Todo: remove print statement - not safe
				sig_handler(0);
			}
		}
    #endif

        // ROS Code Goes Here.
        current_time = ros::Time::now();
        gx = vehicle->getGlobalPosition();
        gxd = vehicle->getGlobalVelocity();
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(gx(2));
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = gx(0);//*cos_PI_4 + gx(1)*sin_PI_4;
        odom_trans.transform.translation.y = gx(1);//*sin_PI_4 + gx(1)*cos_PI_4;
        odom_trans.transform.rotation = odom_quat;
        odomBroadcaster.sendTransform(odom_trans);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = gx(0);//*cos_PI_4 + gx(1)*sin_PI_4;
        odom.pose.pose.position.y = gx(1);//*sin_PI_4 + gx(1)*cos_PI_4;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = gxd(0);
        odom.twist.twist.linear.y = gxd(1);
        odom.twist.twist.angular.z = gxd(2);

        odomPub.publish(odom);

        ros::spinOnce();
        pub_rate.sleep();

    /*
	#ifdef JOYSTICK
		struct event e;
		mq_receive (mq_joystick, (char *)&e, sizeof (e), NULL);
		switch (e.type)
		{
			case NEW_Xd_COMMAND:
				cur_x_dot = MAX_X_VEL * ((int32_t)e.param) / 32767;
				break;

			case NEW_Yd_COMMAND:
				cur_y_dot = MAX_X_VEL * ((int32_t)e.param) / 32767;
				break;

			case NEW_THETAd_COMMAND:
				cur_theta_dot = MAX_THETA_VEL * ((int32_t)e.param) / 32767;
				break;

			if (vehicle->isInitialized()) {
				// handle button events
				case BUTTON_PRESSED:
				{
					switch (e.param)
					{
						// "SAFETY" -- convenience stop!
						case A_BUTTON:
							cout << "A button pressed" << endl;
							// "A" button toggles enable/disable of vehicle
							vehicle->isEnabled() ? vehicle->disable() : vehicle->enable();
						break;
						case X_BUTTON:
							cout << "X button pressed" << endl;
							// "X" changes the control mode to torque control
							control_mode = TORQUE;
							vehicle->disable();
							vehicle->setCtrlMode(control_mode);
							vehicle->enable();
						break;
						case Y_BUTTON:
							cout << "Y button pressed" << endl;
							// "B" changes the control mode to velocity control
							control_mode = VELOCITY;
							vehicle->disable();
							vehicle->setCtrlMode(control_mode);
							vehicle->enable();
						break;

						default:
						break;
					}
				}
			}
			default:
				break;
		}
	#else
		// "SAFETY" -- convenience stop!
		if (kbhit()) {
			cout << "Key is hit" << endl; // Todo: remove print statement - not safe
			sig_handler(0);
		}
	#endif
    */
	}
	delete vehicle;
  raise (SIGINT);
  int status = pthread_kill( control , SIGTERM);
  if ( status <  0)
    perror("pthread_kill control thread failed");
  cout << "Exiting main thread" << endl;
	return 0;
}


/*------------------------------static functions------------------------------*/
/*
 * This function defines the control thread, it utilizes the SYNC message for
 * sending control commands and receiving data from the drivers. In the first
 * half of the control loop, we send a SYNC message and wait for the data to
 * come back from the driver. Then we use that data to calculate the next
 * control effort and update that with a call to the vehicle::set_velocity
 * method. Nothing happens in the second half of the control loop.
 */
static void *
control_thread (void *aux)
{
	struct CO_message msg;
	msg.type = SYNC;

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	/* variable to keep track of number of control loops */
	unsigned long ticks = 0;

	/* Initialize variables */
	Eigen::Vector3d vel_commands = Eigen::Vector3d::Zero();
	Eigen::Vector3d gx_des	 = Eigen::Vector3d::Zero();
	Eigen::Vector3d gxd_des  = Eigen::Vector3d::Zero();
	Eigen::Vector3d gxdd_des = Eigen::Vector3d::Zero();
	
	Eigen::Vector3d gx       = Eigen::Vector3d::Zero();
	Eigen::Vector3d gxd      = Eigen::Vector3d::Zero();
	Eigen::Vector3d x_local  = Eigen::Vector3d::Zero();
	Eigen::Vector3d xd_local = Eigen::Vector3d::Zero();
	Eigen::Matrix<double, 8, 1> qd;
	Eigen::Matrix<double, 8, 1> qd_des;
	Eigen::Matrix<double, 4, 1> joint;
	Eigen::Matrix<double, NUM_MOTORS, 1> tq_des;
	Eigen::Vector3d cf_des_local;
	Eigen::Matrix3d lambda;
	Eigen::MatrixXd C_pinv;
	Eigen::Matrix<double, NUM_CASTERS, 1> q_steer;
	Eigen::Vector3d gx_cam = Eigen::Vector3d::Zero();
	float th_cam = 0.0;

	double max_linear_dist = 0.02;
	double max_ang_dist = 0.1;

#ifdef LINE
	enum ln_case
	{
		STRAIGHT,
		SPIN
	};

	enum ln_case line_case = STRAIGHT;

	gx_des << 0.5, 0.0, 0.0;
#endif

#ifdef SQUARE
	enum sqr_case
	{
		BOTTOM_LEFT,
		BOTTOM_LEFT_TWIST,
		TOP_LEFT,
		TOP_LEFT_TWIST,
		TOP_RIGHT,
		TOP_RIGHT_TWIST,
		BOTTOM_RIGHT,
		BOTTOM_RIGHT_TWIST
	};

	enum sqr_case square_case = BOTTOM_LEFT;
#endif

#ifdef USING_OTG
	Eigen::VectorXd step_desired_position = gx_des;
	Eigen::VectorXd step_desired_velocity = gxd_des;
	OTG* otg = new OTG(gx, CONTROL_PERIOD_s);
	Eigen::Vector3d max_vel;
	max_vel << MAX_VEL_X, MAX_VEL_Y, MAX_VEL_TH;
	otg->setMaxVelocity(max_vel);
	otg->setMaxAcceleration(2.0);
	otg->setMaxJerk(20.0);
	otg->reInitialize(gx);
#endif

	// Disable HB since we are receiving synced status update.
	struct CO_message msg_hb_disable;
	msg_hb_disable.type = SDO_Rx;
	msg_hb_disable.m.SDO = {0x1017, 0x00, 0, 2};
	for (int k = 1; k < 9; k++)
		CO_send_message (vehicle->s, k, &msg_hb_disable);
	usleep(1000);

	/* initial sync message */
	CO_send_message (vehicle->s, 0, &msg);
	sleep_until (&next, CONTROL_PERIOD_ns);

	static bool vehicle_spinning = false;
	static unsigned long start_time = 0;

	/* Initialize loop timer */
	auto t_start = std::chrono::high_resolution_clock::now();
	auto t_previous_loop_start = t_start;

	//ros::start();
	//ros::Rate rate((int)(2./CONTROL_PERIOD_s));
	while (ros::ok())
	{
		/* ---------- first half of control loop ---------- */
		/* Get loop timestamp */
		auto t_loop_start = std::chrono::high_resolution_clock::now();
		double t_delta = (std::chrono::duration<double, std::milli>(t_loop_start - t_previous_loop_start).count()); // Get time it took to complete one loop to plot
		t_previous_loop_start = t_loop_start;

	  	/* Update odometry */
		vehicle->updateOdometry();

	#ifdef CAMERA
		/* Update camera odometry */
		cameraT265->updatePose();
	#endif

	  	/* send sync message */
		//CO_send_message (vehicle->s, 0, &msg);
		//usleep (3500);
		//rate.sleep();

		

		/***************** Write data to csv file *********************************/
		if (dumpData)
		{
			gx = vehicle->getGlobalPosition();  //Odom
			gxd = vehicle->getGlobalVelocity(); //Odom
			tq_des = vehicle->getDesJointTorques();
			cf_des_local = vehicle->getLocalCommandForces();
			lambda = vehicle->getLambda();
			q_steer = vehicle->getJointSteeringAngles();
			x_local = vehicle->getLocalPosition();
			xd_local = vehicle->getLocalVelocity();
			C_pinv = vehicle->getCPinv();
			qd = vehicle->getJointVelocities();
			file << (ticks*CONTROL_PERIOD_s) << "," << t_delta << ",";
			file << gx(0) << "," << gx(1) << "," << gx(2) << ",";
			file << gx_des(0) << "," << gx_des(1) << "," << gx_des(2) << ",";
			file << gxd(0) << "," << gxd(1) << "," << gxd(2);
			for(int i = 0; i < NUM_MOTORS; i++){
				file << "," << tq_des(i);
			}
			file << "," << cf_des_local(0) << "," << cf_des_local(1) << "," << cf_des_local(2);
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					file << "," << lambda(i,j);
				}
			}
			file << "," << q_steer(0) << "," << q_steer(1) << "," << q_steer(2) << "," << q_steer(3) << ",";
			file << step_desired_position(0) << "," << step_desired_position(1) << "," << step_desired_position(2) << ",";
			file << step_desired_velocity(0) << "," << step_desired_velocity(1) << "," << step_desired_velocity(2) << ",";
		#ifdef CAMERA
			gx_cam = cameraT265->getCameraPosition();
			th_cam = cameraT265->getThCam();
			file << gx_cam(0) << "," << gx_cam(1) << "," << gx_cam(2);
			file << "," << th_cam;
		#else
			file << 0 << "," << 0 << "," << 0;
			file << "," << th_cam;
		#endif
			file << "," << vehicle->getHeading();
			file << "," << x_local(0) << "," << x_local(1) << "," << x_local(2);
			file << "," << xd_local(0) << "," << xd_local(1) << "," << xd_local(2);
			file << "," << qd(0) << "," << qd(1) << "," << qd(2) << "," << qd(3);
			for (int i = 0; i < 24; i++){
				file << "," << C_pinv(i);
			}
			file << endl;
		}
		/**************************************************************************/

		switch(control_mode) {
			case TORQUE:
			{
				auto t_end = std::chrono::high_resolution_clock::now();
				double t_curr = (std::chrono::duration<double, std::milli>(t_end-t_start).count())/1000;
				// gx_des << 0.25 * cos(t_curr/4), 0.25*sin(t_curr/4), 0;
				// gx_des << 2.0* fabs(cos(t_curr/6)), 0, t_curr/3.0; // Follow a line
				// gx_des << 0.5, 0, 0;

				gx = vehicle->getGlobalPosition();

			#ifdef SPIN
				gx_des << 0.0, 0.0, t_curr; // Spin in place
				// Change directions after 10 seconds
				// if (t_curr > 10) {
				// 	gx_des << 0.0, 0.0, 10-t_curr;
				// }
			#endif

			#ifdef LINE
				if(vehicle->reachedTarget(gx, gx_des, max_linear_dist, max_ang_dist))
				{
					switch(line_case){
						case STRAIGHT:
						{
							// gx_des[2] += M_PI/3.0;
							line_case = SPIN;
							break;
						}
						case SPIN:
						{
							gx_des[0] = -gx_des[0];
							line_case = STRAIGHT;
							break;
						}
					}
				}
			#endif

			#ifdef SQUARE
  				if(vehicle->reachedTarget(gx, gx_des, max_linear_dist, max_ang_dist))
  				{
					switch(square_case) {
						case BOTTOM_LEFT:
						{
							gx_des[2] += M_PI / 2.0;
							square_case = BOTTOM_LEFT_TWIST;
							break;
						}
						case BOTTOM_LEFT_TWIST:
						{
							gx_des[0] += 0.5;
							square_case = TOP_LEFT;
							break;
						}
						case TOP_LEFT:
						{
							gx_des[2] += M_PI / 2.0;
							square_case = TOP_LEFT_TWIST;
							break;
						}
						case TOP_LEFT_TWIST:
						{
							gx_des[1] += 0.5;
							square_case = TOP_RIGHT;
							break;
						}
						case TOP_RIGHT:
						{
							gx_des[2] += M_PI/ 2.0;
							square_case = TOP_RIGHT_TWIST;
							break;
						}
						case TOP_RIGHT_TWIST:
						{
							gx_des[0] -= 0.5;
							square_case = BOTTOM_RIGHT;
							break;
						}
						case BOTTOM_RIGHT:
						{
							gx_des[2] += M_PI / 2.0;
							square_case = BOTTOM_RIGHT_TWIST;
							break;
						}
						case BOTTOM_RIGHT_TWIST:
						{
							gx_des[1] -= 0.5;
							square_case = BOTTOM_LEFT;
							break;
						}

					}
  				}
  			#endif

			gx_des << gx_des_g[0], gx_des_g[1], gx_des_g[2];
			gxd_des << gxd_des_g[0], gxd_des_g[1], gxd_des_g[2];
			gxdd_des << gxdd_des_g[0], gxdd_des_g[1], gxdd_des_g[2];
			#ifdef USING_OTG
				otg->setGoalPositionAndVelocity(gx_des, gxd_des);
				otg->computeNextState(step_desired_position, step_desired_velocity);
				vehicle->setTargets(step_desired_position,step_desired_velocity,gxdd_des);
   			#else
				vehicle->setTargets(gx_des, gxd_des, gxdd_des);                         // Torque Control Input
			#endif
				break;
			}

			case VELOCITY:
			{
				vel_commands << cur_x_dot, cur_y_dot, cur_theta_dot;
				vehicle->setGlobalVelocity(vel_commands);
				//printf("%f, %f, %f\r\n", vel_commands(0), vel_commands(1), vel_commands(2));
				break;
			}

			default:
			{
				delete vehicle;
				cout << "Unimplemented control mode: " << control_mode << endl;
				return 0;
			}
		}

		//rate.sleep();
		//sleep_until (&next, CONTROL_PERIOD_ns/2); 
/* --------- second half of control loop --------- */
	  /* send sync message */
		//usleep(1500);
		CO_send_message (vehicle->s, 0, &msg);

		ticks++;

		/* write to file
		if (dumpData)
		{
		}
		*/

	  /* do nothing in this half */
		sleep_until (&next, CONTROL_PERIOD_ns);
		//rate.sleep();
	}
  printf("Exiting control thread ... ");
  raise (SIGINT);

}



/*
 * Helper function to handle accurate sleeping inside of the control
 * thread. This provides an easier abstraction to the clock_nanosleep
 * interface
 */
static void
sleep_until (struct timespec *ts, long delay)
{
	ts->tv_nsec += delay;
	if (ts->tv_nsec >= 1000*1000*1000)
	{
		ts->tv_nsec -= 1000*1000*1000;
		ts->tv_sec++;
	}
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL);
}


/*
 * Function for startup tasks. Installs the signal handler for SIGINT signals,
 * and opens a message queue with the controller if we're not testing.
 */
static void
perform_startup_tasks (void)
{
	/* install the signal handler for abnormal program exit */
	struct sigaction sa = {0};
	sa.sa_handler = sig_handler;
	sigfillset (&sa.sa_mask);
	sigaction (SIGINT, &sa, NULL);

#ifdef JOYSTICK
	/* open message queue with joystick */

	const char *name = "/joystick message queue";
	mq_joystick = mq_open (name, 0);

	if (mq_joystick == -1)
	{
		perror ("failed to open message queue in init_mQueue\n");
		exit (-1);
	}
#endif
}


/*
 * Parses command line args and sets various environment variables
 */
static void
parse_command_args (int argc, char *argv[])
{
	for (int i = 0; i < argc; i++)
	{
		std::string str (argv[i]);
		if (str == "--dump")
			dumpData = true;
	}
}


/*
 * Signal handler for SIGINT, used to properly destruct the vehicle object
 * if the program aborts abnormally
 */
static void
sig_handler (int)
{
	cout << endl;
	cout << "SIGINT received, destroying vehicle and exiting" << std::endl;
	delete vehicle;
	if (dumpData)
	{
		file.close ();
		cout << "Closing file " << endl;
	}
	exit (0);
}


/*
 * Registers keystrokes and returns 1 if a key has been hit. Note, this function
 * only works on Linux
 */
static int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if((ch != EOF) && (ch!='\r'))
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}
