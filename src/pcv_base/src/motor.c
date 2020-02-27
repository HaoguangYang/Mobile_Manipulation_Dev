/*------------------------------includes--------------------------------------*/
/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <mqueue.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <inttypes.h>

#include "../include/CAN_utils.h"
#include "../include/COB_ID.h"
#include "../include/CO_objects.h"
#include "../include/CO_message.h"
#include "../include/RT_utils.h"
#include "../include/motor_init_sequence.h"
#include "../include/motor.h"
#include "../include/definitions.h"

/*------------------------------structs---------------------------------------*/
struct motor
{
	pthread_mutex_t lock;				/* lock for serial access */
	pthread_t listener;					/* the listener thread */
	mqd_t mQueue;						/* message queue */
	char *mQueue_name;
	int s;								/* socket for CAN transmission */
	uint8_t no;							/* motor number (1-8) */
	double cur_pos;						/* most recent position received */
	double cur_vel;						/* most recent velocity received */
	double cur_trq;						/* most recent torque received */
	timer_t msg_timer;					/* timer for handling tx timeouts */
	timer_t heartbeat_timer;			/* timer for tracking heartbeat */
	enum ctrl_mode cm;					/* control mode (velocity or torque) */
	enum motor_type mt;					/* motor type (steering or rolling) */
	bool enabled;						/* is the motor software enabled? */
	bool enable_pin_active;             /* is the enable pin active */
	bool stale_pos;
	bool stale_vel;
	bool stale_trq;
	int32_t inputs;
};

/*------------------------static function declarations------------------------*/
static int init_mQueue (struct motor *m);
static int home_motor (struct motor *m);
static int init_motor (struct motor *m);
static int32_t get_cal_offset (struct motor *m);

static void *listener (void *aux);

static void msg_timer_handler (union sigval val);
static void heartbeat_timer_handler (union sigval val);

static int32_t position_SI_to_IU (double position_SI);
static double position_IU_to_SI (int32_t position_IU);
static int32_t velocity_SI_to_IU (double velocity_SI);
static double velocity_IU_to_SI (int32_t velocity_IU);
static int32_t accel_SI_to_IU (double velocity_SI);
static double accel_IU_to_SI (int32_t velocity_IU);
static int16_t torque_SI_to_IU (double torque_SI);
static double torque_IU_to_SI (int16_t torque_IU);


/*------------------------static variable declarations------------------------*/
static const double home_offsets[] = {HOME_OFFSET_MTR_1,
                                			HOME_OFFSET_MTR_3,
                                			HOME_OFFSET_MTR_5,
                                			HOME_OFFSET_MTR_7};

/*---------------------------public functions---------------------------------*/

/*
 * This function initializes motor_no and sets it up for control mode cm,
 * given its particular type. Returns NULL pointer if any part fails
 */
struct motor *
motor_init (uint8_t motor_no, enum ctrl_mode cm, enum motor_type mt)
{
    printf("motor_no: %d\n", motor_no);
    printf("cm: %d\n", cm);
    printf("mt: %d\n", mt);

	assert (0 < motor_no && motor_no <= NUM_MOTORS);
	struct motor *m;
	bool success = false;

	/* malloc the struct */
	m = malloc (sizeof (struct motor));
	if (m == NULL) {
        return NULL;
    }

	/* initialize the motor struct member */
	pthread_mutex_init (&m->lock, NULL);

	m->no = motor_no;
	m->cm = cm;
	m->mt = mt;
	m->enabled = false;
	m->enable_pin_active = true;
	m->stale_pos = true;
	m->stale_vel = true;
	m->stale_trq = true;

	/* open a CAN socket for this motor no */
	m->s = create_can_socket (m->no, 0xF);
	if (m->s < 0){
		printf("CAN socket creation FAILED!\r\n");
		goto done;
	}

	/* initialize timers */
	if (init_rt_timer (&m->msg_timer, msg_timer_handler, m)){
		printf("Init RT MSG timer FAILED!\r\n");
		goto done;
	}

	if (init_rt_timer (&m->heartbeat_timer, heartbeat_timer_handler, m)){
		printf("Init RT HeartBeat timer FAILED!\r\n");
		goto done;
	}

	/* initialize the message queue for this motor */
	if (init_mQueue (m)){
		printf("Init mQueue FAILED!\r\n");
		goto done;
	}

	/* launch the listening thread */
	if (launch_rt_thread (listener, &m->listener, m, MAX_PRIO - 1)){
		printf("Launch RT thread FAILED!\r\n");
		goto done;
	}

	/* now perform the initialization for the particular drive mode */
	if (init_motor (m)){
		printf("Init motor drive FAILED!\r\n");
		goto done;
	}

	/* home the motor if STEERING motor */
	if (m->mt == STEERING)
	{
		if (home_motor (m)){
			printf("Motor homing FAILED!\r\n");
			goto done;
		}
	}
	success = true;

	done:
		/* perform cleanup if any step failed */
		if (!success)
		{
			motor_destroy (m);
			m = NULL;
		}
	return m;
}


/*
 * Sends out RPDO2 with the given velocity data (rad/s) to control a
 * new velocity
 */
void
motor_set_velocity (struct motor *m, double velocity)
{
	assert (m != NULL);

	if (m->enabled)
	{
		/* ignore call if the motor is not in VELOCITY mode*/
		if (m->cm == VELOCITY)
		{
			uint32_t ui_vel = (uint32_t)velocity_SI_to_IU (velocity);
			struct CO_message msg = {RPDO, .m.PDO = {2, ui_vel, 4}};
			CO_send_message (m->s, m->no, &msg);
		}
	}
}


/*
 * Returns the most recently received velocity value in SI units (rad/s). If
 * no new update received since the last call to motor_get_velocity, the
 * function returns -1
 */
int
motor_get_velocity (struct motor *m, double *velocity)
{
	assert (m != NULL);
	assert (velocity != NULL);
	int retVal = -1;

	pthread_mutex_lock (&m->lock);
	*velocity = m->cur_vel;
	if (!m->stale_vel)
	{
		m->stale_vel = true;
		retVal = 0;
	}
	pthread_mutex_unlock (&m->lock);

	return retVal;
}


/*
 * Returns the most recently received position value in SI units (rad). If
 * no new update received since the last call to motor_get_position, the
 * function returns -1
 */
int
motor_get_position (struct motor *m, double *position)
{
	assert (m != NULL);
	assert (position != NULL);
	int retVal = -1;

	pthread_mutex_lock (&m->lock);
	*position = m->cur_pos;
	if (!m->stale_pos)
	{
		m->stale_pos = true;
		retVal = 0;
	}
	pthread_mutex_unlock (&m->lock);

	return retVal;
}


/*
 * Returns true if the bumper next to the motor is hit.
 */
bool
motor_get_inputs(struct motor *m)
{
	assert(m != NULL);
	pthread_mutex_lock(&m->lock);
	int32_t inputs = m->inputs;
	bool retVal = inputs & BIT21HI;
	pthread_mutex_unlock(&m->lock);
	return retVal;
}


/*
 * Enables a motor, returns 0 on success and -1 on failure
 */
int
motor_enable (struct motor *m)
{
	assert (m != NULL);
	struct event e;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = HOME_TIMEOUT;

	/* send the proper mode in the first message of enable sequence */
	struct CO_message msg = enable_sequence[0];
	switch (m->cm)
	{
		case HOMING:
			printf("Motor %d: Home mode case in motor_enable\n", m->no);
            msg.m.SDO.data += HOME_MODE;
            printf("Value: %hhd\n", msg.m.SDO.data);
			break;

		case VELOCITY:
			printf("Motor %d: Velocity mode case in motor_enable\n", m->no);
            msg.m.SDO.data += VELOCITY_MODE;
			printf("Value: %hhd\n", msg.m.SDO.data);
            break;

		case TORQUE:
            printf("Motor %d: Torque mode case in motor_enable\n", m->no);
			msg.m.SDO.data += TORQUE_MODE;
            printf("Value: %hhd\n", msg.m.SDO.data);
			break;
	}

	/* send first message to kick off enable sequence */
	unsigned i = 0;
	CO_send_message (m->s, m->no, &msg);
	timer_settime (m->msg_timer, 0, &itmr, NULL);

	while (i < NUM_ENABLE_STEPS)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);

		if (e.type == TIMEOUT)
		{
			puts ("msg timer timeout, motor_enable failed");
			return -1;
		}
		else if (e.type == SDO_WR_ACK && e.param == MODES_OF_OPERATION)
		{
			CO_send_message (m->s, m->no, &enable_sequence[++i]);
			timer_settime (m->msg_timer, 0, &itmr, NULL);
		}
		else if (e.type == STATUS_WRD_REC &&
							(e.param & ENABLE_Rx_MASK) == enable_responses[i].param)
		{
			if (++i < NUM_ENABLE_STEPS)
			{
				CO_send_message (m->s, m->no, &enable_sequence[i]);
				timer_settime (m->msg_timer, 0, &itmr, NULL);
			}
		}
	}

	/* stop timer */
	itmr.it_value.tv_sec = 0;
	timer_settime (m->msg_timer, 0, &itmr, NULL);

	/* set to enabled */
	m->enabled = true;
	return 0;
}


/*
 * Interface for disabling the motor
 */
void
motor_disable (struct motor *m)
{
	assert (m != NULL);
	struct CO_message msg = {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x00, 2}};
	CO_send_message (m->s, m->no, &msg);
	m->enabled = false;
}


/*
 * Sends out RPDO2 with the given torque data (n/m) to control a
 * new torque
 */
void
motor_set_torque (struct motor *m, double torque)
{
	assert (m != NULL);

	/* ignore call if the motor is not in torque mode*/
    if (m->enabled) {
        if (m->cm == TORQUE)
	    {
      	//printf("Motor torque: %f \r\n", torque);
	    // if(abs(torque) > TORQUE_CONT){
	    // 	printf( "Motor torque too high!");
     //    	torque = ( (torque > 0) - (torque < 0) ) * TORQUE_CONT;
     //    	printf("New torque: %f", torque);

     //  	}
	    uint16_t ui_trq = (uint16_t)torque_SI_to_IU (torque);
	    //printf("UI torque: %u \r\n", ui_trq);
        struct CO_message msg = {RPDO, .m.PDO = {2, (ui_trq<<16), 4}};
	    CO_send_message (m->s, m->no, &msg);
	    }
    }
}


/*
 * Returns the most recently received torque value in SI units (n/m). If
 * no new update received since the last call to motor_get_torque, the
 * function returns -1
 */
int
motor_get_torque (struct motor *m, double *torque)
{
	assert (m != NULL);
	assert (torque != NULL);
	int retVal = -1;

	pthread_mutex_lock (&m->lock);
	*torque = m->cur_trq;
	if (!m->stale_trq)
	{
		m->stale_trq = true;
		retVal = 0;
	}
	pthread_mutex_unlock (&m->lock);

	return retVal;
}


/* interface to the motor control mode */
void
motor_set_ctrl_mode (struct motor *m, enum ctrl_mode cm)
{
	assert (m != NULL);
	m->cm = cm;
	// puts ("motor_set_ctrl_mode not implemented");
}


/*
 * Query function for the control mode of the motor (VELOCITY or TORQUE)
 */
enum ctrl_mode
motor_get_ctrl_mode (struct motor *m)
{
	assert (m != NULL);
	return m->cm;
}


/*
 * Query function for the motor type (ROLLING or STEERING)
 */
enum motor_type
motor_get_type (struct motor *m)
{
	assert (m != NULL);
	return m->mt;
}


/*
 * Stop the motor
 */
void
motor_stop (struct motor *m)
{
	assert (m != NULL);
  	if(m->cm == TORQUE){
    	motor_set_torque(m, 0.);
    	printf("ZERO TORQUE... \r\n");
  	}
  	else if(m->cm == VELOCITY){
    	motor_set_velocity(m, 0.);
    	printf("ZERO VELOCITY... \r\n");
   	}
}


/*
 * Destroy the motor object
 */
void motor_destroy (struct motor *m)
{
	if (m == NULL)
		return;

	motor_disable (m);
	free (m->mQueue_name);
	timer_delete (m->msg_timer);
	timer_delete (m->heartbeat_timer);
	mq_close (m->mQueue);
	close (m->s);
	pthread_cancel(m->listener);
	free (m);
}

bool
is_motor_enable_pin_active(struct motor *m){
	return m->enable_pin_active;
}

/*------------------------------static functions------------------------------*/
/*
 * The timer handler for the msg timer, sends a timeout event to the message
 * queue
 */
static void
msg_timer_handler (union sigval val)
{
	struct motor *m = val.sival_ptr;
	struct event e;
	e.type = TIMEOUT;
	e.param = (uintptr_t)m->msg_timer;
	mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
	printf("TIMEEEE/r/n");
}

/*
 * The timer handler for the msg timer, raises a SIGINT signal to signal
 * shutdown
 */
static void
heartbeat_timer_handler (union sigval val)
{
	puts ("Motor controller heartbeat timeout, killing the process");
	raise (SIGINT);
}


/*
 * Functions for unit conversion. These output values at the motor shaft.
 * Gear ratios included at caster level.
 * Position SI = rad
 * Position IU = encoder ticks
 * Velocity SI = rad/s
 * Velocity IU = encoder ticks/slow loop sampling period
 * Torque SI = Nm
 * Torque IU = Motor phase current (A)
 */
static inline int32_t
position_SI_to_IU (double position_SI)
{
	return (int32_t) (position_SI * POS_MULTIPLIER *
		(Ns * ENCODER_TICKS / TWO_PI));
}

static inline double
position_IU_to_SI (int32_t position_IU)
{
	return (double) (position_IU *
		(TWO_PI / (Ns * ENCODER_TICKS)) / POS_MULTIPLIER);
}

static inline int32_t
velocity_SI_to_IU (double velocity_SI)
{
	return (int32_t) (velocity_SI * VEL_MULTIPLIER *
		(Ns * ENCODER_TICKS * SLOW_LOOP_SAMP_PERIOD / TWO_PI));
}

static inline double
velocity_IU_to_SI (int32_t velocity_IU)
{
	return  (double) velocity_IU * (TWO_PI / (Ns * ENCODER_TICKS *
		SLOW_LOOP_SAMP_PERIOD)) / VEL_MULTIPLIER;
}

static inline int32_t
accel_SI_to_IU (double accel_SI)
{
	return (int32_t) (accel_SI * ACCEL_MULTIPLIER * (Ns *
		ENCODER_TICKS * SLOW_LOOP_SAMP_PERIOD * SLOW_LOOP_SAMP_PERIOD / TWO_PI));
}

static inline double
accel_IU_to_SI (int32_t accel_IU)
{
	return (double) (accel_IU * (TWO_PI / (Ns * ENCODER_TICKS *
		SLOW_LOOP_SAMP_PERIOD * SLOW_LOOP_SAMP_PERIOD)) / ACCEL_MULTIPLIER);
}

static inline int16_t
torque_SI_to_IU (double torque_SI)
{
	return (int16_t) (torque_SI * CURRENT_DENOM /
		(TORQUE_CONSTANT * CURRENT_NUM * CURRENT_PEAK));
}

static inline double
torque_IU_to_SI (int16_t torque_IU)
{
	return (double) (torque_IU * TORQUE_CONSTANT * CURRENT_NUM *
		CURRENT_PEAK / CURRENT_DENOM);
}


/*
 * Initialize the message queue that is used for communication between the
 * listening thread and the thread that calls the motor API
 */
static int
init_mQueue (struct motor *m)
{
	m->mQueue_name = malloc (MAX_NAME_LEN);
	if (m->mQueue_name == NULL)
	{
		perror ("call to malloc failed in init_mQueue\n");
		return -1;
	}

	sprintf(m->mQueue_name, "/motor %u message queue", m->no);
	mq_unlink (m->mQueue_name);
	struct mq_attr attr;
	attr.mq_maxmsg = QUEUE_SIZE;
	attr.mq_msgsize = sizeof (struct event);
	attr.mq_flags = 0;
	m->mQueue = mq_open (m->mQueue_name, O_RDWR | O_CREAT, 0664, &attr);

	if (m->mQueue == -1)
	{
		perror ("failed to open message queue in init_mQueue\n");
		return -1;
	}

	return 0;
}


/*
 * Implementation of an exponential moving average filter for the motor
 * velocity or torque measurement
 */
static double
filter (double old_vel, double new_sample, double coefficient)
{
	return (old_vel + (1.0 - coefficient) * (new_sample - old_vel));
}


/*
 * This is the listener thread, it constantly reads from the CAN socket and
 * translates the frames into events that it sends to the message queue to
 * other threads that might be interested in the events
 */
static void *
listener (void *aux)
{
	struct motor *m = aux;
	struct event e;
	struct can_frame f;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = HEARTBEAT_TIMEOUT;
	printf("Motor listener thread launched!\r\n");

	/* listen for messages forever, thread gets cancelled on motor_destroy call */
	while (1)
	{
	  	int nbytes = read(m->s, &f, sizeof(struct can_frame));

	  	if (nbytes < 0) {
	      perror ("can raw socket read\n");
	      exit (-1);
	  	}
	    //printf("Can ID: %X\n", f.can_id);

	  	/* translate the can frame */
	  	if (f.can_id == COB_ID_NMT_EC_TX(m->no)) /* NMT */
	  	{
	  		if (f.data[0] == 0x05) /* heartbeat in operational state */
	  		{
	  			/* restart the heartbeat timer */
	  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	  		}
	  		else if (f.data[0] == 0x00) /* bootup */
	  		{
	  			e.type = NMT_EC_REC;
	  			e.param = f.data[0];
	  			mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
	  		}
	  	}
	  	else if (f.can_id == COB_ID_TPDO (m->no, 1)) /* TPDO1 - status word*/
	  	{
	  		/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
			uint16_t *data = (uint16_t *)&f.data;
	  		e.type = STATUS_WRD_REC;
	  		e.param = data[0];
	  		mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
	  	}
	  	else if (f.can_id == COB_ID_TPDO (m->no, 2)) /* TPDO2 - {pos, vel/trq}*/
	  	{
	  		int32_t *data = (int32_t *)&f.data; // why not uint32_t
	  		/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	  		/* critical section */
	  		pthread_mutex_lock (&m->lock);
	  		m->cur_pos = position_IU_to_SI (data[0]);
		    m->stale_pos = false;

			/* translate the last 4 bytes of data into vel */
		    m->cur_vel = filter (m->cur_vel, velocity_IU_to_SI (data[1]), LP_VEL_FILTER_COEFF);
		    m->stale_vel = false;
			pthread_mutex_unlock (&m->lock);
			/* end of critical section */
	  	}
	    else if (f.can_id == COB_ID_TPDO (m->no, 3)) /* TPD03 - {current, modes of operation */
	    {
	        int32_t *data = (int32_t *)&f.data; // why not uint16_t?
			/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	        //printf("data[0]: %X\ndata[1]: %X\n\n",data[0],data[1]);
	        pthread_mutex_lock (&m->lock);
	        m->cur_trq = filter (m->cur_trq, torque_IU_to_SI (data[0]), LP_TRQ_FILTER_COEFF);
	        m->stale_trq = false;
	        pthread_mutex_unlock (&m->lock);
	    }
	    else if (f.can_id == COB_ID_TPDO (m->no, 4)) /* TPD04 - digital inputs */
	    {
	    	int32_t *data = (int32_t *)&f.data; // why not uint32_t?
			/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	    	/* critical section */
	  		pthread_mutex_lock (&m->lock);
	  		m->inputs = data[0];
	  		pthread_mutex_unlock (&m->lock);
	  		/* end of critical section */
	    }
	  	else if (f.can_id == COB_ID_SDO_TX (m->no)) /* SDO */
	  	{
	  		if (f.data[0] == CO_WRITE_Ack)
	  		{
	  			/* send an SDO_WR_ACK event and save the object index in param */
	  			e.type = SDO_WR_ACK;
	  			e.param = *(uint16_t *)&(f.data[1]);
	  			mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
	  		}
			/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	  	}
	    else if (f.can_id == COB_ID_EMCY_TX (m->no)) /* Emergency message */
	    {
			/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	    	if ((f.data[0] == 0x41) && (f.data[1] == 0x54))
	    	{
	    		m->enable_pin_active = false;
	    		printf("Motor %d enable pin DISABLED\n", m->no);
			} else if ((f.data[0] == 0x00) && (f.data[1] == 0x00)) {
				printf("Error reset or no error \n");
				m->enable_pin_active = true;
			}else {
	    		//printf("Data 1: %u \n", f.data[0]);
	    		//printf("Data 2: %u \n", f.data[1]);
		        //printf("Error message received\n");
		        //while(1) {}
	    	}
		}
	}
}


/*
 * Initializes the motor, this function steps through the array of messages
 * inside of motor_init_sequence.h. It sends a message and then waits for
 * an event verifying success of that transmission. If at any point a timeout
 * occurs, this is translated as an error and the function returns -1
 */
static int
init_motor (struct motor *m)
{
	struct event e;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = MSG_TIMEOUT;

	/* send first message to kick off */
	unsigned i = 0;
	CO_send_message (m->s, m->no, &init_sequence[i]);
	timer_settime (m->msg_timer, 0, &itmr, NULL);

	while (i < NUM_INIT_STEPS)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		if (e.type == TIMEOUT)
		{
			printf("init timer timeout, init_motor failed");
			return -1;
		}
		if (e.type == init_responses[i].type && e.param == init_responses[i].param)
		{
			if (++i < NUM_INIT_STEPS)
			{
				struct CO_message cur = init_sequence[i];

				/* make sure to add the motor number to the data field for PDO inits
				   and home offsets */
				if (cur.type == SDO_Rx)
				{
                    //printf("Writing to index %X with subindex %X, %X\n", cur.m.SDO.index, cur.m.SDO.subindex, cur.m.SDO.data);
                    switch (cur.m.SDO.index)
					{
						case RPDO1_COMM:
						case RPDO2_COMM:
						case RPDO3_COMM:
						case RPDO4_COMM:
						case TPDO1_COMM:
						case TPDO2_COMM:
						case TPDO3_COMM:
						case TPDO4_COMM:
							if (cur.m.SDO.subindex == 0x01)
								cur.m.SDO.data += m->no;
							break;

						case HOME_OFFSET:
							cur.m.SDO.data = get_cal_offset (m);
							break;

						default:
							break;
					}
				}

				CO_send_message (m->s, m->no, &cur);
				timer_settime (m->msg_timer, 0, &itmr, NULL);
			}
		}
        else {
            printf("Response error | ");
            printf("Type: %X, Param: %X\n", e.type, e.param);
            printf("Expected vals  | ");
            printf("Type: %X, Param: %X\n", e.type, e.param);
        }
	}

	/* stop timer */
	itmr.it_value.tv_sec = 0;
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	return 0;
}


/*
 * Homes the motor by setting the control mode field to HOMING and calling
 * enable, then waits for a STATUS_WORD with data 0xD637 signaling that
 * homing is finished
 */
static int
home_motor (struct motor *m)
{
	/* first enable the motor */
	enum ctrl_mode old = m->cm;
	m->cm = HOMING;
	motor_enable (m);
	m->cm = old;

	/* perform the homing sequence */
	struct event e;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = HOME_TIMEOUT;

	while (1)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);

		if (e.type == TIMEOUT)
		{
			puts ("msg timer timeout, home_motor failed");
			return -1;
		}
		if (e.type == STATUS_WRD_REC && e.param == 0xD637)
			break;
	}

	/* disable motor */
	motor_disable (m);

	/* stop timer */
	itmr.it_value.tv_sec = 0;
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	return 0;
}


/*
 * Returns the proper home offset to write during motor initialization.
 * If the calibration routine was run and the file exists with the offsets,
 * those are factored in to the value
*/
static int32_t
get_cal_offset (struct motor *m)
{
	double retVal, calVal, difference;
	char *buffer = NULL;
	size_t n;
	FILE *fp;

	/* assign initial offset, steer motors have offset of zero */
	if (m->mt == ROLLING)
		return 0;

	retVal = home_offsets[m->no/2];

	/* try to open motor_cal file */
	fp = fopen ("./.motor_cal.txt", "r");
	if (fp != NULL)
	{
		/* cal value is positioned by line in .motor_cal
			 i.e. cal of motor 1 = line 1
			      cal of motor 3 = line 2
			      cal of motor 5 = line 3 .... */
		int i;
		for (i = 0; i <= (m->no / 2); i++)
			getline (&buffer, &n, fp);

		calVal = atof (buffer);

		free (buffer);
		fclose (fp);

		/* calculate the offset value and add it to the normal offset */
		difference = retVal + calVal;
		retVal += difference;
	}

	return position_SI_to_IU (retVal * Ns);
}

/*---------------------------------test harness-------------------------------*/
#ifdef TEST_MOTOR

//#define LOG_COUPLED_MOTOR // if uncommented then intialize joint motor and log values

#define CONTROL_PERIOD_ns 	(700000)
#define CONTROL_PERIOD_s 		(0.007)
#define MAX_VEL							(7*3.14159)		/* [rad/s] */
#define MAX_TOR             (2.0)         /* [Nm] */
#define X_freq							(0.1)					/* [hz] */
#define MAX_VEL_IU					(0x100000)

static void * control_thread (void *aux);
static bool done = false;
static int test_mode = 0;

#ifdef LOG_COUPLED_MOTOR
static void * log_thread (void *aux);
#endif

void
main(char argc, char *argv[])
{
	if (argc < 4)
	{
		puts ("Usage: sudo ./test <motor_no> <test_mode> <duration>");
		exit (0);
	}

	/* prompt to elevate the robot for testing */
	puts("Begin motor test harness");
	puts ("Elevate the robot so that all casters can spin freely");
	puts ("Press any key to continue when done");
	getchar ();

	/* Test unit conversion */
	// puts("\nPos IU\t\t\tPos SI\t\t\tPos_IU (reconverted)");
	// for(int pos_IU = -49000; pos_IU < 49000; pos_IU += 500){
	// 	double pos_SI = position_IU_to_SI(pos_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",pos_IU,pos_SI,position_SI_to_IU(pos_SI));
	// }

	// puts("\nVel IU\t\t\tVel SI\t\t\tVel_IU (reconverted)");
	// for(int vel_IU = -4700000; vel_IU < 4700000; vel_IU += 20000){
	// 	double vel_SI = velocity_IU_to_SI(vel_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",vel_IU,vel_SI,velocity_SI_to_IU(vel_SI));
	// }

	// puts("\nAccel IU\t\t\tAccel SI\t\t\tAccel_IU (reconverted)");
	// for(int accel_IU = -23000; accel_IU < 23000; accel_IU += 100){
	// 	double accel_SI = accel_IU_to_SI(accel_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",accel_IU,accel_SI,accel_SI_to_IU(accel_SI));
	// }

	// puts("\nTorque IU\t\t\tTorque SI\t\t\tTorque_IU (reconverted)");
	// for(int torque_IU = -35000; torque_IU < 35000; torque_IU += 100){
	// 	double torque_SI = torque_IU_to_SI(torque_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",torque_IU,torque_SI,torque_SI_to_IU(torque_SI));
	// }

  // Set motor number, test mode, and test duration from arguments
	int motor_no = atoi (argv[1]);
  test_mode = atoi (argv[2]);
  int duration = atoi (argv[3]);

  if(duration <= 0) {
      puts ("Test duration must be a positive value");
      exit(0);
  }

    #ifdef TEST_VELOCITY
    struct motor *m = motor_init (motor_no, VELOCITY, motor_no % 2);
    home_motor(m);
    m->cm = VELOCITY;  // Home motor changes command mode to HOMING, must be changed back to VELOCITY
    #endif

    #ifdef TEST_TORQUE
    struct motor *m = motor_init (motor_no, TORQUE, motor_no % 2);
    // home_motor(m);
    m->cm = TORQUE;  // Home motor changes command mode to HOMING, must be changed back to TORQUE
    #endif

	if (m == NULL)
	{
		puts ("motor_init failed");
		goto done;
	}

    #ifdef LOG_COUPLED_MOTOR
    int log_motor_no = 4 * ((motor_no + 1) / 2) - 1 - motor_no;
    struct motor *m_log = motor_init(log_motor_no, VELOCITY, log_motor_no % 2);

    if(m_log == NULL) {
        puts("motor_init failed for log motor");
        goto done;
    }
    #endif

    if(motor_enable (m))
	{
		puts ("motor_enable failed");
		goto done;
	}

    #ifdef LOG_COUPLED_MOTOR
    if(motor_enable(m_log)) {
        puts("motor_enable failed for log motor");
        goto done;
    }
    #endif

	/* Launch control thread */
	pthread_t control;
	launch_rt_thread (control_thread, &control, m, MAX_PRIO);

    #ifdef LOG_COUPLED_MOTOR
    pthread_t log;
    launch_rt_thread(log_thread, &log, m_log, MAX_PRIO);
    #endif

	/* sleep for some time and then stop control thread */;
	sleep (duration);
	done = true;
	pthread_join (control, NULL);

    #ifdef LOG_COUPLED_MOTOR
    pthread_join(log, NULL);
    #endif

done:
	motor_destroy (m);
    #ifdef LOG_COUPLED_MOTOR
    motor_destroy(m_log);
    #endif
	puts("");
	puts("End motor test harness");
}

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

static void *
control_thread (void *aux)
{
    printf("control thread launched with success\n");
	struct motor *m = aux;
	int s = create_can_socket (0xF, 0xF);
	struct CO_message msg;
	msg.type = SYNC;

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	/* open dump file */
	FILE *fp;
	fp = fopen("./traces/trace.csv", "w");
	fprintf(fp, "t,vel-command,tor-command,pos-actual,vel-actual,tor-actual\n");

  if (fp == NULL){
    printf("file could not be opened");
    return 0;
  }

	/* variables for sin, cos waves */
	unsigned long ticks = 0;
	double vel_command = 0, tor_command = 0, vel_actual, pos_actual, tor_actual, prev_pos = 0, prev_prev_pos = 0;
	int32_t vel_command_iu = 0, vel_actual_iu, pos_actual_iu;

	/* send initial synch message */
	CO_send_message (s, 0, &msg);
	sleep_until(&next, CONTROL_PERIOD_ns/2);

	while (!done)
	{
/* ---------- first half of control loop ---------- */
	  /* send sync message */
	  CO_send_message (s, 0, &msg);
	  usleep (3000);

	  motor_get_velocity (m, &vel_actual);
	  motor_get_position (m, &pos_actual);
      motor_get_torque (m, &tor_actual);

      #ifdef TEST_VELOCITY
      motor_set_velocity (m, vel_command);
      #endif

      #ifdef TEST_TORQUE
      motor_set_torque (m, tor_command);
      #endif

	  sleep_until (&next, CONTROL_PERIOD_ns/2);
/* --------- second half of control loop --------- */
	  /* send sync message */
	  CO_send_message (s, 0, &msg);

	 	/* write to file */
	 	fprintf(fp, "%f,%f,%f,%f,%f,%f\n", (ticks*(CONTROL_PERIOD_s/2)), vel_command, tor_command, pos_actual, vel_actual, tor_actual);

        vel_command = 1;
        tor_command = 1;
	 	/* calculate next */
    	switch(test_mode) {
    	case 218: //balance
    		tor_command = 0.974262;
        case 0: // Half speed
            vel_command = 0.5;
        case 1: // Full speed
            vel_command *= MAX_VEL;
            break;
        case 2: // Full speed sine wave
            vel_command = MAX_VEL;
        case 3: // Low speed sine wave
            vel_command *= -sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s));
            break;
        case 4: // Full speed square wave
            vel_command = MAX_VEL;
        case 5: // Low speed square wave
            vel_command *= (2 * signbit(sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s))) - 1);
            break;
        case 6: // Very slow constant speed
            vel_command = 2;
            break;
        case 100: // Half torque
            tor_command = 0.5;
        case 101: // Full torque
            tor_command *= MAX_TOR;
            break;
        case 102: // Super low sine wave
            tor_command = MAX_TOR * sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s));
            break;
        case 103: // Square wave
            tor_command = MAX_TOR*(2 * signbit(sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s))) - 1);
            break;
        default:
            puts("illegal test mode");
            vel_command = 0;
            break;
        }

        if(vel_command > MAX_VEL) {
            vel_command = MAX_VEL;
        } else if(vel_command < -MAX_VEL) {
            vel_command = -MAX_VEL;
        }

        if(tor_command > MAX_TOR) {
            tor_command = MAX_TOR;
        } else if(tor_command < -MAX_TOR) {
            tor_command = -MAX_TOR;
        }

        ticks++;

	  /* do nothing in this half */
		sleep_until (&next, CONTROL_PERIOD_ns/2);
	}
}

#ifdef LOG_COUPLED_MOTOR
static void *
log_thread (void *aux) {
    printf("log thread launched with success\n");
    struct motor *m_log = aux;
    int s = create_can_socket(0xF, 0xF);
    struct CO_message msg;
    msg.type = SYNC;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    FILE *fp;
    fp = fopen("./traces/trace_log.csv", "w");
    fprintf(fp, "t,vel,pos\n");

    unsigned long ticks = 1;
    double pos_actual, vel_actual;

    CO_send_message(s, 0, &msg);
    sleep_until(&next, CONTROL_PERIOD_ns/2);

    while(!done) {
        CO_send_message(s, 0, &msg);
        usleep(3000);
        sleep_until(&next, CONTROL_PERIOD_ns/2);
        CO_send_message(s, 0, &msg);

        motor_get_velocity(m_log, &vel_actual);
        motor_get_position(m_log, &pos_actual);

        fprintf(fp, "%f,%f,%f\n", (ticks*CONTROL_PERIOD_s), vel_actual, pos_actual);
        ticks++;

        sleep_until(&next, CONTROL_PERIOD_ns/2);
     }
}
#endif

#endif

/* cpp - c cross compilation */
