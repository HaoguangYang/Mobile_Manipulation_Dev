/*-----------------------------includes--------------------------------------*/
/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>

#include "../include/RT_utils.h"

/*-----------------------------defines---------------------------------------*/
#define MY_STACK_SIZE   			(100*1024) 

/*-----------------------------structs---------------------------------------*/

/*-----------------------static function declarations------------------------*/

/*---------------------------public functions--------------------------------*/
int 
launch_rt_thread (void *(*func)(void *), pthread_t *t, void *aux, int priority)
{
	assert (t != NULL);
	pthread_attr_t attr;
	struct sched_param param;

	/* Lock memory */
	if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) 
    exit(-2);

	/* Initialize pthread attributes (default values) */
	if (pthread_attr_init (&attr))
    return -1;

	/* Set a specific stack size  */
	if (pthread_attr_setstacksize (&attr, MY_STACK_SIZE))
    return -1;

	/* Set scheduler policy and priority of pthread */
	if (pthread_attr_setschedpolicy (&attr, SCHED_FIFO))
    return -1;

	param.sched_priority = priority;
	if (pthread_attr_setschedparam (&attr, &param))
    return -1;

	/* Use scheduling parameters of attr */
	if (pthread_attr_setinheritsched (&attr, PTHREAD_EXPLICIT_SCHED))
	  return -1;

	/* Create a pthread with specified attributes */
	if (pthread_create(t, &attr, func, aux))
    return -1;

  return 0;
}

int
init_rt_timer (timer_t *t, void (*handler)(union sigval val), void *aux)
{
	pthread_attr_t attr = {{0}};
    pthread_attr_init (&attr);
	struct sched_param parm;
    parm.sched_priority = 255;
	pthread_attr_setschedparam (&attr, &parm);

	struct sigevent sev = {{0}};
	sev.sigev_notify = SIGEV_THREAD;
	sev.sigev_value.sival_ptr = aux;
	sev.sigev_notify_function = handler;

	if (timer_create (CLOCK_REALTIME, &sev, t))
	{
		perror ("failed to create timer\n");
		return -1;
	}

	return 0;
}


/*---------------------------static functions--------------------------------*/

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif