#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>
#include <poll.h>
#include <pthread.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>
#include <linux/errno.h>

#include "buffer.h"
#include "libioctl.h"

#define DEVICE_SENSOR "/dev/sensor"


pthread_t polling_thread;
pthread_mutex_t mutex;

int stop = 0;
long my_distance;
int errno;

bufp userfifo[FIFOSIZE];

ioctl_data_t paramconfig;


/* Thread to calculate distance from object using POLL method */
void* polling_function(void* arg)
{

	int fd_sensor[10], i;
	char name[20];
	// int flag = 0;
	// double dist;
	int ret;
	int tail;

	int my_string = 0;

	for(i=0;i<2;i++){
		printf("Enter 1)Trigger and 2)Echo pin for Device %d\n\n", i+1);
		scanf("%d %d", &paramconfig.arg1, &paramconfig.arg2);

		sprintf(name, "/dev/HCSR_%d", i);

		fd_sensor[i] = open(name, O_RDWR);
		if(fd_sensor[i] == -1)
		{
			 	printf("file either does not exit or is currently used by an another user\n");
		 		exit(-1);
		}

		printf("Setting the IO pins TriggerPin=%d as output, EchoPin=%d as input\n", paramconfig.arg1, paramconfig.arg2);
		ret = ioctl(fd_sensor[i], CONFIG_PINS, &paramconfig);
		if(ret == -1)
			exit(-1);

		sleep(1);

	}

	paramconfig.arg1 = 4;
	paramconfig.arg2 = 60;
	printf("Setting the PARAMS m=%d samples, delta=%dms periodic sampling delay  on Device 1\n", paramconfig.arg1, paramconfig.arg2);
	ret = ioctl(fd_sensor[0], SET_PARAMETERS, &paramconfig);
	if(ret == -1)
		printf("ioctl failed\n");

	paramconfig.arg1 = 3;
	paramconfig.arg2 = 50;
	printf("Setting the PARAMS m=%d samples, delta=%dms periodic sampling delay for Device 2\n", paramconfig.arg1, paramconfig.arg2);
	ret = ioctl(fd_sensor[1], SET_PARAMETERS, &paramconfig);
	if(ret == -1)
		printf("ioctl failed\n");
	sleep(1);
	
	////////////////////////////////////////////////////////////////////////////////


	/* ******************* TEST0 START ********************* */

	printf("Starting Sensor operations\n\n");



	printf("Reading 1st to test Empty Buffer... Triggers a new measurement & reads the buffer Device 1\n");
	ret = read(fd_sensor[0], &userfifo, sizeof(bufp)*FIFOSIZE);
	sleep(1);
	for(tail = 0; tail < FIFOSIZE; tail++)
		printf("Buf %d  dist = %ld tsc = %lu\n", tail, userfifo[tail].distance, (unsigned long)userfifo[tail].timestamp);

	/* ******************* TEST0 END ********************* */
			
	/* ******************* TEST1 START ********************* */
#if 1
	printf("Performing 1st measurement on Device 2\n");	
	ret = write(fd_sensor[1], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Performing 2nd measurement on Device 1\n");	
	ret = write(fd_sensor[0], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Performing 3rd measurement on Device 1\n");	
	ret = write(fd_sensor[0], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Performing 4th measurement on Device 1\n");	
	ret = write(fd_sensor[0], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Performing 5th measurement on Device 1\n");	
	ret = write(fd_sensor[0], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Performing 6th measurement  on Device 2\n");	
	ret = write(fd_sensor[1], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Read from the existing Buffer after multiple writes  on Device 1\n");
	ret = read(fd_sensor[0], &userfifo, sizeof(bufp)*FIFOSIZE);
	sleep(1);
	for(tail = 0; tail < FIFOSIZE; tail++)
	printf("Buf %d  dist = %ld tsc = %lu\n", tail, userfifo[tail].distance, (unsigned long)userfifo[tail].timestamp);
#endif
	/* ******************* TEST1 END ********************* */

	/* ******************* TEST2 START ********************* */
#if 1
	paramconfig.arg1 = 6;
	paramconfig.arg2 = 60;
	printf("Setting the PARAMS m=%d samples, delta=%dms periodic sampling delay  on Device 2\n", paramconfig.arg1, paramconfig.arg2);
	ret = ioctl(fd_sensor[1], SET_PARAMETERS, &paramconfig);
	if(ret == -1)
		printf("ioctl failed\n");

	my_string = 2;
	printf("Performing new measurement with non-zero integer value to clear existing Buffer  on Device 2\n");	
	ret = write(fd_sensor[1], &my_string, sizeof(my_string));
	if(ret < 0)
		printf("Timer busy %d\n", errno);				
	sleep(1);

	printf("Read from the existing Buffer after clearing it previously  on Device 1\n");
	ret = read(fd_sensor[0], &userfifo, sizeof(bufp)*FIFOSIZE);
	for(tail = 0; tail < FIFOSIZE; tail++)
		printf("Buf %d  dist = %ld tsc = %lu\n", tail, userfifo[tail].distance, (unsigned long)userfifo[tail].timestamp);
	sleep(1);
#endif
	/* ******************* TEST2 END ********************* */


	close(fd_sensor[0]);
	close(fd_sensor[1]);

	return NULL;
}

int main()
{

	int ret1;

	printf("Trigger Pin from range 0-19\n");
	printf("Echo Pin from 0,1,2,3,10,12\n");

	if (pthread_mutex_init(&mutex, NULL) != 0) 
	{
	    printf("\n mutex init failed\n");
	    return 1;
	}

	ret1 = pthread_create(&polling_thread, NULL, &polling_function, NULL);
	if (ret1 != 0)
	      printf("\ncan't create polling thread\n");
	usleep (3000);

	pthread_join (polling_thread, NULL);
	pthread_mutex_destroy(&mutex);

	return 0;
}
