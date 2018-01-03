#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/param.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>	
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/math64.h>
#include <linux/unistd.h>
#include <linux/sched.h>

#include "buffer.h"
#include "libioctl.h"
#include "gpioconfig.h"
///////////////////////////////////////////////////////////////
//#define MAXRUNS 5

int argNum;

module_param(argNum, int, S_IRUGO|S_IWUSR);

int distance_int;
int edge_flag = 1;
int ready;
int min = 10000000, max = 0, sum;
int MAXRUNS;
int NUM_HCSR_DEVICES;
static int local_count = 0;
static unsigned long period_ns;
unsigned long flags;
uint64_t start, finish;
uint64_t stamp, sensor_cycles;
long cycles;

unsigned int irq_line;
ktime_t ktime;


struct hcsr_dev **hcsr_devpArr;

struct hcsr_drv{
	struct hcsr_dev *head;
}*hcsr_drvp;

struct hcsr_dev{
	struct miscdevice sensor_dev;
	char name[20];		/* Name of Device */
	/* Trigger and Echo pin numbers */
	int trig_pin;
	int echo_pin;
	int IOpinTrig;
	int IOpinEcho;
	int done;
	int mydown;
	/* FIFO pointers & bufsize elements present */
	int head, tail;
	int bufsize;
	/* m samples, delta periodic delay */
	int m;
	unsigned long delta;
	unsigned long flags;
	int runcount;
	spinlock_t lock_kprobe;
	struct semaphore devlock;
	bufp fifo[FIFOSIZE];
	struct hrtimer my_hrtimer;
	struct hcsr_dev *next;
};


/* time stamp counter to retrieve the CPU cycles */
uint64_t tsc(void)
{
	uint32_t a, d;
	asm volatile("rdtsc" : "=a" (a), "=d" (d));
	return (( (uint64_t)a)|( (uint64_t)d)<<32 );
}

/* interrupt handler to detect the rising and falling edge on the GPIO ECHO pin */
static irqreturn_t sensor_interrupt_handler (int irq, void *dev_id)
{
	struct hcsr_dev* hcsr_devp = (struct hcsr_dev *)dev_id;
	int avg_distance = 0;
	long my_distance;

		if (gpio_get_value(hcsr_devp->echo_pin))
		{	
			//Detect the Rising edge, note the timestamp value and set the IRQ Line to detect the FALLING edge
			start = tsc();
			irq_set_irq_type(irq, IRQF_TRIGGER_FALLING);		
		}
		else
		{
			//Detect the FALLING edge, note the timestamp value and set the IRQ Line to detect the RISING edge
 
			finish = tsc();
			sensor_cycles = finish - start;
			
			spin_lock_irqsave(&hcsr_devp->lock_kprobe,flags);

			cycles = (long)sensor_cycles;
			my_distance = (cycles*340)/80000;


			if(hcsr_devp->runcount < (MAXRUNS - 1) && hcsr_devp->runcount >= 0)
			{
				printk(KERN_INFO "distance = %ld\n", my_distance/100);
				if(cycles<min)
					min = cycles;
				if(cycles>max)
					max = cycles;
				sum += cycles;
				hcsr_devp->runcount++;
			}
			if(hcsr_devp->runcount == (MAXRUNS -1))
			{
				printk(KERN_INFO "distance = %ld\n", my_distance/100);
				if(cycles<min)
					min = cycles;
				if(cycles>max)
					max = cycles;
				sum += cycles;

				sum = sum - max - min;
  				avg_distance = sum/(MAXRUNS - 2);
  				avg_distance = (avg_distance*340)/80000;

				hcsr_devp->fifo[hcsr_devp->head].distance = avg_distance/100;
				hcsr_devp->fifo[hcsr_devp->head].timestamp = tsc();
				printk("Storing Buffer at head:%d dist = %ld tsc = %lu\n", hcsr_devp->head, hcsr_devp->fifo[hcsr_devp->head].distance, (unsigned long)hcsr_devp->fifo[hcsr_devp->head].timestamp);
				hcsr_devp->head = (hcsr_devp->head + 1) % FIFOSIZE;
				hcsr_devp->bufsize = ((hcsr_devp->bufsize+1) > FIFOSIZE) ? FIFOSIZE:(hcsr_devp->bufsize+1);
				if(hcsr_devp->head == hcsr_devp->tail)
					hcsr_devp->tail = (hcsr_devp->tail + 1) % FIFOSIZE;
				
				hcsr_devp->runcount = -1;
				sum = 0;
				min = 100000000;
				max = 0;
				hcsr_devp->done =1;
			}
			spin_unlock_irqrestore(&hcsr_devp->lock_kprobe,flags);
			irq_set_irq_type(irq, IRQF_TRIGGER_RISING);
		}							
	return IRQ_HANDLED;
}

/* sensor device open function */
int sensor_open(struct inode *inode, struct file *file)
{

	const struct file_operations *fops = NULL;
	struct hcsr_dev *hcsr_devp = hcsr_drvp->head;
	int minor = iminor(inode);

	while(hcsr_devp->next != NULL)
	{
		if(hcsr_devp->sensor_dev.minor == minor)
		{
			fops = fops_get(hcsr_devp->sensor_dev.fops);
			break;
		}
		hcsr_devp = hcsr_devp->next;
	}
	
	/* Easy access to devp from rest of the entry points */
	file->private_data = hcsr_devp;
	printk(KERN_INFO "Opening device\n");

	return 0;
}

/* sensor device release function */
int sensor_release(struct inode *inode, struct file *file)
{
	struct hcsr_dev *hcsr_devp = file->private_data;

	/* Releasing gpio pins */
	gpio_config_pin(hcsr_devp->IOpinTrig, -1, 1);
	gpio_config_pin(hcsr_devp->IOpinEcho, -1, 1);

	free_irq(gpio_to_irq(hcsr_devp->echo_pin), (void *)hcsr_devp);

	printk(KERN_INFO "\nDevice is closing\n");

	return 0;
}

static enum hrtimer_restart myfunc(struct hrtimer *timer)
{
	
	int ret_overrun;
	ktime_t currtime , interval;
	struct hcsr_dev *hcsr_devp;
	/* Get the per-device structure that contains this cdev */
	hcsr_devp = container_of(timer, struct hcsr_dev, my_hrtimer);

	currtime  = ktime_get();

	//Generate TRIGGER PULSE
	gpio_set_value_cansleep(hcsr_devp->trig_pin, 0);
	udelay(100);
	gpio_set_value_cansleep(hcsr_devp->trig_pin, 1);
	udelay(116);
	gpio_set_value_cansleep(hcsr_devp->trig_pin, 0);
 

	if (local_count < (MAXRUNS)) {

		interval = ktime_set(0,period_ns); 
	  	ret_overrun = hrtimer_forward(timer, currtime , interval);
		local_count++;
		
		return HRTIMER_RESTART;
  	}

  else{
		local_count = 0;
		return HRTIMER_NORESTART;
  	}
}

/* sensor device read function which is used to copy the distance measurement to the user space */
ssize_t sensor_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct hcsr_dev *hcsr_devp = file->private_data;
	int ret = 0;
	
		// printk(KERN_INFO "Read for device: %s\n", hcsr_devp->name);

	if(hcsr_devp->bufsize==0)
	{
		if(hrtimer_active(&hcsr_devp->my_hrtimer))
		{			
			//wait and copy to user
			
			while(!hcsr_devp->bufsize)
			{
				// printk(KERN_INFO "Timer active, waiting for buffer data1\n");

			}		
			
			ret = copy_to_user(buf, &hcsr_devp->fifo, sizeof(bufp)*FIFOSIZE);
			// printk(KERN_INFO "Copied buffer. Buff size = %d\n", hcsr_devp->bufsize);
		}
		else
		{
			// printk(KERN_INFO "Timer not active\n");

			//trigger new measurement
			hcsr_devp->done = 0;
		  	local_count = 0;
			hcsr_devp->runcount = 0;
			MAXRUNS = hcsr_devp->m + 2;
			period_ns = 1000000*hcsr_devp->delta;
			ktime = ktime_set( 0, period_ns );
			hrtimer_start((&hcsr_devp->my_hrtimer), ktime, HRTIMER_MODE_REL);

			printk(KERN_INFO "Triggered new measurement\n");

			//wait and copy to user
			while(!hcsr_devp->bufsize && hrtimer_active(&hcsr_devp->my_hrtimer))
			{
				// printk(KERN_INFO "Timer active, waiting for buffer data2\n");
				
			}
			
			ret = copy_to_user(buf, &hcsr_devp->fifo, sizeof(bufp)*FIFOSIZE);
			// printk(KERN_INFO "Copied buffer to user: Buff size = %d\n", hcsr_devp->bufsize);
		}
	}

	else //if buffsize!=0
	{
		//After we have detected both the RISING and FALLING edges
		while(hrtimer_active(&hcsr_devp->my_hrtimer) || (hcsr_devp->done == 0) )
		{
			// printk(KERN_INFO "Timer 2 active, waiting for buffer data\n");

		}

		ret = copy_to_user(buf, &hcsr_devp->fifo, sizeof(bufp)*FIFOSIZE);
		printk(KERN_INFO "Copied buffer. Buff size = %d\n", hcsr_devp->bufsize);

	}
	
		return 0;
	
}

/* sensor device write function which is used to generate the TRIGGER using the GPIO pin */
ssize_t sensor_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct hcsr_dev *hcsr_devp = file->private_data;
	int clearbuf;
	int ret;

	printk(KERN_INFO "Write for device: %s\n", hcsr_devp->name);


	ret = copy_from_user(&clearbuf, (int *)buf, sizeof(int));

	if(clearbuf)
		printk(KERN_INFO "Clearing Buffer:%d\n", clearbuf);


	if(clearbuf)
	{
		//clear fifo --------------- Needs to be hcsr_devp->done
		memset(hcsr_devp->fifo, 0, FIFOSIZE*sizeof(bufp));
		hcsr_devp->head = 0;
		hcsr_devp->tail = 0;
		hcsr_devp->bufsize = 0;
		printk(KERN_INFO "\nBuffer Cleared\n");
	}

	if(!hrtimer_active((&hcsr_devp->my_hrtimer)))
	{
	    // MAXRUNS = hcsr_devp->m + 2;
		hcsr_devp->done = 0;
		local_count = 0;
		hcsr_devp->runcount = 0;

		MAXRUNS = hcsr_devp->m + 2;
		period_ns = 1000000*hcsr_devp->delta;
		ktime = ktime_set( 0, period_ns );
		printk(KERN_INFO "Write started measurement\n");
		hrtimer_start((&hcsr_devp->my_hrtimer), ktime, HRTIMER_MODE_REL);

	return 0;
	}
	else{
		printk(KERN_INFO "********Write Failed\n");
		return -EINVAL;
	}
	
}

long sensor_ioctl(struct file *file,unsigned int cmd, ioctl_data_t *ioctl_data)
{

	struct hcsr_dev *hcsr_devp = file->private_data;
	int ret, res;
	
	switch(cmd)
	{
		case CONFIG_PINS:
		{

			if((ioctl_data->arg1 >= 0 && ioctl_data->arg1 <=19) && 
				(ioctl_data->arg2>=0 &&	(ioctl_data->arg2 <=3 || ioctl_data->arg2 == 10 || ioctl_data->arg2 == 12)))
			{
				/* configuring the trigger pin */
				hcsr_devp->trig_pin = gpio_config_pin(ioctl_data->arg1, 0, 0);
				hcsr_devp->IOpinTrig = ioctl_data->arg1;
				/* configuring the echo pin */
				hcsr_devp->echo_pin = gpio_config_pin(ioctl_data->arg2, 1, 0);
				hcsr_devp->IOpinEcho = ioctl_data->arg2;
				printk("PIN SET TRIG=%d  ECHO=%d\n", hcsr_devp->trig_pin, hcsr_devp->echo_pin);
			}
			else
				return -EINVAL;

			//connect the gpio pin for ECHO to the irq line to detect rising and falling edge interrupts
			if( (irq_line = gpio_to_irq(hcsr_devp->echo_pin)) < 0 )
			{
				printk(KERN_INFO "Failed to get IRQ no for the gpio pin\n");
			}
			//Request the IRQ line 
			res = request_irq(irq_line, sensor_interrupt_handler, IRQF_TRIGGER_RISING, "gpio_change_state", (void *)hcsr_devp);
			if (res < 0)
			{
				printk(KERN_INFO "Failed to request IRQ line\n");
				if (res == -EBUSY)
					ret = res;
				else
					ret = -EINVAL;
				return ret;
			}
			else
				edge_flag = 1;


		}
		break;
		case SET_PARAMETERS:
		{
			/* Need atleast 3 samples to compute average distance and,
			a positive sampling interval to take periodic measurements */
			if(ioctl_data->arg1<=2 || ioctl_data->arg2<=0)
				return -EINVAL;

			hcsr_devp->m = ioctl_data->arg1;
			hcsr_devp->delta = ioctl_data->arg2;
			// MAXRUNS = hcsr_devp->m + 2;
			
			printk(KERN_INFO "PARAM SET to m = %d, delta = %lu\n", hcsr_devp->m, hcsr_devp->delta);
		}
		break;	
		default:
			printk("Invalid Command\n");
	}
	return 0;
}

static struct file_operations my_dev_fops = {
	.owner = THIS_MODULE, 		/* Owner */
	.open = sensor_open, 		/* Open method */
	.release = sensor_release,	/* Release method */
	.unlocked_ioctl = sensor_ioctl,
	.write = sensor_write, 		/* Write method */
	.read = sensor_read, 		/* Read method */
	
};

/* sensor device initialisation function to initialise the gpios and request an irq line for the rising edge */
int __init sensor_init(void)
{
	int i;
	char name[20];
	/* number of HCSR sensor devices to be initialized received as an argument from command line */
	NUM_HCSR_DEVICES = (int)argNum;
	
	hcsr_devpArr = kmalloc(sizeof(struct hcsr_dev *)*NUM_HCSR_DEVICES, GFP_KERNEL);

	hcsr_drvp = kmalloc(sizeof(struct hcsr_drv), GFP_KERNEL);
	if(!hcsr_drvp) {
		printk(KERN_INFO "Bad kmalloc for Driver container\n");
		return -ENOMEM;
	}

	for(i = 0; i < NUM_HCSR_DEVICES ; i++)
	{
		// printk("IM HERE %d\n", i);
		spin_lock_init(&hcsr_devpArr[i]->lock_kprobe);
		/* Allocate memory for the per-device structure */
		hcsr_devpArr[i] = kmalloc(sizeof(struct hcsr_dev), GFP_KERNEL);
		if(!hcsr_devpArr[i]) {
			printk(KERN_INFO "Bad kmalloc for Device memory\n");
			return -ENOMEM;
		}
		memset(hcsr_devpArr[i], 0, sizeof(struct hcsr_dev));
		// printk("IM HERE 2%d\n", i);
		/* Request I/O region */
		sprintf(name, "HCSR_%d", i);

		hcsr_devpArr[i]->sensor_dev.minor = MISC_DYNAMIC_MINOR;
		hcsr_devpArr[i]->sensor_dev.name = name;

		printk("Init device %s\n", hcsr_devpArr[i]->sensor_dev.name);

		hcsr_devpArr[i]->sensor_dev.fops = &my_dev_fops;
		// printk("IM HERE 3%d\n", i);
		misc_register(&hcsr_devpArr[i]->sensor_dev);
		// printk("IM HERE 4%d\n", i);
		/* Initialize the buffer and the fifo buffer operating parameters */
		memset(hcsr_devpArr[i]->fifo, 0, FIFOSIZE*sizeof(bufp));
		hcsr_devpArr[i]->head = 0;
		hcsr_devpArr[i]->tail = 0;
		hcsr_devpArr[i]->bufsize = 0;
		hcsr_devpArr[i]->done = 0;
		hcsr_devpArr[i]->mydown = 0;
		hcsr_devpArr[i]->runcount = 0;
		hcsr_devpArr[i]->IOpinTrig = 0;
		hcsr_devpArr[i]->IOpinEcho = 0;
		hrtimer_init(&(hcsr_devpArr[i]->my_hrtimer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
 	 	hcsr_devpArr[i]->my_hrtimer.function = &myfunc;

 	 	sema_init(&hcsr_devpArr[i]->devlock, 1);

	}

	// printk("IM HERE 5 %d\n", i);
	i--;

	hcsr_devpArr[i]->next = NULL;
	printk("Linked %d to NULL\n ", i);

	while(i)
	{
		printk("Linked %d to %d\n ", i-1, i);
		hcsr_devpArr[i-1]->next = hcsr_devpArr[i];
		i=i-1;
	}
	hcsr_drvp->head = hcsr_devpArr[i];
	printk("Linked head to %d\n ", i);

	printk(" Sensor Driver initialized.\n");
	
	return 0;
}

void __exit sensor_exit(void)
{
	int i;
	for(i = NUM_HCSR_DEVICES-1; i >= 0; i--)
	{
		printk("Unlinking %d : set next to NULL\n", i);
		hcsr_devpArr[i]->next = NULL;
		// kfree(hcsr_devpArr[i]->fifo);
		misc_deregister(&hcsr_devpArr[i]->sensor_dev);
		printk("Freeing Buffer %d :\n", i);
		kfree(hcsr_devpArr[i]);
	}

	hcsr_drvp->head = NULL;
	kfree(hcsr_drvp);
	
	printk("Sensor driver removed.\n");
}

module_init (sensor_init);
module_exit (sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CSE_530-TEAM_27");
MODULE_DESCRIPTION("CSE_530-Assignment2Part1");