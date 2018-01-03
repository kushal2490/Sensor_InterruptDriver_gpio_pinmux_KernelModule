Part 1 - Device Driver for HC-SR04 sensor
==================================================================================================================

kernelmain.c	-> user program to excerise on 2 device files performing R/W operations
				with separate samples set and delta for periodic delay .
				User input is the pin numbers for two devices

sensor_driver.c ->	Implements sensor function using periodic measurement with the help of hrtimer.
	It has 2 ioctl functions:
	CONFIG_PINS: to configure pins trigger and echo using the IO pin numbers.
	SET_PARAMETERS: to set the number of sample per measurement and sampling period.

	READ function:- Reads the device buffer containing most recent 5 measures.
	WRITE function:- Triggers a new measurement if no ongoing measurement, clears existing buffer if non-zero
					integer is passed as int argument else appends new data to buffer if int argument is 0.
					Returns EINVAL if ongoing measurement.

	INIT:- Initializes "argNum" number of devices.



==================================================================================================
NOTE:
1) Reboot

2) Export your SDK's toolchain path to your current directory to avoid unrecognized command.
export PATH="/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux/:$PATH"

3) Connect IO pins before starting the program.
=================================================================================================


Method:-

1) Change the following KDIR path to your source kernel path and the SROOT path to your sysroots path in the Makefile
KDIR:=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/usr/src/kernel
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/

2) Run make command to compile both user app and the sensor_driver kernel object.
make

3) copy 2 files from current dir to board dir using the below command:
	i) 	sensor_driver.ko
	ii)	kernelmain

sudo scp <filename> root@<inet_address>:/home/root

4) insmod sensor_driver.ko argNum=2

5) Run the user app 
./kernelmain

6) User asks input
Enter Trigger and Echo from the displayed ranges
for eg)

	Trigger Pin from range 0-19
	Echo Pin from 0,1,2,3,10,12
	Enter 1)Trigger and 2)Echo pin for Device 1

	$:>2 10

	Enter 1)Trigger and 2)Echo pin for Device 2

	$:>1 12