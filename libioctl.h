#include <linux/ioctl.h>

typedef struct ioctl_data
{
	int arg1;
	int arg2;
}ioctl_data_t;

#define IOC_MAGIC 'k'
#define CONFIG_PINS _IOW(IOC_MAGIC, 0, ioctl_data_t)
#define SET_PARAMETERS _IOW(IOC_MAGIC, 1, ioctl_data_t)
