/* 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/syscalls.h>
#include <linux/miscdevice.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

/*----------------------------------------------------------------------------*/
#define SENSORS_STATE_IOCTL_TYPE		('s')
#define SENSORS_STATE_IOCTL_MARKET_AREA		_IOR(SENSORS_STATE_IOCTL_TYPE, 0, int)
#define SENSORS_STATE_IOCTL_PSENSOR_EXISTS	_IOR(SENSORS_STATE_IOCTL_TYPE, 1, int)
#define SENSORS_STATE_IOCTL_LSENSOR_EXISTS	_IOR(SENSORS_STATE_IOCTL_TYPE, 2, int)
#define SENSORS_STATE_IOCTL_GYRO_EXISTS		_IOR(SENSORS_STATE_IOCTL_TYPE, 3, int)
#define SENSORS_STATE_IOCTL_ASENSOR_EXISTS	_IOR(SENSORS_STATE_IOCTL_TYPE, 4, int)
#define SENSORS_STATE_IOCTL_MSENSOR_EXISTS	_IOR(SENSORS_STATE_IOCTL_TYPE, 5, int)
#define SENSORS_STATE_IOCTL_OTG_EXISTS		_IOR(SENSORS_STATE_IOCTL_TYPE, 6, int)
#define SENSORS_STATE_IOCTL_HALL_EXISTS		_IOR(SENSORS_STATE_IOCTL_TYPE, 7, int)

/*----------------------------------------------------------------------------*/
char* Market_Area;
int Proximity_sensor;
int Light_sensor;
int Gyroscope_sensor;
int Acceleration_sensor;
int Magnetic_sensor;
int OTG_open;
int Hall;

core_param(Market_Area, Market_Area, charp, 0444);
core_param(Proximity_sensor, Proximity_sensor, int, 0444);
core_param(Light_sensor, Light_sensor, int, 0444);
core_param(Gyroscope_sensor, Gyroscope_sensor, int, 0444);
core_param(Acceleration_sensor, Acceleration_sensor, int, 0444);
core_param(Magnetic_sensor, Magnetic_sensor, int, 0444);
core_param(OTG_open, OTG_open, int, 0444);
core_param(Hall, Hall, int, 0444);

/*----------------------------------------------------------------------------*/
static int sensors_state_open(struct inode *inode, struct file *filp);
static int sensors_state_release(struct inode *inode, struct file *filp);
static long sensors_state_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static const struct file_operations sensors_state_fops = {
	.owner = THIS_MODULE,
	.open = sensors_state_open,
	.release = sensors_state_release,
	.unlocked_ioctl = sensors_state_ioctl,
};

static struct miscdevice md = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "sensors_state",
	.fops	= &sensors_state_fops,
};

/*----------------------------------------------------------------------------*/
static int sensors_state_open(struct inode *inode, struct file *filp)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static int sensors_state_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static long sensors_state_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//int ret;

	switch (cmd) {
	case SENSORS_STATE_IOCTL_MARKET_AREA:
		if (Market_Area) {
			strncpy((char *)arg, Market_Area, strlen(Market_Area)); 
		}
		break;
	case SENSORS_STATE_IOCTL_PSENSOR_EXISTS:
		*(unsigned long*)arg = Proximity_sensor;
		break;
	case SENSORS_STATE_IOCTL_LSENSOR_EXISTS:
		*(unsigned long*)arg = Light_sensor;
		break;
	case SENSORS_STATE_IOCTL_GYRO_EXISTS:
		*(unsigned long*)arg = Gyroscope_sensor;
		break;
	case SENSORS_STATE_IOCTL_ASENSOR_EXISTS:
		*(unsigned long*)arg = Acceleration_sensor;
		break;
	case SENSORS_STATE_IOCTL_MSENSOR_EXISTS:
		*(unsigned long*)arg = Magnetic_sensor;
		break;
	case SENSORS_STATE_IOCTL_OTG_EXISTS:
		*(unsigned long*)arg = OTG_open;
		break;
	case SENSORS_STATE_IOCTL_HALL_EXISTS:
		*(unsigned long*)arg = Hall;
		break;
	default:
		break;
	}

	return 0; 
}

/*----------------------------------------------------------------------------*/
static int __init sensors_state_init(void)
{
	int ret;

	printk("%s\n", __func__);
	ret = misc_register(&md);
	if (ret) {
		printk("Failed to register sensors state device\n");
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit sensors_state_exit(void)
{
	int ret;

	printk("%s\n", __func__);
	ret = misc_deregister(&md);
	if (ret) {
		printk("Failed to deregister sensors state device\n");
	}
}

/*----------------------------------------------------------------------------*/
module_init(sensors_state_init);
module_exit(sensors_state_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sensors state dummy driver");
MODULE_AUTHOR("Tinno Mobile");


