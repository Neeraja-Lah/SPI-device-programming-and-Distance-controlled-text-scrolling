#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include<linux/init.h>
#include<linux/moduleparam.h>

#include "hcsr_ioctl.h"

// HC-SR04 /dev Device Name
#define DEVICE_NAME                     "hcsr04_dev"

static dev_t hcsr04_dev_number;
struct class *hcsr04_dev_class;

// Per Device Structure for HCSR-04 Device
typedef struct {
    char name[20];
	char busy_flag;
	unsigned long distance;
	struct gpio_desc *gpiod_trig;
	struct gpio_desc *gpiod_echo;
	unsigned int irq_num;
	struct mutex mutex;
    struct cdev cdev;
} hcsr04_dev_t;

hcsr04_dev_t *hcsr04_dev;

// Flag to check the ECHO Input Pulse
int measure_flag;

// Kernel Time variables to measure the pulse time
ktime_t begin, end;

// Callback function whenever an interrupt arrives
static irqreturn_t irq_handler(int irq, void *dev_id)
{
	hcsr04_dev_t *hcsr04_devp = (hcsr04_dev_t *)dev_id;

	// Lock the device mutex before measuring
	mutex_lock(&hcsr04_devp->mutex);

	// Check whether device is busy or not
	if(hcsr04_devp->busy_flag) {
		// Check for rising or falling edge to either begin or end the distance calculation
		if(!measure_flag) {
			begin = ktime_get();
			measure_flag = 1;
		}
		else {
			end = ktime_get();
			hcsr04_devp->distance = (unsigned long)(end - begin);
			// Scale the input pulse to make the text scrollig bounded by a specific speed
			hcsr04_devp->distance /= 10;
			if(hcsr04_devp->distance < 10000) {
				hcsr04_devp->distance = 10000;
			}
			else if(hcsr04_devp->distance > 1000000) {
				hcsr04_devp->distance = 1000000;
			}
			measure_flag = 0;
			hcsr04_devp->busy_flag = 0;
		}
	}

	// Unlock the mutex after measurement
	mutex_unlock(&hcsr04_devp->mutex);

	return IRQ_HANDLED;
}

static int hcsr04_drv_open(struct inode *inode, struct file *file)
{
	hcsr04_dev_t *hcsr04_devp;

	hcsr04_devp = container_of(inode->i_cdev, hcsr04_dev_t, cdev);

	hcsr04_devp->busy_flag = 0;

	file->private_data = hcsr04_devp;
	
	printk(KERN_DEBUG "Device Opened: %s\n", hcsr04_devp->name);

	return 0;
}

static int hcsr04_drv_release(struct inode *inode, struct file *file)
{
	hcsr04_dev_t *hcsr04_devp = file->private_data;

	// Free the assigned GPIOs and IRQs before closing
	if(hcsr04_devp->gpiod_trig) {
		gpiod_put(hcsr04_devp->gpiod_trig);
	}

	if(hcsr04_devp->gpiod_echo) {
		gpiod_put(hcsr04_devp->gpiod_echo);
	}
	
	free_irq(hcsr04_devp->irq_num, hcsr04_devp);
	
	printk(KERN_DEBUG "Device Closed: %s\n", hcsr04_devp->name);

	return 0;
}

ssize_t hcsr04_drv_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
	hcsr04_dev_t *hcsr04_devp = file->private_data;

	// Copy the saved distance to the distance buffer in user space
	if(copy_to_user((unsigned long *)buf, &hcsr04_devp->distance, sizeof(unsigned long))) {
		printk("Cannot copy to user\n");
		return -1;
	}
	
	return 0;
}

ssize_t hcsr04_drv_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{
	hcsr04_dev_t *hcsr04_devp = file->private_data;

	// Check whether device is busy of not
	if(!hcsr04_devp->busy_flag) {
		// Lock the mutex before sending Trigger Pulse
		mutex_lock(&hcsr04_devp->mutex);
		
		// Send the HIGH-LOW pulse with a delay to 10 us to begin the measurement
		gpiod_set_value(hcsr04_devp->gpiod_trig, 1);
		udelay(10);
		gpiod_set_value(hcsr04_devp->gpiod_trig, 0);
		hcsr04_devp->busy_flag = 1;

		// Release the mutex lock
		mutex_unlock(&hcsr04_devp->mutex);
	}
	else {
		printk("Device Busy\n");
	}
	
	return count;
}

static long hcsr04_drv_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	hcsr04_dev_t *hcsr04_devp = file->private_data;

	unsigned int pin = (unsigned int)arg;

	switch(cmd)
	{
		case SET_TRIGGER:
			hcsr04_devp->gpiod_trig = gpio_to_desc(pin);
			if(!hcsr04_devp->gpiod_trig) {
				printk(KERN_DEBUG "Invalid GPIO %d\n", pin);
				return -1;
			}

			ret = gpiod_direction_output(hcsr04_devp->gpiod_trig, 0);
			if(ret) {
				printk(KERN_DEBUG "Error %d setting direction for GPIO %d\n", ret, pin);
				return -1;
			}
			break;

		case SET_ECHO:
			hcsr04_devp->gpiod_echo = gpio_to_desc(pin);
			if(!hcsr04_devp->gpiod_echo) {
				printk(KERN_DEBUG "Invalid GPIO %d\n", pin);
				return -1;
			}

			ret = gpiod_direction_input(hcsr04_devp->gpiod_echo);
			if(ret) {
				printk(KERN_DEBUG "Error %d setting direction for GPIO %d\n", ret, pin);
				return -1;
			}

			hcsr04_devp->irq_num = gpiod_to_irq(hcsr04_devp->gpiod_echo);
			if(hcsr04_devp->irq_num < 0) {
				printk(KERN_DEBUG "Cannot convert GPIO to IRQ\n");
				return -1;
			}

			ret = request_irq(hcsr04_devp->irq_num, irq_handler, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING), "Distance IRQ", hcsr04_devp);
			break;

		default:
			printk(KERN_DEBUG "Invalid Command\n");
			return -1;
	}

	return 0;
}

struct file_operations hcsr04_fops = {
	.owner = THIS_MODULE,
	.open = hcsr04_drv_open,
	.release = hcsr04_drv_release,
	.read = hcsr04_drv_read,
	.write = hcsr04_drv_write,
	.unlocked_ioctl = hcsr04_drv_ioctl,
};

/*
 * Driver Initialization
 */
int __init hcsr04_drv_init(void)
{
    int ret;

	/* Request dynamic allocation of a device major number */
    if(alloc_chrdev_region(&hcsr04_dev_number, 0, 1, DEVICE_NAME)) {
        printk(KERN_DEBUG "Can't register device\n"); return -1;
    }

	/* Allocate memory for the per-device structure */
	hcsr04_dev = kmalloc(sizeof(hcsr04_dev_t), GFP_KERNEL);
	if (!hcsr04_dev) {
		printk("Bad Kmalloc\n"); return -ENOMEM;
	}

	/* Populate sysfs entries */
	hcsr04_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

	/* Request I/O region */
	sprintf(hcsr04_dev->name, DEVICE_NAME);

	/* Connect the file operations with the cdev */
    cdev_init(&hcsr04_dev->cdev, &hcsr04_fops);
    hcsr04_dev->cdev.owner = THIS_MODULE;

	/* Connect the major/minor number to the cdev */
    ret = cdev_add(&hcsr04_dev->cdev, MKDEV(MAJOR(hcsr04_dev_number), 0), 1);
    if (ret) {
        printk("Bad cdev\n");
        return ret;
    }

	/* Send uevents to udev, so it'll create /dev nodes */
    device_create(hcsr04_dev_class, NULL, MKDEV(MAJOR(hcsr04_dev_number), 0), NULL, hcsr04_dev->name);

	mutex_init(&hcsr04_dev->mutex);

    printk("Device %s Initialized\n", hcsr04_dev->name);

    return ret;
}

void __exit hcsr04_drv_exit(void)
{
	/* Release the major number */
	unregister_chrdev_region(hcsr04_dev_number, 1);

	/* Destroy device */
	device_destroy(hcsr04_dev_class, MKDEV(MAJOR(hcsr04_dev_number), 0));
	cdev_del(&hcsr04_dev->cdev);
	kfree(hcsr04_dev);
	
	/* Destroy driver_class */
	class_destroy(hcsr04_dev_class);

	printk("Device %s Removed\n", hcsr04_dev->name);
}

module_init(hcsr04_drv_init);
module_exit(hcsr04_drv_exit);
