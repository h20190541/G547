#include<linux/init.h>
#include<linux/module.h>
#include<linux/version.h>
#include<linux/kernel.h>
#include<linux/types.h>
#include<linux/kdev_t.h>
#include<linux/cdev.h>
#include<linux/fs.h>
#include<linux/device.h>
#include <linux/types.h>
#include <linux/random.h>
#include<linux/errno.h>
#include <linux/uaccess.h>
//#include "chardev.h"

#define MAGIC_NUM 'Z'

#define CHANNEL 1
#define ALLIGNMENT 2

#define SET_CHANNEL _IOW(MAGIC_NUM, CHANNEL, unsigned int)
#define SET_ALLIGNMENT _IOW(MAGIC_NUM, ALLIGNMENT, unsigned int)

static dev_t first; // variable for device number
static struct cdev adc; // variable for the adc
static struct class *cls; // variable for the device class

uint16_t value;
uint16_t channel;
uint16_t allignment;


static int adc_open(struct inode *i, struct file *f)
{
	printk(KERN_INFO "open()\n");
	return 0;
}

static int adc_close(struct inode *i, struct file *f)
{
	printk(KERN_INFO "close()\n");
	return 0;
}

static ssize_t adc_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	printk(KERN_INFO "read()\n");
	get_random_bytes(&value, 2);
	value=value%1024; // Resolution is 10 bit so max value can be 1023
	
	/*FILE* urandom = fopen("/dev/urandom", "r");
	fread(&seed, sizeof(int), 1, urandom);
	fclose(urandom);
	adc_val=srand(seed);*/

	printk(KERN_INFO "%d\n",value);
	printk(KERN_INFO "ADC Value: %d", value);
	printk(KERN_INFO "Channel : %d \n",channel);
	printk(KERN_INFO "Allignment : %d \n",allignment);
	if(allignment == 1)
		value = value << 6;
	copy_to_user(buf, &value, 2);
	return 0;
}

static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	switch(cmd){

		case SET_CHANNEL:
		copy_from_user(&channel, (int32_t *) arg, sizeof(channel));
		break;
		case SET_ALLIGNMENT:
		copy_from_user(&allignment, (int32_t *) arg, sizeof(allignment));
		break;
		

		default:
		return -ENOTTY;

	}
	return 0;
}




static struct file_operations fops =
{
  .owner 	= THIS_MODULE,
  .open 	= adc_open,
  .release 	= adc_close,
  .read 	= adc_read,
  .unlocked_ioctl= my_ioctl,
};
 

static int __init mychar_init(void) 
{
	printk(KERN_INFO "Namaste: ADC registered");
	
	// STEP 1 : reserve <major, minor>
	if (alloc_chrdev_region(&first, 0, 1, "BITS-Pilani") < 0)
	{
		return -1;
	}
	
	// STEP 2 : dynamically create device node in /dev directory
    if ((cls = class_create(THIS_MODULE, "chardrv")) == NULL)
	{
		unregister_chrdev_region(first, 1);
		return -1;
	}
    if (device_create(cls, NULL, first, NULL, "adc8") == NULL)
	{
		class_destroy(cls);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	
	// STEP 3 : Link fops and cdev to device node
    cdev_init(&adc, &fops);
    if (cdev_add(&adc, first, 1) == -1)
	{
		device_destroy(cls, first);
		class_destroy(cls);
		unregister_chrdev_region(first, 1);
		return -1;
	}
	return 0;
}
 
static void __exit mychar_exit(void) 
{
	cdev_del(&adc);
	device_destroy(cls, first);
	class_destroy(cls);
	unregister_chrdev_region(first, 1);
	printk(KERN_INFO "Bye: Adc unregistered\n\n");
}
 
module_init(mychar_init);
module_exit(mychar_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vikas Gupta");
MODULE_DESCRIPTION("Driver for ADC");
