/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Ilias Ntontoros >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
static int __attribute__((unused)) lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *);
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;

	WARN_ON(!(sensor = state->sensor));
	if (sensor->msr_data[state->type]->last_update > state->buf_timestamp)
		return 1;
	else
		return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	unsigned long flags;
	uint32_t data;
	unsigned int dec, fract;
	unsigned char sign;
	long values;
	long *lookup[N_LUNIX_MSR] = {lookup_voltage, lookup_temperature, lookup_light};

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	sensor = state->sensor;
	spin_lock_irqsave(&sensor->lock, flags);

	/*
	 * Any new data available?
	 */

	if (!lunix_chrdev_state_needs_refresh(state))
	{
		spin_unlock_irqrestore(&sensor->lock, flags);
		return -EAGAIN;
	}
	data = sensor->msr_data[state->type]->values[0];
	state->buf_timestamp = sensor->msr_data[state->type]->last_update;

	spin_unlock_irqrestore(&sensor->lock, flags);
	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	values = lookup[state->type][data];
	if (values >= 0)
		sign = ' ';
	else
		sign = '-';

	dec = values / 1000;
	fract = values % 1000;
	sprintf(state->buf_data, " %c%d.%d  ", sign, dec, fract);

	state->buf_lim = strnlen(state->buf_data, 20);

	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	int ret;
	struct lunix_chrdev_state_struct *state;
	dev_t nodeminor;
	int type;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;
	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */

	nodeminor = iminor(inode);
	type = nodeminor % 8;
	if (type >= 4)
	{
		ret = -ENODEV;
		goto out;
	}

	/* Allocate a new Lunix character device private state structure */
	state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if (!state)
	{
		ret = -EFAULT;
		goto out;
	}

	if (type == 0)
		state->type = BATT;
	if (type == 1)
		state->type = TEMP;
	if (type == 2)
		state->type = LIGHT;

	state->buf_lim = 0;
	state->buf_timestamp = 0;
	state->sensor = &lunix_sensors[(nodeminor >> 3)];

	sema_init(&state->lock, 1);
	filp->private_data = state;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);

	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	if (down_interruptible(&state->lock))
		ret = -ERESTARTSYS;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos >= (state->buf_lim))
		*f_pos = 0;

	if (*f_pos == 0)
	{
		while (lunix_chrdev_state_update(state) == -EAGAIN)
		{

			/* The process needs to sleep */
			up(&state->lock);

			if (filp->f_flags & O_NONBLOCK)
			{
				ret = -EAGAIN;
				goto out;
			}

			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
			{
				ret = -ERESTARTSYS;
				goto out;
			}

			if (down_interruptible(&state->lock))
			{
				ret = -ERESTARTSYS;
				goto out;
			}
		}
	}

	/* End of file */

	/* Determine the number of cached bytes to copy to userspace */

	if (state->buf_lim < *f_pos + cnt)
		cnt = state->buf_lim - *f_pos;

	if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt))
	{
		ret = -EFAULT;
		goto out;
	}

	*f_pos += cnt;
	ret = cnt;
	/* Auto-rewind on EOF mode? */

out:
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
	{
		.owner = THIS_MODULE,
		.open = lunix_chrdev_open,
		.release = lunix_chrdev_release,
		.read = lunix_chrdev_read,
		.unlocked_ioctl = lunix_chrdev_ioctl,
		.mmap = lunix_chrdev_mmap};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);

	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "Lunix");

	if (ret < 0)
	{
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}

	ret = cdev_add(&lunix_chrdev_cdev, dev_no, 123);
	if (ret < 0)
	{
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
