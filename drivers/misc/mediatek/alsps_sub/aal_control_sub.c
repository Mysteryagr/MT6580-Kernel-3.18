#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <hwmsen_dev.h>
#include <sensors_io.h>
#include "inc/alsps_sub.h"
#include "inc/aal_control_sub.h"


int aal_use_sub = 0;

static int AAL_sub_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int AAL_sub_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long AAL_sub_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	void __user *ptr = (void __user *) arg;
	int dat;
	uint32_t enable;

	switch (cmd) {

	case AAL_SUB_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}

		if (enable)
			aal_use_sub = 1;
		else
			aal_use_sub = 0;

		err = alsps_sub_aal_enable(enable);
		if (err) {
			AAL_SUB_LOG("als driver don't support new arch, goto execute old arch: %ld\n", err);
			err = hwmsen_aal_sub_enable(enable);
			if (err != 0)
				AAL_SUB_ERR("Enable als driver fail %ld\n", err);
		}
		break;

	case AAL_SUB_GET_ALS_MODE:
		AAL_SUB_LOG("AAL_GET_ALS_MODE do nothing\n");
		break;

	case AAL_SUB_GET_ALS_DATA:
		dat = alsps_sub_aal_get_data();
		if (dat < 0) {
			AAL_SUB_LOG("alsps_aal_get_data fail\n");
			dat = hwmsen_aal_sub_get_data();
		}
		AAL_SUB_LOG("Get als dat :%d\n", dat);

		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	default:
		AAL_SUB_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

err_out:
		return err;
}

#ifdef CONFIG_COMPAT
static long AAL_sub_compact_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *data32;

	data32 = compat_ptr(arg);
	return AAL_sub_unlocked_ioctl(file, cmd, (unsigned long)data32);
}
#endif

static const struct file_operations AAL_sub_fops = {
	.owner = THIS_MODULE,
	.open = AAL_sub_open,
	.release = AAL_sub_release,
	.unlocked_ioctl = AAL_sub_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = AAL_sub_compact_ioctl,
#endif
};

static struct miscdevice AAL_sub_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aal_als_sub",
	.fops = &AAL_sub_fops,
};


/*----------------------------------------------------------------------------*/
static int __init AAL_sub_init(void)
{
	int err;

	err = misc_register(&AAL_sub_device);
	if (err)
		AAL_SUB_ERR("AAL_sub_device misc_register failed: %d\n", err);

	AAL_SUB_LOG("OK!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit AAL_sub_exit(void)
{
	int err;

	err = misc_deregister(&AAL_sub_device);
	if (err)
		AAL_SUB_ERR("AAL_sub_device misc_deregister fail: %d\n", err);
}

late_initcall(AAL_sub_init);
MODULE_AUTHOR("Mediatek");
MODULE_DESCRIPTION("AAL sub driver");
MODULE_LICENSE("GPL");


