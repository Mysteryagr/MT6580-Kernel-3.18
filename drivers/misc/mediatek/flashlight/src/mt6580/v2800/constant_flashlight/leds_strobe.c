#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;






static struct work_struct workTimeOut;

//#define GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN
/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

static int g_bLtVersion;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);


#ifdef CONFIG_OF
static const struct of_device_id Flashlight_use_gpio_of_match[] = {
	{.compatible = "mediatek,strobe_gpio_main"},
	{},
};
#endif
struct pinctrl *flashlightpinctrl = NULL;

struct pinctrl_state *flashlight_mode_h = NULL;
struct pinctrl_state *flashlight_mode_l = NULL;
struct pinctrl_state *flashlight_en_h = NULL;
struct pinctrl_state *flashlight_en_l = NULL;

static int Flashlight_use_gpio_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct task_struct *keyEvent_thread = NULL;
	PK_DBG("Flashlight_use_gpio_probe\n");


	
		flashlightpinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(flashlightpinctrl)) {
				PK_DBG("IS_ERR(flashlightpinctrl) \n");
		return -1;	
		}
		flashlight_mode_l= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_cfg0");
		if (IS_ERR(flashlight_mode_l)) {
			PK_DBG("IS_ERR(flashlight_mode_l) \n");
		return -1;	 
		}

	   flashlight_mode_h = pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_cfg1");
	   if (IS_ERR(flashlight_mode_h)) {
	  	PK_DBG("IS_ERR(flashlight_mode_h) \n");
	   return -1;	
	   }
	   flashlight_en_l= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_en0");
	   if (IS_ERR(flashlight_en_l)) {
	   	PK_DBG("IS_ERR(flashlight_en_l) \n");
	   return -1;	
	   }
	   flashlight_en_h= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_en1");
	   if (IS_ERR(flashlight_en_h)) {
	   	PK_DBG("IS_ERR(flashlight_en_h) \n");
	   return -1;	
	   }

    return 0;
}

static int Flashlight_use_gpio_remove(struct platform_device *dev)	
{
	return 0;
}

static struct platform_driver Flashlight_use_gpio_driver = {
	.probe	= Flashlight_use_gpio_probe,
	.remove  = Flashlight_use_gpio_remove,
	.driver    = {
	.name       = "flashlight",
	.of_match_table = Flashlight_use_gpio_of_match,	
	},
};

static int __init Flashlight_use_gpio_init(void)
{
	PK_DBG("Flashlight_use_gpio_init\n");
	//return i2c_add_driver(&Flashlight_use_gpio_i2c_driver);
	    platform_driver_register(&Flashlight_use_gpio_driver);
}

static void __exit Flashlight_use_gpio_exit(void)
{
	//i2c_del_driver(&Flashlight_use_gpio_i2c_driver);
	    platform_driver_unregister(&Flashlight_use_gpio_driver);
}


module_init(Flashlight_use_gpio_init);
module_exit(Flashlight_use_gpio_exit);

MODULE_DESCRIPTION("Flash driver for GPIO flashlight");
MODULE_AUTHOR("jack <jack.kang@tinno.com>");
MODULE_LICENSE("GPL v2");


int FL_Enable(void)
{
	if(g_duty==0)
	{
		
	pinctrl_select_state(flashlightpinctrl, flashlight_en_h);
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_l);
	
	//	mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
	//	mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
	}
	else
	{
		pinctrl_select_state(flashlightpinctrl, flashlight_en_h);
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_h);
	//	mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
	//	mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);

	}


	return 0;
}



int FL_Disable(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_l);

//	mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
//	mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}




int FL_Init(void)
{
#if 1
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
#else

    if(mt_set_gpio_mode(GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    /*Init. to disable*/
    if(mt_set_gpio_mode(GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

	#endif
	    PK_DBG(" FL_Init line=%d\n",__LINE__);
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("Flashlight_use_gpio constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
