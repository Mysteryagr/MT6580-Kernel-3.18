
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
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
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
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif



/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock_sub); /* cotta-- SMP proection */
static struct work_struct workTimeOut_sub;


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xCE


#ifdef CONFIG_OF
static const struct of_device_id Flashlight_sub_use_gpio_of_match[] = {
	{.compatible = "mediatek,strobe_gpio_sub"},
	{},
};
#endif
struct pinctrl *sub_flashlightpinctrl = NULL;

struct pinctrl_state *sub_flashlight_mode_h = NULL;
struct pinctrl_state *sub_flashlight_mode_l = NULL;
struct pinctrl_state *sub_flashlight_en_h = NULL;
struct pinctrl_state *sub_flashlight_en_l = NULL;

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteReg_led(u8 a_u2Addr , u8 a_u4Data , u8 a_u4Bytes , u16 i2cId);
extern int iReadRegI2C_led(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

static int Flashlight_sub_use_gpio_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct task_struct *keyEvent_thread = NULL;
	PK_DBG("sub_Flashlight_use_gpio_probe\n");


	
		sub_flashlightpinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(sub_flashlightpinctrl)) {
				PK_DBG("IS_ERR(sub_flashlightpinctrl) \n");
		return -1;	
		}
		sub_flashlight_mode_l= pinctrl_lookup_state(sub_flashlightpinctrl, "flashlightsubpin_cfg0");
		if (IS_ERR(sub_flashlight_mode_l)) {
			PK_DBG("IS_ERR(sub_flashlight_mode_l) \n");
		return -1;	 
		}

	   sub_flashlight_mode_h = pinctrl_lookup_state(sub_flashlightpinctrl, "flashlightsubpin_cfg1");
	   if (IS_ERR(sub_flashlight_mode_h)) {
	  	PK_DBG("IS_ERR(sub_flashlight_mode_h) \n");
	   return -1;	
	   }
	   sub_flashlight_en_l= pinctrl_lookup_state(sub_flashlightpinctrl, "flashlightsubpin_en0");
	   if (IS_ERR(sub_flashlight_en_l)) {
	   	PK_DBG("IS_ERR(sub_flashlight_en_l) \n");
	   return -1;	
	   }
	   sub_flashlight_en_h= pinctrl_lookup_state(sub_flashlightpinctrl, "flashlightsubpin_en1");
	   if (IS_ERR(sub_flashlight_en_h)) {
	   	PK_DBG("IS_ERR(sub_flashlight_en_h) \n");
	   return -1;	
	   }

    return 0;
}

static int Flashlight_sub_use_gpio_remove(struct platform_device *dev)	
{
	return 0;
}

static struct platform_driver Flashlight_sub_use_gpio_driver = {
	.probe	= Flashlight_sub_use_gpio_probe,
	.remove  = Flashlight_sub_use_gpio_remove,
	.driver    = {
	.name       = "flashlight_sub",
	.of_match_table = Flashlight_sub_use_gpio_of_match,	
	},
};

static int __init Flashlight_sub_use_gpio_init(void)
{
	PK_DBG("Flashlight_use_gpio_init\n");

	platform_driver_register(&Flashlight_sub_use_gpio_driver);
}

static void __exit Flashlight_sub_use_gpio_exit(void)
{
	
	 platform_driver_unregister(&Flashlight_sub_use_gpio_driver);
}


module_init(Flashlight_sub_use_gpio_init);
module_exit(Flashlight_sub_use_gpio_exit);

MODULE_DESCRIPTION("Flash driver for GPIO flashlight");
MODULE_AUTHOR("jack <jack.kang@tinno.com>");
MODULE_LICENSE("GPL v2");


static int FL_Enable(void)
{
	if(g_duty==0)
	{
		pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_en_h);
		pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_mode_l);
		PK_DBG(" FL_Enable line=%d\n",__LINE__);
	}
	else
	{	
		pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_en_h);
		pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_mode_h);
		PK_DBG(" FL_Enable line=%d\n",__LINE__);
	}

    return 0;
}



static int FL_Disable(void)
{

	pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_en_l);
	pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_mode_l);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}
static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d,duty=%d\n",__LINE__,duty);
	g_duty=duty;
    return 0;
}
static int FL_Init(void)
{
	pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_en_l);
	pinctrl_select_state(sub_flashlightpinctrl, sub_flashlight_mode_l);
    INIT_WORK(&workTimeOut_sub, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


static int FL_Uninit(void)
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
    //printk(KERN_ALERT "work handler function./n");
}

enum hrtimer_restart subledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut_sub);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=subledTimeOutCallback;

}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
        PK_DBG("sub dummy ioctl");
        int i4RetValue = 0;
        int ior_shift;
        int iow_shift;
        int iowr_shift;
        ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
        iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
        iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
        switch(cmd)
        {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %ld\n",arg);
		g_timeOutTimeMs=arg;
	break;


	case FLASH_IOC_SET_DUTY :
		PK_DBG("FLASHLIGHT_DUTY: %ld\n",arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %ld\n",arg);

		break;

	case FLASH_IOC_SET_ONOFF :
		PK_DBG("FLASHLIGHT_ONOFF: %ld\n",arg);
        if(arg==1)
        {
                if(g_timeOutTimeMs!=0)
                {
                        ktime_t ktime;
                        ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                        hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
                }
                FL_Enable();
        }
        else
        {
                FL_Disable();
                hrtimer_cancel( &g_timeOutTimer );
        }
        break;
	default :
		PK_DBG(" No such command \n");
		i4RetValue = -EPERM;
		break;
        }
        return i4RetValue;
//        return 0;
}

static int sub_strobe_open(void *pArg)
{
        PK_DBG("sub dummy open");

        int i4RetValue = 0;
        PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

        if (0 == strobe_Res)
        {
                FL_Init();
                timerInit();
        }
        PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
        spin_lock_irq(&g_strobeSMPLock_sub);

        if(strobe_Res)
        {
                PK_ERR(" busy!\n");
                i4RetValue = -EBUSY;
        }
        else
        {
                strobe_Res += 1;
        }
        spin_unlock_irq(&g_strobeSMPLock_sub);

        return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub dummy release");
	
    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock_sub);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock_sub);

    	FL_Uninit();
    }
	
    return 0;
}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}






