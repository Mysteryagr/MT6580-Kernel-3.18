/* ------------------------------------------------------------------------- */
/* hall_device.c - a device driver for the hall device interface             */
/* ------------------------------------------------------------------------- */
/*   
    Copyright (C) 2013-2020 Leatek Co.,Ltd

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* ------------------------------------------------------------------------- */
/*#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/time.h>

#include <linux/string.h>
#include <linux/of_irq.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>

#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/mt_pm_ldo.h>

#include <linux/mtgpio.h>
#include <linux/gpio.h>
*/
#include <linux/kthread.h>

#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mt-plat/aee.h>
#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#include <linux/sbsuspend.h>	/* smartbook */
#endif
#include <linux/atomic.h>

#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/types.h>



/*------
----------------------------------------------------------------
static variable defination
----------------------------------------------------------------------*/

//#define HALL_DEVNAME    "hall_dev"

#define EN_DEBUG

#if defined(EN_DEBUG)
		
#define TRACE_FUNC 	printk("[hall_dev] function: %s, line: %d \n", __func__, __LINE__);

#define HALL_DEBUG  printk
#else

#define TRACE_FUNC(x,...)

#define HALL_DEBUG(x,...)
#endif

#define  HALL_CLOSE  0
#define  HALL_OPEN    1

//#define HALL_SWITCH_EINT        CUST_EINT_MHALL_NUM
//#define HALL_SWITCH_DEBOUNCE    CUST_EINT_MHALL_DEBOUNCE_CN		/* ms */
//#define HALL_SWITCH_TYPE        CUST_EINT_MHALL_POLARITY
//#define HALL_SWITCH_SENSITIVE   CUST_EINT_MHALL_SENSITIVE

/****************************************************************/
/*******static function defination                             **/
/****************************************************************/

static struct device *hall_nor_device = NULL;
static struct input_dev *hall_input_dev;
static  int cur_hall_status = HALL_OPEN;
struct wake_lock hall_key_lock;
static int hall_key_event = 0;
static int g_hall_first = 1;
int hall_irq;
#if defined(CONFIG_OF)
static irqreturn_t hall_eint_func(int irq,void *data);
#else
static void hall_eint_func(unsigned long data);
#endif

static atomic_t send_event_flag = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(send_event_wq);
static int hall_probe(struct platform_device *pdev);
static int hall_remove(struct platform_device *dev);
/****************************************************************/
/*******export function defination                             **/
/****************************************************************/
#if defined(CONFIG_OF)
struct platform_device hall_device = {
	.name	  ="hall",
	.id		  = -1,
	/* .dev    ={ */
	/* .release = accdet_dumy_release, */
	/* } */
};

#endif
struct of_device_id hall_of_match[] = {
	{ .compatible = "mediatek, hall", },
	{},
};

static struct platform_driver hall_driver = {
	.probe	= hall_probe,
	.remove  = hall_remove,
	.driver    = {
	.name       = "hall",
	.of_match_table = hall_of_match,	
	},
};

static ssize_t hall_status_info_show(struct device_driver *ddri, char *buf)
{
	HALL_DEBUG("[hall_dev] cur_hall_status=%d\n", cur_hall_status);
	return sprintf(buf, "%d\n", cur_hall_status);
}

static ssize_t hall_status_info_store(struct device_driver * ddri,char * buf,size_t count)
{
	HALL_DEBUG("[hall_dev] %s ON/OFF value = %d:\n ", __func__, cur_hall_status);

	if(sscanf(buf, "%u", &cur_hall_status) != 1)
	{
		HALL_DEBUG("[hall_dev]: Invalid values\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(hall_state, 0664, hall_status_info_show,  hall_status_info_store);

static int sendKeyEvent(void *unuse)
{
   while(1)
    {
    
        HALL_DEBUG("[hall_dev]:sendKeyEvent wait\n");
#ifdef CONFIG_OF	

	 enable_irq(hall_irq);
	 HALL_DEBUG("[HALL]enable_irq  !!!!!!\n");
#else
     /* for detecting the return to old_hall_state */
        mt_eint_unmask(HALL_SWITCH_EINT);
#endif    	
		
        //wait for signal
        wait_event_interruptible(send_event_wq, (atomic_read(&send_event_flag) != 0));

        wake_lock_timeout(&hall_key_lock, 2*HZ);    //set the wake lock.
        HALL_DEBUG("[hall_dev]:going to send event %d\n", hall_key_event);
#ifdef CONFIG_OF
        disable_irq(hall_irq);
#else
        mt_eint_mask(HALL_SWITCH_EINT);
#endif

        //send key event
        if(HALL_OPEN == hall_key_event)
          {
                HALL_DEBUG("[hall_dev]:HALL_OPEN!\n");
                input_report_key(hall_input_dev, KEY_HALLOPEN, 1);
                input_report_key(hall_input_dev, KEY_HALLOPEN, 0);
                input_sync(hall_input_dev);
          }
	  else if(HALL_CLOSE == hall_key_event)
          {
                HALL_DEBUG("[hall_dev]:HALL_CLOSE!\n");
                input_report_key(hall_input_dev, KEY_HALLCLOSE, 1);
                input_report_key(hall_input_dev, KEY_HALLCLOSE, 0);
                input_sync(hall_input_dev);
          }
	
	  
        atomic_set(&send_event_flag, 0);
    }
    return 0;
}

static ssize_t notify_sendKeyEvent(int event)
{
    hall_key_event = event;
    atomic_set(&send_event_flag, 1);
    wake_up(&send_event_wq);
    HALL_DEBUG("[hall_dev]:notify_sendKeyEvent !\n");
    return 0;
}

#ifdef CONFIG_OF
static irqreturn_t hall_eint_func(int irq,void *data)
{
	int ret=0;

	HALL_DEBUG("hall_eint_func \n");	
	if(cur_hall_status ==  HALL_CLOSE ) 
	{
		HALL_DEBUG("hall_eint_func  HALL_OPEN\n");
		notify_sendKeyEvent(HALL_OPEN);
		//if (HALL_SWITCH_TYPE == CUST_EINT_POLARITY_HIGH){
		//    irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_HIGH);
		//}else{
		    irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_LOW);
		//}
		//mt_gpio_set_debounce(HALL_SWITCH_EINT,HALL_SWITCH_DEBOUNCE);

		/* update the eint status */
		cur_hall_status = HALL_OPEN;
	} 
	else 
	{
		HALL_DEBUG("hall_eint_func  HALL_CLOSE\n");
		notify_sendKeyEvent(HALL_CLOSE);
		//if (HALL_SWITCH_TYPE == CUST_EINT_POLARITY_HIGH){
		//    irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_LOW);
		//}else{
		    irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_HIGH);
		//}
	
		//mt_gpio_set_debounce(HALL_SWITCH_EINT,HALL_SWITCH_DEBOUNCE);

		cur_hall_status = HALL_CLOSE;
	}
	return IRQ_HANDLED;
}
#else
static void hall_eint_func(unsigned long data)
{
    
    TRACE_FUNC;

    mt65xx_eint_mask(HALL_SWITCH_EINT);

	if(cur_hall_status ==  HALL_CLOSE ) 
   {
   	HALL_DEBUG("[hall_dev]:HALL_opened \n");
	notify_sendKeyEvent(HALL_OPEN);
	cur_hall_status = HALL_OPEN;
   }
   else
   {
   	HALL_DEBUG("[hall_dev]:HALL_closed \n");
	notify_sendKeyEvent(HALL_CLOSE);
	cur_hall_status = HALL_CLOSE;
   }
   mt65xx_eint_set_polarity(HALL_SWITCH_EINT, cur_hall_status);
    mdelay(10); 
    mt65xx_eint_unmask(HALL_SWITCH_EINT);
}
#endif
struct pinctrl *hallpinctrl = NULL;

static  int hall_setup_eint(void)
{
	int ret;
#ifdef CONFIG_OF
	u32 ints[2]={0,0};
	unsigned int gpiopin, debounce;
	struct device_node *node;
#endif
	struct pinctrl_state *pins_cfg;

	/*configure to GPIO function, external interrupt*/
    HALL_DEBUG("[Hall]hall_setup_eint\n");
	pins_cfg = pinctrl_lookup_state(hallpinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
	
		HALL_DEBUG("Cannot find alsps pinctrl pin_cfg!\n");
	
	}
/*
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_DISABLE); //To disable GPIO PULL.
*/
#ifdef CONFIG_OF
    node = of_find_compatible_node(NULL,NULL,"mediatek, MHALL-eint");
	if(node) {
		
		
        of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));
	//	gpiopin = ints[0];
	//	debounce = ints[1];
	//	mt_gpio_set_debounce(gpiopin,debounce);
		pinctrl_select_state(hallpinctrl, pins_cfg);
		hall_irq = irq_of_parse_and_map(node,0);
		ret = request_irq(hall_irq,hall_eint_func,IRQF_TRIGGER_NONE,"MHALL-eint",NULL);
		if(ret>0){
            HALL_DEBUG("[Hall]EINT IRQ LINE£NNOT AVAILABLE\n");
		}
	}
	else {
        HALL_DEBUG("[Hall]%s can't find compatible node\n", __func__);
	}
#else
	mt_eint_set_sens(HALL_SWITCH_EINT, HALL_SWITCH_SENSITIVE);
	mt_eint_set_hw_debounce(HALL_SWITCH_EINT, HALL_SWITCH_DEBOUNCE);
	mt_eint_registration(HALL_SWITCH_EINT, HALL_SWITCH_DEBOUNCE, HALL_SWITCH_TYPE, hall_eint_func, 0);
	mt_eint_unmask(HALL_SWITCH_EINT);
#endif

    return 0;
}

static int hall_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct task_struct *keyEvent_thread = NULL;

    TRACE_FUNC;
	
		hallpinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(hallpinctrl)) {
			HALL_DEBUG("hall_probe   Cannot find hall pinctrl!");
		return -ENOMEM;	
		}

   hall_input_dev = input_allocate_device();
	
   if (!hall_input_dev)
   {
   	HALL_DEBUG("[hall_dev]:hall_input_dev : fail!\n");
       return -ENOMEM;
   }

   __set_bit(EV_KEY, hall_input_dev->evbit);
   __set_bit(KEY_HALLOPEN, hall_input_dev->keybit);
  __set_bit(KEY_HALLCLOSE, hall_input_dev->keybit);

  hall_input_dev->id.bustype = BUS_HOST;
  hall_input_dev->name = "HALL_DEV";
  if(input_register_device(hall_input_dev))
  {
	HALL_DEBUG("[hall_dev]:hall_input_dev register : fail!\n");
  }else
  {
	HALL_DEBUG("[hall_dev]:hall_input_dev register : success!!\n");
  }

   wake_lock_init(&hall_key_lock, WAKE_LOCK_SUSPEND, "hall key wakelock");
  
   init_waitqueue_head(&send_event_wq);
   //start send key event thread
   keyEvent_thread = kthread_run(sendKeyEvent, 0, "keyEvent_send");
   if (IS_ERR(keyEvent_thread)) 
   { 
      ret = PTR_ERR(keyEvent_thread);
      HALL_DEBUG("[hall_dev]:failed to create kernel thread: %d\n", ret);
   }

   if(g_hall_first)
   {
    	ret = driver_create_file(&hall_driver.driver, &dev_attr_hall_state.attr);
    	if (ret) 
	{
        	HALL_DEBUG("[hall_dev]:%s: sysfs_create_file failed\n", __func__);
        	driver_remove_file(&hall_driver.driver,&dev_attr_hall_state.attr);
    	}
	hall_setup_eint();
	g_hall_first = 0;
   }	
    return 0;
}

static int hall_remove(struct platform_device *dev)	
{
	HALL_DEBUG("[hall_dev]:hall_remove begin!\n");

	input_unregister_device(hall_input_dev);
	HALL_DEBUG("[hall_dev]:hall_remove Done!\n");
    
	return 0;
}

static int __init hall_init(void)
{
	int ret = 0;
    TRACE_FUNC;
#ifdef CONFIG_WIKO_UNIFY    
    extern int Hall;
    if (Hall == 0)
    {
    		HALL_DEBUG("[hall_dev]:Hall == 0!\n");
        return 0;
    }
#endif
    
#ifdef CONFIG_MTK_LEGACY
#if defined(CONFIG_OF)
    ret = platform_device_register(&hall_device);
    HALL_DEBUG("[%s]: hall_device, retval=%d \n!", __func__, ret);

	if (ret != 0)
	{
		HALL_DEBUG("platform_device_hall_device_register error:(%d)\n", ret);
		return ret;
	}
	else
	{
		HALL_DEBUG("platform_device_hall_device_register done!\n");
	}
#endif	
#endif
    platform_driver_register(&hall_driver);

    return 0;
}

static void __exit hall_exit(void)
{
    TRACE_FUNC;
    platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_DESCRIPTION("HALL DEVICE driver");
MODULE_AUTHOR("liling <ling.li@tinno.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("halldevice");

