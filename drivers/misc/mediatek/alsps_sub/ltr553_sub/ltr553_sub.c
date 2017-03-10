/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.   
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/ltr553.c - ltr553 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
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

/*
 * Change log
 *
 * Date     Author      Description
 * ----------------------------------------------------------------------------
 * 11/17/2011   chenqy      Initial modification from ltr502.
 * 01/03/2012   chenqy      Fix logical error in sensor enable function.
 *
 */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
	
#include "cust_alsps_sub.h"
#include "ltr553_sub.h"
#include "alsps_sub.h"
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif
#define POWER_NONE_MACRO -1   

/******************************************************************************
 * extern functions
*******************************************************************************/
#ifdef CUST_EINT_ALS_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif


/******************************************************************************
 * configuration
*******************************************************************************/
#define I2C_DRIVERID_ltr553_sub 559
/*----------------------------------------------------------------------------*/
#define ltr553_SUB_I2C_ADDR_RAR 0 /*!< the index in obj->hw->i2c_addr: alert response address */
#define ltr553_SUB_I2C_ADDR_ALS 1 /*!< the index in obj->hw->i2c_addr: ALS address */
#define ltr553_SUB_I2C_ADDR_PS  2 /*!< the index in obj->hw->i2c_addr: PS address */
#define ltr553_sub_DEV_NAME     "ltr553_sub"
/*----------------------------------------------------------------------------*/
#define APS_SUB_TAG "[ALS/PS_SUB] "
#define APS_SUB_DEBUG
#if defined(APS_SUB_DEBUG)
#define APS_SUB_FUN(f)      printk(KERN_INFO APS_SUB_TAG"%s\n", __FUNCTION__)
#define APS_SUB_ERR(fmt, args...)   printk(KERN_ERR APS_SUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_SUB_LOG(fmt, args...)   printk(KERN_ERR APS_SUB_TAG "%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#define APS_SUB_DBG(fmt, args...)   printk(KERN_ERR fmt, ##args)
#else
#define APS_SUB_FUN(f)
#define APS_SUB_ERR(fmt, args...)
#define APS_SUB_LOG(fmt, args...)
#define APS_SUB_DBG(fmt, args...)
#endif


#define GN_MTK_BSP_ALSPS_SUB_INTERRUPT_MODE
//#define GN_MTK_BSP_PS_DYNAMIC_CALI
#define FEATURE_PS_SUB_CALIBRATION

#ifdef FEATURE_PS_SUB_CALIBRATION
static int g_ps_sub_cali_flag = 0;
static int g_ps_sub_base_value = 0;
static int g_tp_tpye_checked = 0;
extern char * synaptics_get_vendor_info(void);
static void ps_sub_cali_tp_check(void);
static void ps_sub_cali_set_threshold(void);
static void ps_sub_cali_start(void);
#endif
/******************************************************************************
 * extern functions
*******************************************************************************/
static struct i2c_client *ltr553_sub_i2c_client = NULL;
struct platform_device *alspsSubPltFmDev;

//static struct wake_lock ps_wake_lock;
//static int ps_wakeup_timeout = 3;
static int als_sub_times = 0;
static int als_sub_value[3] = {0};
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr553_sub_i2c_id[] = {{ltr553_sub_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_ltr553= { I2C_BOARD_INFO(ltr553_DEV_NAME, (ltr553_I2C_SLAVE_ADDR>>1))};

/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltr553_sub_i2c_remove(struct i2c_client *client);
static int ltr553_sub_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr553_sub_i2c_resume(struct i2c_client *client);

static int ltr553_sub_init_flag = 0;
static int dynamic_calibrate = 0;
static u8 part_id = 0;
static u8 als_sub_range = 0;
static u8 ps_sub_gain = 0;

static u16 lux_val_prev = 0;
#define ALS_GAIN_1x     (0 << 2)
#define ALS_GAIN_2x     (1 << 2)
#define ALS_GAIN_4x     (2 << 2)
#define ALS_GAIN_8x     (3 << 2)
#define ALS_GAIN_48x    (6 << 2)
#define ALS_GAIN_96x    (7 << 2)
static u8 eqn_prev = 0;
static u8 ratio_old = 0;
static u16 winfac1 = 100;
static u16 winfac2 = 80;
static u16 winfac3 = 44;

static struct ltr553_sub_priv *g_ltr553_sub_ptr = NULL;

/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_TRC_APS_DATA    = 0x0002,
    CMC_TRC_EINT        = 0x0004,
    CMC_TRC_IOCTL       = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS     = 0x0020,
    CMC_TRC_CVT_PS      = 0x0040,
    CMC_TRC_DEBUG       = 0x8000,
} SUB_CMC_TRC;
/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS     = 1,
    CMC_BIT_PS      = 2,
} SUB_CMC_BIT;
/*----------------------------------------------------------------------------*/


struct PS_SUB_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_SUB_CALI_DATA_STRUCT ps_sub_cali= {0,0,0};
struct alsps_sub_hw alsps_sub_cust;
static struct alsps_sub_hw *hw = &alsps_sub_cust;

/* For alsp driver get cust info */
struct alsps_sub_hw *get_cust_alsps_sub(void) {
    return &alsps_sub_cust;
}
/*----------------------------------------------------------------------------*/
struct ltr553_sub_priv
{
    struct alsps_sub_hw *hw;
    struct i2c_client *client;
#ifdef GN_MTK_BSP_ALSPS_SUB_INTERRUPT_MODE
    struct delayed_work eint_work;
#endif /* GN_MTK_BSP_ALSPS_INTERRUPT_MODE */
    //struct timer_list first_read_ps_timer;
    //struct timer_list first_read_als_timer;

    /*misc*/
    atomic_t    trace;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on; /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;    /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;  /*indicates if the debounce is on*/
    atomic_t    ps_deb_end; /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
	atomic_t  init_done;
	struct device_node *irq_node;
	int		irq;

    /*data*/
    // u8       als;
    // u8       ps;
    int     als;
    int     ps;
    u8      _align;
    u16     als_level_num;
    u16     als_value_num;
    u32     als_level[C_CUST_ALS_LEVEL-1];
    u32     als_value[C_CUST_ALS_LEVEL];

    bool        als_enable; /*record current als status*/
    unsigned int    als_widow_loss;

    bool        ps_enable;   /*record current ps status*/
    unsigned int    ps_thd_val;  /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;      /*record HAL enalbe status*/
    ulong       pending_intr;   /*pending interrupt*/
    //ulong     first_read; // record first read ps and als
    unsigned int    polling;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
//static struct platform_driver ltr553_alsps_driver;
#ifdef CONFIG_OF
static const struct of_device_id alsps_sub_of_match[] = {
        {.compatible = "mediatek,alsps_sub"},
        {},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr553_sub_i2c_driver =
{
    .probe      = ltr553_sub_i2c_probe,
    .remove     = ltr553_sub_i2c_remove,
    .detect     = ltr553_sub_i2c_detect,
    .suspend    = ltr553_sub_i2c_suspend,
    .resume     = ltr553_sub_i2c_resume,
    .id_table   = ltr553_sub_i2c_id,
//  .address_data   = &ltr553_addr_data,
    .driver = {
//      .owner  = THIS_MODULE,
        .name   = ltr553_sub_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = alsps_sub_of_match,
#endif

    },
};

struct ltr553_sub_priv *ltr553_sub_obj = NULL;

//static int ps_gainrange;
//static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

/*
 * The ps_trigger_xxx_table
 * controls the interrupt trigger range
 * bigger value means close to p-sensor
 * smaller value means far away from p-sensor
 */
int ltr553_sub_get_ps_value(struct ltr553_sub_priv *obj, int ps);
static int ltr553_sub_get_als_value(struct ltr553_sub_priv *obj, int als);

#ifdef GN_MTK_BSP_PS_SUB_DYNAMIC_CALI
static int ltr553_sub_dynamic_calibrate(void);
#endif
int AudioChannel = 0;
int GsensorState = 0;
#if 1
static int ltr553_sub_power(struct alsps_sub_hw *hw, unsigned int on);

static int  ltr553_sub_local_init(void)
{
  //  struct alsps_hw *hw = get_cust_alsps_hw();
	//printk("fwq loccal init+++\n");
    printk("[xucs]ltr553_sub_local_init\n");
	ltr553_sub_power(hw, 1);
	if(i2c_add_driver(&ltr553_sub_i2c_driver))
	{
		APS_SUB_ERR("add sub driver error\n");
		return -1;
	}
	if(-1 == ltr553_sub_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}

static int ltr553_sub_alsps_remove(void)
{
//    struct alsps_hw *hw = get_cust_alsps_hw();
    APS_SUB_FUN();
    ltr553_sub_power(hw, 0);
    i2c_del_driver(&ltr553_sub_i2c_driver);
    return 0;
}



static struct alsps_sub_init_info ltr553_sub_init_info = {
		.name = "ltr553_sub",
		.init = ltr553_sub_local_init,
		.uninit = ltr553_sub_alsps_remove,
};
#endif

/*----------------------------------------------------------------------------*/
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 buf;
    int ret = 0;
    struct i2c_client client_read = *client;

    client_read.addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
    buf = addr;
    ret = i2c_master_send(&client_read, (const char*)&buf, 1<<8 | 1);
    if (ret < 0)
    {
        APS_SUB_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
    client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

/*----------------------------------------------------------------------------*/
static int lux_formula(int ch0_adc, int ch1_adc, u8 eqtn)
{
    int luxval = 0;
    int luxval_i = 0;
    int luxval_f = 0;
    u16 ch0_coeff_i = 0;
    u16 ch1_coeff_i = 0;
    u16 ch0_coeff_f = 0;
    u16 ch1_coeff_f = 0;
    int8_t ret;
    u8 gain = 1, als_int_fac;
    u8 buf;
    u16 win_fac = 0;
    int8_t fac = 1;

    if (hwmsen_read_byte_sr(ltr553_sub_obj->client, APS_RO_ALS_PS_STATUS, &buf))
    {
        APS_SUB_ERR("reads als data error APS_RO_ALS_PS_STATUS = %d\n", buf);
        return -EFAULT;
    }

    gain = (buf & 0x70);
    gain >>= 4;

    if (gain == 0)              //gain 1
    {
        gain = 1;
    }
    else if (gain == 1)         //gain 2
    {
        gain = 2;
    }
    else if (gain == 2)         //gain 4
    {
        gain = 4;
    }
    else if (gain == 3)         //gain 8
    {
        gain = 8;
    }
    else if (gain == 6)         //gain 48
    {
        gain = 48;
    }
    else if (gain == 7)         //gain 96
    {
        gain = 96;
    }

    if (hwmsen_read_byte_sr(ltr553_sub_obj->client, APS_RW_ALS_MEAS_RATE, &buf))
    {
        APS_SUB_ERR("reads als sub data error APS_RW_ALS_MEAS_RATE = %d\n", buf);
        return -EFAULT;
    }

    als_int_fac = buf & 0x38;
    als_int_fac >>= 3;

    if (als_int_fac == 0)
    {
        als_int_fac = 10;
    }
    else if (als_int_fac == 1)
    {
        als_int_fac = 5;
    }
    else if (als_int_fac == 2)
    {
        als_int_fac = 20;
    }
    else if (als_int_fac == 3)
    {
        als_int_fac = 40;
    }
    else if (als_int_fac == 4)
    {
        als_int_fac = 15;
    }
    else if (als_int_fac == 5)
    {
        als_int_fac = 25;
    }
    else if (als_int_fac == 6)
    {
        als_int_fac = 30;
    }
    else if (als_int_fac == 7)
    {
        als_int_fac = 35;
    }

    if (eqtn == 1)
    {
        ch0_coeff_i = 1;
        ch1_coeff_i = 1;
        ch0_coeff_f = 7743;
        ch1_coeff_f = 1059;
        fac = 1;
        win_fac = winfac1;
        luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
        luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
        //luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));
        //luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) / (gain * (als_int_fac / 10));
    }
    else if (eqtn == 2)
    {
        ch0_coeff_i = 4;
        ch1_coeff_i = 1;
        ch0_coeff_f = 2785;
        ch1_coeff_f = 696;
        win_fac = winfac2;
        if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f))
        {
            fac = 1;
            luxval_f = (((ch0_adc * ch0_coeff_f) - (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
        }
        else
        {
            fac = -1;
            luxval_f = (((ch1_adc * ch1_coeff_f) - (ch0_adc * ch0_coeff_f)) / 100) * win_fac;
        }
        luxval_i = ((ch0_adc * ch0_coeff_i) - (ch1_adc * ch1_coeff_i)) * win_fac;
        //luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));
        //luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) / (gain * (als_int_fac / 10));
    }
    else if (eqtn == 3)
    {
        ch0_coeff_i = 0;
        ch1_coeff_i = 0;
        ch0_coeff_f = 5926;
        ch1_coeff_f = 1300;
        fac = 1;
        win_fac = winfac3;
        luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
        luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
        //luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));
        //luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) / (gain * (als_int_fac / 10));
    }
    else if (eqtn == 4)
    {
        ch0_coeff_i = 0;
        ch1_coeff_i = 0;
        ch0_coeff_f = 0;
        ch1_coeff_f = 0;
        fac = 1;
        luxval_i = 0;
        luxval_f = 0;
        //luxval = 0;
    }

    if (fac < 0)
    {
        luxval = (luxval_i  - (luxval_f / 100)) / (gain * als_int_fac);
    }
    else
    {
        luxval = (luxval_i  + (luxval_f / 100)) / (gain * als_int_fac);
    }

    return luxval;
}

/*----------------------------------------------------------------------------*/
static int ratioHysterisis (int ch0_adc, int ch1_adc)
{
#define RATIO_HYSVAL    10
    int ratio;
    u8 buf, eqn_now;
    int ch0_calc;
    int luxval = 0;
    int abs_ratio_now_old;

    if (hwmsen_read_byte_sr(ltr553_sub_obj->client, APS_RW_ALS_CONTR, &buf))
    {
        APS_SUB_ERR("reads als data error APS_RW_ALS_CONTR = %d\n", buf);
        return -EFAULT;
    }

    ch0_calc = ch0_adc;
    if ((buf & 0x20) == 0x20)
    {
        ch0_calc = ch0_adc - ch1_adc;
    }

    if ((ch1_adc + ch0_calc) == 0)
    {
        ratio = 100;
    }
    else
    {
        ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);
    }

    if (ratio < 45)
    {
        eqn_now = 1;
    }
    else if ((ratio >= 45) && (ratio < 68))
    {
        eqn_now = 2;
    }
    else if ((ratio >= 68) && (ratio < 99))
    {
        eqn_now = 3;
    }
    else if (ratio >= 99)
    {
        eqn_now = 4;
    }

    if (eqn_prev == 0)
    {
        luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
        ratio_old = ratio;
        eqn_prev = eqn_now;
    }
    else
    {
        if (eqn_now == eqn_prev)
        {
            luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
            ratio_old = ratio;
            eqn_prev = eqn_now;
        }
        else
        {
            abs_ratio_now_old = ratio - ratio_old;
            if (abs_ratio_now_old < 0)
            {
                abs_ratio_now_old *= (-1);
            }
            if (abs_ratio_now_old > RATIO_HYSVAL)
            {
                luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
                ratio_old = ratio;
                eqn_prev = eqn_now;
            }
            else
            {
                luxval = lux_formula(ch0_calc, ch1_adc, eqn_prev);
            }
        }
    }

    return luxval;
}


/*----------------------------------------------------------------------------*/
int ltr553_sub_read_data_als(struct i2c_client *client, int *data)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int ret = 0;
    int alsval_ch1_lo = 0;
    int alsval_ch1_hi = 0;
    int alsval_ch0_lo = 0;
    int alsval_ch0_hi = 0;
    int luxdata_int;
    int luxdata_flt;
    int ratio;
    int alsval_ch0;
    int alsval_ch1;

    int als_zero_try = 0;

    u8 buf;
    u8 value_temp;
    u8 temp, gain;
    u8 gain_chg_req = 0;
#define AGC_UP_THRESHOLD        40000
#define AGC_DOWN_THRESHOLD      5000
#define AGC_HYS                 15
#define MAX_VAL                 50000

als_data_try:
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_0, &alsval_ch1_lo))
    {
        APS_SUB_ERR("reads als sub data error (ch1 lo) = %d\n", alsval_ch1_lo);
        return -EFAULT;
    }
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_1, &alsval_ch1_hi))
    {
        APS_SUB_ERR("reads aps sub data error (ch1 hi) = %d\n", alsval_ch1_hi);
        return -EFAULT;
    }
    alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
    APS_SUB_ERR("alsval_ch1_hi=%x alsval_ch1_lo=%x\n",alsval_ch1_hi,alsval_ch1_lo);


    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_0, &alsval_ch0_lo))
    {
        APS_SUB_ERR("reads als data error (ch0 lo) = %d\n", alsval_ch0_lo);
        return -EFAULT;
    }
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_1, &alsval_ch0_hi))
    {
        APS_SUB_ERR("reads als data error (ch0 hi) = %d\n", alsval_ch0_hi);
        return -EFAULT;
    }
    alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
    APS_SUB_ERR("alsval_ch0_hi=%x alsval_ch0_lo=%x\n",alsval_ch0_hi,alsval_ch0_lo);


    if (hwmsen_read_byte_sr(client, APS_RO_ALS_PS_STATUS, &buf))
    {
        APS_SUB_ERR("reads als data error APS_RO_ALS_PS_STATUS = %d\n", buf);
        return -EFAULT;
    }
    value_temp = buf;
    temp = buf;
    gain = (value_temp & 0x70);
    gain >>= 4;

    if (gain == 0)              //gain 1
    {
        gain = 1;
    }
    else if (gain == 1)         //gain 2
    {
        gain = 2;
    }
    else if (gain == 2)         //gain 4
    {
        gain = 4;
    }
    else if (gain == 3)         //gain 8
    {
        gain = 8;
    }
    else if (gain == 6)         //gain 48
    {
        gain = 48;
    }
    else if (gain == 7)         //gain 96
    {
        gain = 96;
    }

    if (hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &buf))
    {
        APS_SUB_ERR("reads als data error APS_RW_ALS_CONTR = %d\n", buf);
        return -EFAULT;
    }
    value_temp = buf;
    value_temp &= 0xE3;

    if ((alsval_ch0 == 0) && (alsval_ch1 > 50 ))
    {
        *data = lux_val_prev;
    }
    else
    {
        if (gain == 1)
        {
            if ((alsval_ch0 + alsval_ch1) < ((AGC_DOWN_THRESHOLD * 10) / AGC_HYS))
            {
                *data = ratioHysterisis(alsval_ch0, alsval_ch1);
                value_temp |= ALS_GAIN_8x;
                gain_chg_req = 1;
            }
            else
            {
                *data = ratioHysterisis(alsval_ch0, alsval_ch1);
            }
        }
        else if (gain == 8)
        {
            if ((alsval_ch0 + alsval_ch1) > AGC_UP_THRESHOLD)
            {
                *data = ratioHysterisis(alsval_ch0, alsval_ch1);
                value_temp |= ALS_GAIN_1x;
                gain_chg_req = 1;
            }
            else
            {
                *data = ratioHysterisis(alsval_ch0, alsval_ch1);
            }
        }
        else
        {
            *data = ratioHysterisis(alsval_ch0, alsval_ch1);
        }
        if (gain_chg_req)
        {
            if (hwmsen_write_byte(client, APS_RW_ALS_CONTR, value_temp))
            {
                APS_SUB_LOG("auto change error!\n");
                return -1;
            }
        }

    }

    if ((*data > MAX_VAL) || (((alsval_ch0 + alsval_ch1) > MAX_VAL) && (temp & 0x80)))
    {
        *data = MAX_VAL;
    }
    lux_val_prev = *data;

    APS_SUB_ERR("ltr553 sub als value: =%x \n",lux_val_prev);

    return 0;

}
#define LOW_TEMPERATURE
#ifdef LOW_TEMPERATURE
static int min_value_low_temperature=0x7ff;
static int set_first_value_flag_low_temperature=0;
static int reset_flag_low_temperature=0;

static int set_low_temperature_threshold_value(struct ltr553_sub_priv *obj,int ps)
{
	int temp_value_high;
	int temp_value_low;
	//struct alsps_hw *hw = get_cust_alsps_hw();

	APS_SUB_LOG(" reset_low_temperature_threshold_value:  ps=%d \n",ps);
	APS_SUB_LOG("reset_low_temperature_threshold_value:reset_flag_low_temperature= %d  set_first_value_flag_low_temperature=%d \n", reset_flag_low_temperature,set_first_value_flag_low_temperature);

	if(reset_flag_low_temperature==1)
	{
		goto end_set_threshold;
	}
		
	APS_SUB_LOG("set_low_temperature_threshold_value:g_ps_base_value= %d \n", g_ps_sub_base_value);
	APS_SUB_LOG(" set_low_temperature_threshold_value:readobj->ps_thd_val_high=%x, obj->ps_thd_val_low=%x! \n",atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));

	if(min_value_low_temperature>ps)
	{
		min_value_low_temperature=ps;
	}

	if(g_ps_sub_base_value+50>ps)
	{
		min_value_low_temperature=g_ps_sub_base_value;
		reset_flag_low_temperature=1;
		
	}
	temp_value_high=min_value_low_temperature+hw->ps_threshold_high;
	temp_value_low=min_value_low_temperature+ hw->ps_threshold_low;
	
	if(set_first_value_flag_low_temperature==0)
	{
		if(g_ps_sub_base_value+50<ps)
		{	

			APS_SUB_LOG(" set_low_temperature_threshold_value:PS is larger than g_ps_base_value, and threshold  need to be reset! \n");
			APS_SUB_LOG(" set_low_temperature_threshold_value: ltr553  default_ps_highthresh=%x, default_ps_lowthresh=%x! \n",atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));
		
		}
		else
		{
			reset_flag_low_temperature=1;
			set_first_value_flag_low_temperature=1;
                                        atomic_set(&ltr553_sub_obj->ps_thd_val_high, temp_value_high);
                                        atomic_set(&ltr553_sub_obj->ps_thd_val_low,  temp_value_low);
			APS_SUB_LOG(" set_low_temperature_threshold_value:There is no need to set threshold! \n");
		
			goto end_set_threshold;		
		}
				
		set_first_value_flag_low_temperature=1;
	}
	
	if(temp_value_high> 0x7ff)
	{
		temp_value_high= 0x7ff ;
		temp_value_low= 0x7ff ;
		APS_SUB_LOG("set_low_temperature_threshold_value: set value_high=0x7ff,value_low=0x7ff, please check the phone \n");
	}
	
	APS_SUB_LOG(" set_low_temperature_threshold_value: ltr553  sub temp_value_high=%x, temp_value_low=%x! \n",temp_value_high,temp_value_low);

            atomic_set(&ltr553_sub_obj->ps_thd_val_high, temp_value_high);
            atomic_set(&ltr553_sub_obj->ps_thd_val_low,  temp_value_low);

	return ps;

	end_set_threshold: 
	return ps;
}
#endif


int ltr553_sub_read_data_ps(struct i2c_client *client, int *data)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int ret = 0;
    int psval_lo = 0;
    int psval_hi = 0;
    int psdata = 0;

    if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_0, &psval_lo))
    {
        APS_SUB_LOG("reads aps data = %d\n", psval_lo);
        return -EFAULT;
    }

    if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_1, &psval_hi))
    {
        APS_SUB_LOG("reads aps hi data = %d\n", psval_hi);
        return -EFAULT;
    }

    psdata = ((psval_hi & 0x87) * 256) + psval_lo;
    APS_SUB_LOG("psensor rawdata is:%d\n", psdata);
#ifdef LOW_TEMPERATURE
        psdata=set_low_temperature_threshold_value(obj,psdata);
#endif

    *data = psdata;
    final_prox_val = psdata;
    return 0;
}

/*----------------------------------------------------------------------------*/

int ltr553_sub_init_device(struct i2c_client *client)
{
    //struct ltr553_priv *obj = i2c_get_clientdata(client);
    APS_SUB_LOG("ltr553_sub_init_device.........\r\n");
	struct ltr553_sub_priv *obj = ltr553_sub_obj;
    u8 buf =0;
    int i = 0;
    int ret = 0;

    hwmsen_write_byte(client, 0x82, 0x1F);              //100mA,100%,60kHz      //yaoyaoqin
    hwmsen_write_byte(client, 0x83, 0x0F); //0x06              ///5 pulse
    hwmsen_write_byte(client, 0x85, 0x01);              //als measurement time 100 ms
    hwmsen_write_byte(client, 0x9e, 0x40);              ///2 consecutive data outside range to interrupt
    if(part_id == ltr553_PART_ID)
    {
        hwmsen_write_byte(client, 0x84,0x00 );          //0x0f ps measurement time 10 ms 0x01=70ms
        als_sub_range = ltr553_ALS_ON_Range8;
        ps_sub_gain = ltr553_PS_ON_Gain16;
    }
    else if(part_id == LTR558_PART_ID)
    {
        hwmsen_write_byte(client, 0x84, 0x0);           //ps measurement time 50 ms
        als_sub_range = LTR558_ALS_ON_Range2;
        ps_sub_gain = LTR558_PS_ON_Gain8;
    }
    else
    {
        APS_SUB_ERR("ltr553 sub part_id is error, part_id = 0x%x\n", part_id);
    }
	APS_SUB_LOG("LTR553 sub part_id = 0x%x\n", part_id);
#ifdef GN_MTK_BSP_ALSPS_SUB_INTERRUPT_MODE
    hwmsen_write_byte(client, 0x8f, 0x01);              //enable ps for interrupt mode
    hwmsen_write_byte(client, 0x90, (u8)((atomic_read(&obj->ps_thd_val_high)) & 0xff));
    hwmsen_write_byte(client, 0x91, (u8)((atomic_read(&obj->ps_thd_val_high))>>8) & 0X07);

    hwmsen_write_byte(client, 0x92, 0x00);
    hwmsen_write_byte(client, 0x93, 0x00);
#endif  // GN_MTK_BSP_ALSPS_INTERRUPT_MODE 

    mdelay(WAKEUP_DELAY);
    return 0;
}

int ltr553_sub_init_device_ESD_Recover(struct i2c_client *client)
{
	int err = 0;
	
  	if((err = ltr553_sub_init_device(client)))
	    {
	        APS_SUB_ERR("init dev: %d\n", err);
	        return err;
	    }
   	 return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr553_sub_power(struct alsps_sub_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;
    int status = 0;

    APS_SUB_LOG("power %s\n", on ? "on" : "off");
    APS_SUB_LOG("power id:%d POWER_NONE_MACRO:%d\n", hw->power_id, POWER_NONE_MACRO);
#if 1//def __USE_LINUX_REGULATOR_FRAMEWORK__
#else

    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
            status = 0;
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, "ltr553"))
            {
                APS_ERR("power on fails!!\n");
            }
            status =  1;
        }
        else
        {
            if(!hwPowerDown(hw->power_id, "ltr553"))
            {
                APS_ERR("power off fail!!\n");
            }
            status =  -1;
        }
    }
    power_on = on;

    return status;
#endif
}
/*----------------------------------------------------------------------------*/
static int ltr553_sub_enable_als(struct i2c_client *client, bool enable)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int err=0;
    int trc = atomic_read(&obj->trace);
    u8 regdata=0;
    u8 regint=0;
    int i;

    if(enable == obj->als_enable)
    {
        return 0;
    }
    else if(enable == true)
    {
        if (hwmsen_write_byte(client, APS_RW_ALS_CONTR, als_sub_range))
        {
            APS_SUB_LOG("ltr553_sub_enable_als enable failed!\n");
            return -1;
        }
        hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata);            //yaoyaoqin: need delay at least 50ms
        mdelay(200);
        APS_SUB_LOG("ltr553_sub_enable_als, regdata: 0x%x!\n", regdata);
    }
    else if(enable == false)
    {
        if (hwmsen_write_byte(client, APS_RW_ALS_CONTR, MODE_ALS_StdBy))
        {
            APS_SUB_LOG("ltr553_sub_enable_als disable failed!\n");
            return -1;
        }
        hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata);
        APS_SUB_LOG("ltr553_sub_enable_als, regdata: 0x%x!\n", regdata);
    }

    obj->als_enable = enable;

    als_sub_times = 0;
    for(i=0; i<3; i++)
    {
        als_sub_value[i] = 0;
    }

    if(trc & CMC_TRC_DEBUG)
    {
        APS_SUB_LOG("enable als (%d)\n", enable);
    }

    return err;
}
/*----------------------------------------------------------------------------*/

#define ltr553_ERR_I2C -1
static int ltr553_sub_ps_set_thres(void)

{
    APS_SUB_FUN();

    int res;
    u8 databuf[2];

    struct i2c_client *client = ltr553_sub_obj->client;
    struct ltr553_sub_priv *obj = ltr553_sub_obj;

    if(1 == ps_sub_cali.valid)
    {
        databuf[0] = APS_RW_PS_THRES_LOW_0;
        databuf[1] = (u8)(ps_sub_cali.far_away & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
        databuf[0] = APS_RW_PS_THRES_LOW_1;
        databuf[1] = (u8)((ps_sub_cali.far_away & 0xFF00) >> 8);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
        databuf[0] = APS_RW_PS_THRES_UP_0;
        databuf[1] = (u8)(ps_sub_cali.close & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
        databuf[0] = APS_RW_PS_THRES_UP_1;
        databuf[1] = (u8)((ps_sub_cali.close & 0xFF00) >> 8);;
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
    }
    else
    {
        databuf[0] = APS_RW_PS_THRES_LOW_0;
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
        databuf[0] = APS_RW_PS_THRES_LOW_1;
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) >> 8) & 0x00FF);

        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
        databuf[0] = APS_RW_PS_THRES_UP_0;
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high))& 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }
        databuf[0] = APS_RW_PS_THRES_UP_1;
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
        res = i2c_master_send(client, databuf, 0x2);
        if(res <= 0)
        {
            goto EXIT_ERR;
            return ltr553_ERR_I2C;
        }

    }

    res = 0;
    return res;

EXIT_ERR:
    APS_SUB_ERR("set thres: %d\n", res);
    return res;

}
#ifdef LOW_TEMPERATURE
static void low_temperature_value_all_reset(void)
{
        min_value_low_temperature=0x7ff;
        set_first_value_flag_low_temperature=0;
        reset_flag_low_temperature=0;
        
}
#endif

int ltr553_sub_enable_ps(struct i2c_client *client, bool enable)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int err=0;
    int trc = atomic_read(&obj->trace);
    u8 regdata = 0;
    u8 regint = 0;
   struct hwm_sensor_data sensor_data;




    APS_SUB_LOG(" ltr553_sub_enable_ps: enable:  %d, obj->ps_enable: %d\n",enable, obj->ps_enable);
    if (enable == obj->ps_enable)
    {
        return 0;
    }
    else if (enable == true)
    {

	    if((err = ltr553_sub_init_device_ESD_Recover(client)))
	    {
	        APS_SUB_LOG("init dev: %d\n", err);
	        return err;
	    }
		
        if (hwmsen_write_byte(client, APS_RW_PS_CONTR, ps_sub_gain))
        {
            APS_SUB_LOG("ltr553_sub_enable_ps enable failed!\n");
            return -1;
        }
        hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
        APS_SUB_ERR("ltr553_sub_enable_ps, regdata: %0xx!\n", regdata);

#ifdef GN_MTK_BSP_PS_SUB_DYNAMIC_CALI
        if(regdata == ps_sub_gain )
        {
#ifdef CUST_EINT_ALS_TYPE
            mt_eint_mask(CUST_EINT_ALS_NUM);
#else
            mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif

            ltr553_sub_dynamic_calibrate();

#ifdef CUST_EINT_ALS_TYPE
            mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
            mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
        }
#endif
		ltr553_sub_ps_set_thres();

        sensor_data.values[0] = 1;
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        if (err = hwmsen_get_interrupt_data(ID_PROXIMITY_SUB, &sensor_data))
        {
            APS_SUB_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
            return err;
        }

    }
    else if(enable == false)
    {
#ifdef LOW_TEMPERATURE
        low_temperature_value_all_reset();
#endif

        if (hwmsen_write_byte(client, APS_RW_PS_CONTR, MODE_PS_StdBy))
        {
            APS_SUB_LOG("ltr553_sub_enable_ps disable failed!\n");
            return -1;
        }
        hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
        APS_SUB_ERR("ltr553_sub_enable_ps, regdata: %0xx!\n", regdata);
    }
    obj->ps_enable = enable;
    if(trc & CMC_TRC_DEBUG)
    {
        APS_SUB_LOG("enable ps (%d)\n", enable);
    }

    return err;
}
/*----------------------------------------------------------------------------*/

static int ltr553_sub_check_intr(struct i2c_client *client)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int err;
    u8 data=0;

    // err = hwmsen_read_byte_sr(client,APS_INT_STATUS,&data);
    err = hwmsen_read_byte_sr(client,APS_RO_ALS_PS_STATUS,&data);
    APS_SUB_ERR("INT flage: = %x\n", data);

    if (err)
    {
        APS_SUB_ERR("WARNING: read int status: %d\n", err);
        return 0;
    }

    if (data & 0x08)
    {
        set_bit(CMC_BIT_ALS, &obj->pending_intr);
    }
    else
    {
        clear_bit(CMC_BIT_ALS, &obj->pending_intr);
    }

    if (data & 0x02)
    {
        set_bit(CMC_BIT_PS, &obj->pending_intr);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &obj->pending_intr);
    }

    if (atomic_read(&obj->trace) & CMC_TRC_DEBUG)
    {
        APS_SUB_LOG("check intr: 0x%lu\n", obj->pending_intr);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_ALSPS_SUB_INTERRUPT_MODE
static void ltr553_sub_eint_func(void)
{
    struct ltr553_sub_priv *obj = g_ltr553_sub_ptr;
    APS_SUB_LOG("fwq interrupt fuc\n");
    if(!obj)
    {
        return;
    }

    schedule_delayed_work(&obj->eint_work,0);
    if(atomic_read(&obj->trace) & CMC_TRC_EINT)
    {
        APS_SUB_LOG("eint: als/ps intrs\n");
    }
}
#if defined(CONFIG_OF)
static irqreturn_t ltr553_sub_eint_handler(int irq, void *desc)
{
	ltr553_sub_eint_func();
	disable_irq_nosync(ltr553_sub_obj->irq);
	
	return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/
static void ltr553_sub_eint_work(struct work_struct *work)
{
    struct ltr553_sub_priv *obj = (struct ltr553_sub_priv *)container_of(work, struct ltr553_sub_priv, eint_work);
    int err;
  struct  hwm_sensor_data sensor_data;
    int temp_noise = 0;
    u8 buf;

    APS_SUB_ERR("interrupt........");
	struct alsps_sub_hw *hw = get_cust_alsps_sub();
    memset(&sensor_data, 0, sizeof(sensor_data));

    if (0 == atomic_read(&obj->ps_deb_on))
    {
        // first enable do not check interrupt
        err = ltr553_sub_check_intr(obj->client);
    }

    if (err)
    {
        APS_SUB_ERR("check intrs: %d\n", err);
    }

    APS_SUB_ERR("ltr553_sub_eint_work obj->pending_intr =%lu, obj:%p\n",obj->pending_intr,obj);

    if ((1<<CMC_BIT_ALS) & obj->pending_intr)
    {
        // get raw data
        APS_SUB_ERR("fwq als INT\n");
        if (err = ltr553_sub_read_data_als(obj->client, &obj->als))
        {
            APS_SUB_ERR("ltr553 sub read als data: %d\n", err);;
        }
        //map and store data to hwm_sensor_data
        while(-1 == ltr553_sub_get_als_value(obj, obj->als))
        {
            ltr553_sub_read_data_als(obj->client, &obj->als);
            msleep(50);
        }
        sensor_data.values[0] = ltr553_sub_get_als_value(obj, obj->als);
        APS_SUB_LOG("ltr553_sub_eint_work sensor_data.values[0] =%d\n",sensor_data.values[0]);
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        // let up layer to know
        if (err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data))
        {
            APS_SUB_ERR("call hwmsen_get_interrupt_data light fail = %d\n", err);
        }
    }

    if ((1 << CMC_BIT_PS) & obj->pending_intr)
    {
        // get raw data
        APS_SUB_ERR("fwq ps INT\n");
        if (err = ltr553_sub_read_data_ps(obj->client, &obj->ps))
        {
            APS_SUB_ERR("ltr553 sub read ps data: %d\n", err);;
        }
        /*added by fully for overflow.*/
        if(obj->ps & 0x8000)
        {
            sensor_data.values[0] = 1;
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            if(err = hwmsen_get_interrupt_data(ID_PROXIMITY_SUB, &sensor_data))
            {
                APS_SUB_ERR("call hwmsen_get_interrupt_data proximity fail = %d\n", err);
            }
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
            mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
            mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

            return;
        }
        /*the end added by fully for overflow.*/

        //map and store data to hwm_sensor_data
        while(-1 == ltr553_sub_get_ps_value(obj, obj->ps))
        {
            ltr553_sub_read_data_ps(obj->client, &obj->ps);
            msleep(50);
            APS_SUB_ERR("ltr553 sub read ps data delay\n");;
        }
        sensor_data.values[0] = ltr553_sub_get_ps_value(obj, obj->ps);

        if(sensor_data.values[0] == 0)
        {
            hwmsen_write_byte(obj->client,0x90,0xff);
            hwmsen_write_byte(obj->client,0x91,0x07);

            hwmsen_write_byte(obj->client,0x92,(u8)(atomic_read(&obj->ps_thd_val_low)) & 0xff);              //yaoyaoqin:low threshold for faraway interrupt only
            hwmsen_write_byte(obj->client,0x93,(u8)((atomic_read(&obj->ps_thd_val_low))>>8) & 0X07);
            //wake_unlock(&ps_wake_lock);
        }
        else if(sensor_data.values[0] == 1)
        {
#ifdef GN_MTK_BSP_PS_SUB_DYNAMIC_CALI
            if(obj->ps  > 20 && obj->ps < (dynamic_calibrate - 50))
            {
                if(obj->ps < 50)
                {
                    hw->ps_threshold_high= obj->ps + 50;
                    hw->ps_threshold_low = obj->ps + 35;
                }
                else if(obj->ps < 100)
                {
                    hw->ps_threshold_high = obj->ps + 80;
                    hw->ps_threshold_low = obj->ps + 60;
                }
                else if(obj->ps < 200)
                {
                    hw->ps_threshold_high = obj->ps + 90;
                    hw->ps_threshold_low = obj->ps + 70;
                }
                else if(obj->ps < 300)
                {
                    hw->ps_threshold_high = obj->ps + 100;
                    hw->ps_threshold_low = obj->ps + 80;
                }
                else if(obj->ps < 400)
                {
                    hw->ps_threshold_high = obj->ps + 120;
                    hw->ps_threshold_low = obj->ps + 100;
                }
                else if(obj->ps < 500)
                {
                    hw->ps_threshold_high = obj->ps + 140;
                    hw->ps_threshold_low = obj->ps + 120;
                }
                else if(obj->ps < 600)
                {
                    hw->ps_threshold_high = obj->ps + 160;
                    hw->ps_threshold_low = obj->ps + 140;
                }
                else
                {
                    hw->ps_threshold_high = 50;
                    hw->ps_threshold_low = 35;
                }
                dynamic_calibrate = obj->ps;

            }

            if(obj->ps  > 50)
            {
                temp_noise = obj->ps - 50;
            }
            else
            {
                temp_noise = 0;
            }
#endif

	APS_SUB_LOG("ltr553_sub_eint_work ps_thd_val_high: %d,ps_thd_val: %d\n", atomic_read(&obj->ps_thd_val_high),obj->ps_thd_val);
            //wake_lock_timeout(&ps_wake_lock,ps_wakeup_timeout*HZ);
            hwmsen_write_byte(obj->client,0x90,(u8)(atomic_read(&obj->ps_thd_val_high) & 0xff));             //yaoyaoqin:high threshold for close interrupt only
            hwmsen_write_byte(obj->client,0x91,(u8)((atomic_read(&obj->ps_thd_val_high)>>8) & 0X07));

            hwmsen_write_byte(obj->client,0x92,temp_noise & 0xff);
            hwmsen_write_byte(obj->client,0x93,(temp_noise>>8) & 0x07);
#ifdef LOW_TEMPERATURE
        if(reset_flag_low_temperature==0)
        {
                hwmsen_write_byte(obj->client,0x92,(u8)(atomic_read(&obj->ps_thd_val_low)) & 0xff);  
                hwmsen_write_byte(obj->client,0x93,(u8)((atomic_read(&obj->ps_thd_val_low))>>8) & 0X07);				
        }
#endif

        }

        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        //let up layer to know
        APS_SUB_ERR("ltr553 sub read ps data = %d \n",sensor_data.values[0]);
        if(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))
        {
            APS_SUB_ERR("call hwmsen_get_interrupt_data proximity fail = %d\n", err);
        }
    }
#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

}

int ltr553_sub_setup_eint(struct i2c_client *client)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
#if defined(CONFIG_OF)
	u32 ints[2] = {0, 0};
#endif
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;

    APS_SUB_FUN();
    g_ltr553_sub_ptr = obj;
/*
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
*/


	
		alspsSubPltFmDev = get_alsps_sub_platformdev();
	/* gpio setting */
		pinctrl = devm_pinctrl_get(&alspsSubPltFmDev->dev);
		if (IS_ERR(pinctrl)) {
			ret = PTR_ERR(pinctrl);
			APS_SUB_ERR("Cannot find alsps pinctrl!\n");
		}
		pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
		if (IS_ERR(pins_default)) {
			ret = PTR_ERR(pins_default);
			APS_SUB_ERR("Cannot find alsps pinctrl default!\n");
	
		}
	
		pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
		if (IS_ERR(pins_cfg)) {
			ret = PTR_ERR(pins_cfg);
			APS_SUB_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	
		}
         
	/* eint request */

#if defined(CONFIG_OF)
	if ( ltr553_sub_obj->irq_node)
	{
		of_property_read_u32_array( ltr553_sub_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		 pinctrl_select_state(pinctrl, pins_cfg);
		APS_SUB_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		 ltr553_sub_obj->irq = irq_of_parse_and_map( ltr553_sub_obj->irq_node, 0);
	
		APS_SUB_LOG(" ltr553_sub_obj->irq = %d\n",  ltr553_sub_obj->irq);
		if (! ltr553_sub_obj->irq)
		{
			APS_SUB_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		
		if(request_irq( ltr553_sub_obj->irq,  ltr553_sub_eint_handler, IRQF_TRIGGER_NONE, "ALSSUB-eint", NULL)) {
			APS_SUB_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		
		enable_irq( ltr553_sub_obj->irq);
	}
	else
	{
		APS_SUB_ERR("null irq node!!\n");
		return -EINVAL;
	}

#elif defined( CUST_EINT_ALS_TYPE)
    mt_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE);
    mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, ltr553_sub_eint_func, 0);
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, ltr553_sub_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

#endif /*CUST_EINT_ALS_TYPE*/

    return 0;
}
#endif /* GN_MTK_BSP_ALSPS_INTERRUPT_MODE */
/*----------------------------------------------------------------------------*/
static int ltr553_sub_init_client(struct i2c_client *client)
{
//    struct ltr553_priv *obj = i2c_get_clientdata(client);
    int err=0;
    APS_SUB_LOG("ltr553_sub_init_client.........\r\n");

#ifdef GN_MTK_BSP_ALSPS_SUB_INTERRUPT_MODE
    if((err = ltr553_sub_setup_eint(client)))
    {
        APS_SUB_ERR("setup eint: %d\n", err);
        return err;
    }
#endif

    if((err = ltr553_sub_init_device(client)))
    {
        APS_SUB_ERR("init dev: %d\n", err);
        return err;
    }
    return err;
}
/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr553_sub_show_config(struct device_driver *ddri, char *buf)
{
    ssize_t res;

    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
                   atomic_read(&ltr553_sub_obj->i2c_retry), atomic_read(&ltr553_sub_obj->als_debounce),
                   atomic_read(&ltr553_sub_obj->ps_mask), ltr553_sub_obj->ps_thd_val, atomic_read(&ltr553_sub_obj->ps_debounce));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_store_config(struct device_driver *ddri, char *buf, size_t count)
{
    int retry, als_deb, ps_deb, mask, thres;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
    {
        atomic_set(&ltr553_sub_obj->i2c_retry, retry);
        atomic_set(&ltr553_sub_obj->als_debounce, als_deb);
        atomic_set(&ltr553_sub_obj->ps_mask, mask);
        ltr553_sub_obj->ps_thd_val= thres;
        atomic_set(&ltr553_sub_obj->ps_debounce, ps_deb);
    }
    else
    {
        APS_SUB_ERR("invalid content: '%s', length = %d\n", buf, count);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_show_trace(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr553_sub_obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_store_trace(struct device_driver *ddri, char *buf, size_t count)
{
    int trace;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&ltr553_sub_obj->trace, trace);
    }
    else
    {
        APS_SUB_ERR("invalid content: '%s', length = %d\n", buf, count);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_show_als(struct device_driver *ddri, char *buf)
{
    int res;
    u8 dat = 0;

    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_obj is null!!\n");
        return 0;
    }
    // if(res = ltr553_read_data(ltr553_obj->client, &ltr553_obj->als))
    if(res = ltr553_sub_read_data_als(ltr553_sub_obj->client, &ltr553_sub_obj->als))
    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
#if 0
    else
    {
        // dat = ltr553_obj->als & 0x3f;
        dat = ltr553_obj->als;
        return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
    }
#endif
    while(-1 == ltr553_sub_get_als_value(ltr553_sub_obj,ltr553_sub_obj->als))
    {
        ltr553_sub_read_data_als(ltr553_sub_obj->client,&ltr553_sub_obj->als);
        msleep(50);
    }
    dat = ltr553_sub_get_als_value(ltr553_sub_obj,ltr553_sub_obj->als);

    return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_show_ps(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    u8 dat=0;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_obj is null!!\n");
        return 0;
    }

    // if(res = ltr553_read_data(ltr553_obj->client, &ltr553_obj->ps))
    if(res = ltr553_sub_read_data_ps(ltr553_sub_obj->client, &ltr553_sub_obj->ps))
    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        // dat = ltr553_obj->ps & 0x80;
        dat = ltr553_sub_get_ps_value(ltr553_sub_obj, ltr553_sub_obj->ps);
        //return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
        return snprintf(buf, PAGE_SIZE, "0x%04X\n", ltr553_sub_obj->ps);
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    if(ltr553_sub_obj->hw)
    {

        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
                        ltr553_sub_obj->hw->i2c_num, ltr553_sub_obj->hw->power_id, ltr553_sub_obj->hw->power_vol);

    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }

#ifdef MT6516
    len += snprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
                    CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

    len += snprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n", GPIO_ALS_EINT_PIN,
                    mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN),
                    mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
#endif

    len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr553_sub_obj->als_suspend), atomic_read(&ltr553_sub_obj->ps_suspend));

    return len;
}

#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr553_sub_priv *obj, const char* buf, size_t count,
                             u32 data[], int len)
{
    int idx = 0;
    char *cur = (char*)buf, *end = (char*)(buf+count);

    while(idx < len)
    {
        while((cur < end) && IS_SPACE(*cur))
        {
            cur++;
        }

        if(1 != sscanf(cur, "%d", &data[idx]))
        {
            break;
        }

        idx++;
        while((cur < end) && !IS_SPACE(*cur))
        {
            cur++;
        }
    }
    return idx;
}
/*----------------------------------------------------------------------------*/


static ssize_t ltr553_sub_show_reg(struct device_driver *ddri, char *buf)
{
    int i = 0;
    u8 bufdata;
    int count  = 0;

    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    for(i = 0; i < 31 ; i++)
    {
        hwmsen_read_byte_sr(ltr553_sub_obj->client,0x80+i,&bufdata);
        count+= sprintf(buf+count,"[%x] = (%x)\n",0x80+i,bufdata);
    }

    return count;
}

static ssize_t ltr553_sub_store_reg(struct device_driver *ddri,char *buf,ssize_t count)
{

    u32 data[2];
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null\n");
        return 0;
    }
    /*else if(2 != sscanf(buf,"%d %d",&addr,&data))*/
    else if(2 != read_int_from_buf(ltr553_sub_obj,buf,count,data,2))
    {
        APS_SUB_ERR("invalid format:%s\n",buf);
        return 0;
    }

    hwmsen_write_byte(ltr553_sub_obj->client,data[0],data[1]);

    return count;
}
/*----------------------------------------------------------------------------*/


static ssize_t ltr553_sub_show_alslv(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr553_sub_obj->als_level_num; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr553_sub_obj->hw->als_level[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_store_alslv(struct device_driver *ddri, char *buf, size_t count)
{
    struct ltr553_sub_priv *obj;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }
    else if(!strcmp(buf, "def"))
    {
        memcpy(ltr553_sub_obj->als_level, ltr553_sub_obj->hw->als_level, sizeof(ltr553_sub_obj->als_level));
    }
    else if(ltr553_sub_obj->als_level_num != read_int_from_buf(ltr553_sub_obj, buf, count,
            ltr553_sub_obj->hw->als_level, ltr553_sub_obj->als_level_num))
    {
        APS_SUB_ERR("invalid format: '%s'\n", buf);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_show_alsval(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr553_sub_obj->als_value_num; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr553_sub_obj->hw->als_value[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr553_sub_store_alsval(struct device_driver *ddri, char *buf, size_t count)
{
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }
    else if(!strcmp(buf, "def"))
    {
        memcpy(ltr553_sub_obj->als_value, ltr553_sub_obj->hw->als_value, sizeof(ltr553_sub_obj->als_value));
    }
    else if(ltr553_sub_obj->als_value_num != read_int_from_buf(ltr553_sub_obj, buf, count,
            ltr553_sub_obj->hw->als_value, ltr553_sub_obj->als_value_num))
    {
        APS_SUB_ERR("invalid format: '%s'\n", buf);
    }
    return count;
}

static ssize_t ltr553_sub_show_enable_als(struct device_driver *ddrv,char *buf)
{
    ssize_t len =  0;
    int idx;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }

    if(true == ltr553_sub_obj->als_enable)
    {
        len = sprintf(buf,"%d\n",1);
    }
    else
    {
        len = sprintf(buf,"%d\n",0);
    }
    return len;

}
static  ssize_t ltr553_sub_store_enable_als(struct device_driver *ddrv,char *buf, size_t count)
{
    int enable;
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        return 0;
    }
    if(1 == sscanf(buf,"%d",&enable))
    {
        if(enable)
        {
            ltr553_sub_enable_als(ltr553_sub_obj->client,true);
        }
        else
        {
            ltr553_sub_enable_als(ltr553_sub_obj->client,false);
        }
    }
    else
    {
        APS_SUB_ERR("enable als fail\n");
    }
    return count;
}

#ifdef GN_MTK_BSP_PS_SUB_DYNAMIC_CALI
static ssize_t ltr553_sub_dynamic_calibrate(void)
{
    int ret=0;
    int i=0;
    int data;
    int data_total=0;
    ssize_t len = 0;
    int noise = 0;
    int count = 5;
    int max = 0;
//	struct alsps_hw *hw = get_cust_alsps_hw();
	
    if(!ltr553_sub_obj)
    {
        APS_SUB_ERR("ltr553_sub_obj is null!!\n");
        //len = sprintf(buf, "ltr553_obj is null\n");
        return -1;
    }

    // wait for register to be stable
    if(part_id == ltr553_PART_ID)
    {
        msleep(15);
    }
    else
    {
        msleep(100);
    }

    for (i = 0; i < count; i++)
    {
        // wait for ps value be stable
        if(part_id == ltr553_PART_ID)
        {
            msleep(15);
        }
        else
        {
            msleep(55);
        }

        ret=ltr553_sub_read_data_ps(ltr553_sub_obj->client,&data);
        if (ret < 0)
        {
            i--;
            continue;
        }

        if(data & 0x8000)
        {
            noise = dynamic_calibrate;
            break;
        }

        data_total+=data;

        if (max++ > 100)
        {
            //len = sprintf(buf,"adjust fail\n");
            return len;
        }
    }

    if(noise == dynamic_calibrate)
    {
        ;
    }
    else
    {
        noise=data_total/count;
    }

    dynamic_calibrate = noise;
    if(noise < 50)
    {
        hw->ps_threshold_high = noise + 50;
        hw->ps_threshold_low = noise + 35;
    }
    else if(noise < 100)
    {
        hw->ps_threshold_high = noise + 80;
        hw->ps_threshold_low = noise + 60;
    }
    else if(noise < 200)
    {
        hw->ps_threshold_high = noise + 90;
        hw->ps_threshold_low = noise + 70;
    }
    else if(noise < 300)
    {
        hw->ps_threshold_high = noise + 100;
        hw->ps_threshold_low = noise + 80;
    }
    else if(noise < 400)
    {
        hw->ps_threshold_high = noise + 120;
        hw->ps_threshold_low = noise + 100;
    }
    else if(noise < 500)
    {
        hw->ps_threshold_high = noise + 140;
        hw->ps_threshold_low = noise + 120;
    }
    else if(noise < 600)
    {
        hw->ps_threshold_high = noise + 160;
        hw->ps_threshold_low = noise + 140;
    }
    else
    {
        hw->ps_threshold_high = 55;
        hw->ps_threshold_low = 35;
    }
APS_SUB_ERR("ltr553_sub_dynamic_calibrate High:%d,Low:%d\n",hw->ps_threshold_high,hw->ps_threshold_low);	
    hwmsen_write_byte(ltr553_sub_obj->client, 0x90, (u8)(hw->ps_threshold_high) & 0XFF);
    hwmsen_write_byte(ltr553_sub_obj->client, 0x91, (u8)((hw->ps_threshold_high)>>8) & 0X07);
    hwmsen_write_byte(ltr553_sub_obj->client, 0x92, (u8)(hw->ps_threshold_low) & 0xff);
    hwmsen_write_byte(ltr553_sub_obj->client, 0x93, (u8)((hw->ps_threshold_low)>>8) & 0x07);

//  len = sprintf(buf,"ps_trigger_high: %d, ps_trigger_low: %d\n",
//      ps_trigger_high,ps_trigger_low);
    return 0;
}
#endif



//guomingyi 20140812 add for cat ps chip info start
static ssize_t show_chipInfo(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ltr553_sub_i2c_driver.driver.name);     
}
//guomingyi 20140812 add for cat ps chip info end

static ssize_t ltr553_sub_show_gsensorState(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", GsensorState);     
        return 0;     
}

static ssize_t ltr553_sub_store_gsensorState(struct device_driver *ddri, char *buf, size_t count)
{
    int enable;
	sscanf(buf, "%d", &enable);
    GsensorState = enable;
	printk("The value of GsensorState is %d\n",GsensorState); 
    return count;
}

static ssize_t show_audioChannel(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", AudioChannel);     
}

static ssize_t store_audioChannel(struct device_driver *ddri, char *buf, size_t count)
{
    int enable;
	sscanf(buf, "%d", &enable);
    AudioChannel = enable;
	printk("The value of AudioChannel is %d\n",AudioChannel); 
    return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,  S_IWUSR | S_IRUGO, ltr553_sub_show_als,    NULL);
static DRIVER_ATTR(ps,   S_IWUSR | S_IRUGO, ltr553_sub_show_ps, NULL);
static DRIVER_ATTR(alslv, S_IWUSR | S_IRUGO, ltr553_sub_show_alslv, ltr553_sub_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, ltr553_sub_show_alsval,ltr553_sub_store_alsval);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, ltr553_sub_show_trace, ltr553_sub_store_trace);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, ltr553_sub_show_status, NULL);
static DRIVER_ATTR(reg,  S_IWUSR | S_IRUGO, ltr553_sub_show_reg, ltr553_sub_store_reg);
static DRIVER_ATTR(enable_als,   0664, ltr553_sub_show_enable_als, ltr553_sub_store_enable_als);
static DRIVER_ATTR(chipinfo,     S_IWUSR | S_IRUGO, show_chipInfo, NULL); //guomingyi 20140812 add for cat ps chip info
static DRIVER_ATTR(gsensorState,   0664, ltr553_sub_show_gsensorState, ltr553_sub_store_gsensorState);
static DRIVER_ATTR(audioChannel,     0664, show_audioChannel, store_audioChannel); //guomingyi 20140812 add for cat ps chip info
//static DRIVER_ATTR(adjust, S_IWUSR | S_IRUGO, ltr553_ps_adjust, NULL);
/*----------------------------------------------------------------------------*/
static struct device_attribute *ltr553_sub_attr_list[] =
{
    &driver_attr_als,
    &driver_attr_ps,
    &driver_attr_trace,     /*trace log*/
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_reg,
    &driver_attr_enable_als,
    &driver_attr_chipinfo, //guomingyi 20140812 add for cat ps chip info
    //&driver_attr_adjust
    &driver_attr_gsensorState,
    &driver_attr_audioChannel,
};
/*----------------------------------------------------------------------------*/
static int ltr553_sub_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(ltr553_sub_attr_list)/sizeof(ltr553_sub_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, ltr553_sub_attr_list[idx]))
        {
            APS_SUB_ERR("driver_create_file (%s) = %d\n", ltr553_sub_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr553_sub_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(ltr553_sub_attr_list)/sizeof(ltr553_sub_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, ltr553_sub_attr_list[idx]);
    }
    return err;
}
/******************************************************************************
 * Function Configuration
******************************************************************************/
// static int ltr553_get_als_value(struct ltr553_priv *obj, u8 als)
static int ltr553_sub_get_als_value(struct ltr553_sub_priv *obj, int als)
{
    int idx;
    int invalid = 0;
    for(idx = 0; idx < obj->als_level_num; idx++)
    {
        if(als < obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_SUB_ERR("exceed range\n");
        idx = obj->als_value_num - 1;
    }

    if(1 == atomic_read(&obj->als_deb_on))
    {
        unsigned long endt = atomic_read(&obj->als_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->als_deb_on, 0);
        }

        if(1 == atomic_read(&obj->als_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
        {
            APS_SUB_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        }

        return obj->hw->als_value[idx];
    }
    else
    {
        if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
        {
            APS_SUB_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        }
        return -1;
    }
}
/*----------------------------------------------------------------------------*/
int  ltr553_sub_get_ps_value_gesture(struct ltr553_sub_priv *obj, int ps)
{
    int val= -1;
    int invalid = 0;

    APS_SUB_ERR("ltr553_sub_get_ps_value_gesture###:ps=%x,Hight=%x,Low=%x\n", ps,(hw->ps_threshold_high),(hw->ps_threshold_low));

    if (ps > (g_ps_sub_base_value + hw->ps_threshold_high) )
    {
        // bigger value, close
        val = 0;
    }
    else if (ps < (g_ps_sub_base_value + hw->ps_threshold_low))
    {
        // smaller value, far away
        val = 1;
    }
    else
    {
        val = 1;
    }

    if(atomic_read(&obj->ps_suspend))
    {
        invalid = 1;
    }
    else if(1 == atomic_read(&obj->ps_deb_on))
    {
        unsigned long endt = atomic_read(&obj->ps_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->ps_deb_on, 0);
        }

        if (1 == atomic_read(&obj->ps_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
        {
            APS_SUB_DBG("PS: %05d => %05d\n", ps, val);
        }
        return val;

    }
    else
    {
        if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
        {
            APS_SUB_DBG("PS: %05d => %05d (-1)\n", ps, val);
        }
        return -1;
    }

}
int ltr553_sub_get_ps_value(struct ltr553_sub_priv *obj, int ps)
{
    int val= -1;
    int invalid = 0;

    APS_SUB_ERR("ltr553_sub_get_ps_value###:ps=%x,Hight=%x,Low=%x\n", ps,atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));

    if (ps > atomic_read(&obj->ps_thd_val_high))
    {
        // bigger value, close
        val = 0;
    }
    else if (ps < atomic_read(&obj->ps_thd_val_low))
    {
        // smaller value, far away
        val = 1;
    }
    else
    {
        val = 1;
    }

    if(atomic_read(&obj->ps_suspend))
    {
        invalid = 1;
    }
    else if(1 == atomic_read(&obj->ps_deb_on))
    {
        unsigned long endt = atomic_read(&obj->ps_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->ps_deb_on, 0);
        }

        if (1 == atomic_read(&obj->ps_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
        {
            APS_SUB_DBG("PS: %05d => %05d\n", ps, val);
        }
        return val;

    }
    else
    {
        if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
        {
            APS_SUB_DBG("PS: %05d => %05d (-1)\n", ps, val);
        }
        return -1;
    }

}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int ltr553_sub_open(struct inode *inode, struct file *file)
{
    file->private_data = ltr553_sub_i2c_client;

    if (!file->private_data)
    {
        APS_SUB_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr553_sub_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/

#ifdef FEATURE_PS_SUB_CALIBRATION

static void ps_sub_cali_tp_check(void)
{
#if 0
    u8 data, data2, data3;
    int res =0;
    char *product_id = NULL;

    if(g_tp_tpye_checked)
    {
        APS_LOG("ps_tp_check tp already checked \n");
        return;
    }

    product_id = synaptics_get_vendor_info();

    APS_LOG("ps_tp_check product_id = %s \n", product_id);

    if( 0 == memcmp(product_id, "JTOUCH", 6))
    {
        //PS LED Control
        data = (PS_SETTING_LED_PULSE_2<< PS_LED_PULSE_SHIFT) & PS_LED_PULSE_MASK;
        data2 = (PS_SETTING_LED_RATIO_16 << PS_LED_RATIO_SHIFT) & PS_LED_RATIO_MASK;
        data |= data2;
        res = i2c_write_reg(REG_PS_LED,data);

        if(res < 0)
        {
            APS_LOG("i2c_master_send function err in jtouch_white_tp_threshold_reset\n");
        }

        mdelay(50);
    }

    g_tp_tpye_checked = 1;
#endif
}

static void ps_sub_cali_set_threshold(void)
{
    u8 data, data2, data3;
    u16 value_high,value_low;
    int res =0;

//    struct alsps_hw *hw = get_cust_alsps_hw();

    APS_SUB_LOG("ps_cali_set_threshold:g_ps_base_value=%x, hw->ps_threshold_high=%x,hw->ps_threshold_low=%x \n",
            g_ps_sub_base_value, hw->ps_threshold_high, hw->ps_threshold_low);

    value_high= g_ps_sub_base_value + hw->ps_threshold_high;
    value_low= g_ps_sub_base_value + hw->ps_threshold_low;

    if( value_high > 0x7f0)  //2032,ps max value is 2047
    {
            value_high= 0x7f0;//2032
        value_low= 0x75a;//1882
        APS_SUB_LOG("ps_cali_set_threshold: set value_high=0x32,value_low=0x23, please check the phone \n");
    }

    atomic_set(&ltr553_sub_obj->ps_thd_val_high, value_high);
    atomic_set(&ltr553_sub_obj->ps_thd_val_low,  value_low);

    ltr553_sub_ps_set_thres();
}

static void ps_sub_cali_start(void)
{
    int 			err = 0;
    u16             vl_read_ps = 0;
    u16             vl_ps_count = 0;
    u16             vl_ps_sun = 0;
    u16             vl_index = 0;



    APS_SUB_LOG("entry ps_cali_start \n");

    if(NULL == ltr553_sub_obj->client)
    {
        APS_SUB_ERR("ltr553_sub_obj->client == NULL\n");
        return;
    }


    //ps_cali_tp_check();

    // enable ps and backup reg data
    /*
    if((err = enable_ps(ltr501_obj->client, 1)))
    {
        APS_ERR("enable ps fail: %ld\n", err);
        goto exit_handle;
    }
    mdelay(50);
    */


    // read ps
    for(vl_index = 0; vl_index < 4; vl_index++)
    {
     
	 if(err = ltr553_sub_read_data_ps(ltr553_sub_obj->client, &ltr553_sub_obj->ps))
	  {
			APS_SUB_ERR("read data error\n");
	  }
	 
        vl_read_ps = ltr553_sub_obj->ps;

        APS_SUB_LOG("vl_index=%d, vl_read_ps = %d \n",vl_index, vl_read_ps);

        if(vl_index >=2)
        {
            vl_ps_sun += vl_read_ps;

            vl_ps_count ++;
        }

        vl_read_ps = 0;

        mdelay(30);
    }

    g_ps_sub_base_value = (vl_ps_sun/vl_ps_count);
    g_ps_sub_cali_flag = 1;

    APS_SUB_LOG("ps_cali_start:g_ps_base_value=%x \n",g_ps_sub_base_value);


exit_handle:
    APS_SUB_LOG("tag: exit_handle\n");
    /*
    if((err = enable_ps(ltr553_obj->client, 0)))
    {
        APS_ERR("disable ps fail: %d\n", err);
    }
    */

}
#endif

static long ltr553_sub_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
  #ifdef FEATURE_PS_SUB_CALIBRATION
    int ps_sub_cali_data[2] = {0x00};
  #endif

    switch (cmd)
    {
        case ALSPS_SUB_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                if(err = ltr553_sub_enable_ps(obj->client, true))
                {
                    APS_SUB_ERR("enable ps fail: %d\n", err);
                    goto err_out;
                }
                set_bit(CMC_BIT_PS, &obj->enable);
            }
            else
            {
                if(err = ltr553_sub_enable_ps(obj->client, false))
                {
                    APS_SUB_ERR("disable ps fail: %d\n", err);
                    goto err_out;
                }
                clear_bit(CMC_BIT_PS, &obj->enable);
            }
            break;

        case ALSPS_SUB_GET_PS_MODE:
            enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SUB_GET_PS_DATA:
            if(err = ltr553_sub_read_data_ps(obj->client, &obj->ps))
            {
                goto err_out;
            }
            dat = ltr553_sub_get_ps_value(obj, obj->ps);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SUB_GET_PS_RAW_DATA:
            if(err = ltr553_sub_read_data_ps(obj->client, &obj->ps))
            {
                goto err_out;
            }

            dat = obj->ps & 0x80;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SUB_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                if(err = ltr553_sub_enable_als(obj->client, true))
                {
                    APS_SUB_ERR("enable als fail: %d\n", err);
                    goto err_out;
                }
                set_bit(CMC_BIT_ALS, &obj->enable);
            }
            else
            {
                if(err = ltr553_sub_enable_als(obj->client, false))
                {
                    APS_SUB_ERR("disable als fail: %d\n", err);
                    goto err_out;
                }
                clear_bit(CMC_BIT_ALS, &obj->enable);
            }
            break;

        case ALSPS_SUB_GET_ALS_MODE:
            enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SUB_GET_ALS_DATA:
            if(err = ltr553_sub_read_data_als(obj->client, &obj->als))
            {
                goto err_out;
            }
            dat = obj->als;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SUB_GET_ALS_RAW_DATA:
            if(err = ltr553_sub_read_data_als(obj->client, &obj->als))
            {
                goto err_out;
            }

            dat = obj->als;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

#ifdef FEATURE_PS_SUB_CALIBRATION

        case ALSPS_SUB_IOCTL_PS_CALI_START:
            APS_SUB_LOG("case ALSPS_SUB_IOCTL_PS_CALI_START: \n");

            ps_sub_cali_start();

            if (ptr == NULL)
            {
                APS_SUB_ERR("%s ptr == NULL", __FUNCTION__);
                err = -EINVAL;
                break;
            }

            ps_sub_cali_data[0] = g_ps_sub_cali_flag;
            ps_sub_cali_data[1] = g_ps_sub_base_value;

            APS_SUB_LOG("g_ps_sub_cali_flag = %x, g_ps_sub_base_value = %x \n", g_ps_sub_cali_flag, g_ps_sub_base_value);

            if (copy_to_user(ptr, ps_sub_cali_data, sizeof(ps_sub_cali_data)))
            {
                APS_SUB_ERR("%s copy_from_user error", __FUNCTION__);
                err = -EFAULT;
                break;
            }
            break;

        case ALSPS_SUB_IOCTL_PS_SET_CALI:
            APS_SUB_LOG("case ALSPS_SUB_IOCTL_PS_SET_CALI: \n");

            if (ptr == NULL)
            {
                APS_SUB_ERR("%s ptr == NULL", __FUNCTION__);
                err = -EINVAL;
                break;
            }

            if (copy_from_user(&ps_sub_cali_data, ptr, sizeof(ps_sub_cali_data)))
            {
                APS_SUB_ERR("%s copy_from_user error", __FUNCTION__);
                err = -EFAULT;
                break;
            }

            g_ps_sub_cali_flag = ps_sub_cali_data[0];
            g_ps_sub_base_value = ps_sub_cali_data[1];

            if(!g_ps_sub_cali_flag)
            {
                g_ps_sub_base_value = 0x90; // set default base value
                APS_SUB_LOG("not calibration!!! set g_ps_base_value = 0x80 \n");
            }

            APS_SUB_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_sub_cali_flag, g_ps_sub_base_value);

            ps_sub_cali_set_threshold();

            break;

        case ALSPS_SUB_IOCTL_PS_GET_CALI:
            APS_SUB_LOG("case ALSPS_SUB_IOCTL_PS_GET_CALI: \n");

            if (ptr == NULL)
            {
                APS_SUB_ERR("%s ptr == NULL", __FUNCTION__);
                err = -EINVAL;
                break;
            }

            ps_sub_cali_data[0] = g_ps_sub_cali_flag;
            ps_sub_cali_data[1] = g_ps_sub_base_value;

            APS_SUB_LOG("g_ps_sub_cali_flag = %x, g_ps_sub_base_value = %x \n", g_ps_sub_cali_flag, g_ps_sub_base_value);

            if (copy_to_user(ptr, ps_sub_cali_data, sizeof(ps_sub_cali_data)))
            {
                APS_SUB_ERR("%s copy_to_user error", __FUNCTION__);
                err = -EFAULT;
                break;
            }
            break;

        case ALSPS_SUB_IOCTL_PS_CLR_CALI:
            APS_SUB_LOG("case ALSPS_SUB_IOCTL_PS_CLR_CALI: \n");
            g_ps_sub_cali_flag = 0;
            g_ps_sub_base_value = 0;
            ps_sub_cali_set_threshold();
            break;

        case ALSPS_SUB_IOCTL_PS_CALI_RAW_DATA:
            if(err = ltr553_sub_read_data_ps(obj->client, &obj->ps))
            {
                goto err_out;
            }

            dat = obj->ps;

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
#endif
        default:
            APS_SUB_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}
/*----------------------------------------------------------------------------*/
static struct file_operations ltr553_sub_fops =
{
//  .owner = THIS_MODULE,
    .open = ltr553_sub_open,
    .release = ltr553_sub_release,
    .unlocked_ioctl = ltr553_sub_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr553_sub_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps_sub",
    .fops = &ltr553_sub_fops,
};
/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_SUB_FUN();

    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_SUB_ERR("null pointer!!\n");
            return -EINVAL;
        }

        atomic_set(&obj->als_suspend, 1);
        if(err = ltr553_sub_enable_als(client, false))
        {
            APS_SUB_ERR("disable als: %d\n", err);
            return err;
        }
        if(!obj->ps_enable)
            ltr553_sub_power(obj->hw, 0);
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_resume(struct i2c_client *client)
{
    struct ltr553_sub_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_SUB_FUN();

    if(!obj)
    {
        APS_SUB_ERR("null pointer!!\n");
        return -EINVAL;
    }

    if(1 == ltr553_sub_power(obj->hw, 1))
    {
        if(err = ltr553_sub_init_device(client))
        {
            APS_SUB_ERR("initialize client fail!!\n");
            return err;
        }
        if(obj->ps_enable)
        {
            if(err = ltr553_sub_enable_ps(client,true))
            {
                APS_SUB_ERR("enable ps usb fail: %d\n",err);
            }
            return err;
        }
    }
    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))                                                     //yaoyaoqin
    {
        if(err = ltr553_sub_enable_als(client, true))
        {
            APS_SUB_ERR("enable als sub fail: %d\n", err);
        }
    }
    else
    {
        if(err = ltr553_sub_enable_als(client, false))
        {
            APS_SUB_ERR("enable als sub fail: %d\n", err);
        }
    }
    return 0;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)

/*----------------------------------------------------------------------------*/
static void ltr553_sub_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct ltr553_sub_priv *obj = container_of(h, struct ltr553_sub_priv, early_drv);
    int err;
    APS_SUB_FUN();

    if(!obj)
    {
        APS_SUB_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
    if(err = ltr553_sub_enable_als(obj->client, false))
    {
        APS_SUB_ERR("disable als sub fail: %d\n", err);
    }
}
/*----------------------------------------------------------------------------*/
static void ltr553_sub_late_resume(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct ltr553_sub_priv *obj = container_of(h, struct ltr553_sub_priv, early_drv);
    int err;
    APS_SUB_FUN();

    if(!obj)
    {
        APS_SUB_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))
    {
        if(err = ltr553_sub_enable_als(obj->client, true))
        {
            APS_SUB_ERR("enable als fail: %d\n", err);
        }
    }
    else
    {
        if(err = ltr553_sub_enable_als(obj->client, false))
        {
            APS_SUB_ERR("enable als fail: %d\n", err);
        }
    }
}
#endif
int ltr553_sub_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                      void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
   struct hwm_sensor_data* sensor_data;
    struct ltr553_sub_priv *obj = (struct ltr553_sub_priv *)self;

    APS_SUB_LOG("ltr553_sub_ps_operate command:%d\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_SUB_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_SUB_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                if(value)
                {
                    if(err = ltr553_sub_enable_ps(obj->client, true))
                    {
                        APS_SUB_ERR("enable ps sub fail: %d\n", err);
                        return -1;
                    }
                    set_bit(CMC_BIT_PS, &obj->enable);
                }
                else
                {
                    if(err = ltr553_sub_enable_ps(obj->client, false))
                    {
                        APS_SUB_ERR("disable ps fail: %d\n", err);
                        return -1;
                    }
                    clear_bit(CMC_BIT_PS, &obj->enable);
                }
            }
            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                APS_SUB_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (struct hwm_sensor_data *)buff_out;
                if (err = ltr553_sub_read_data_ps(obj->client, &obj->ps))
                {
                    err = -1;
                    break;
                }
                else
                {
                    while(-1 == ltr553_sub_get_ps_value(obj, obj->ps))
                    {
                        ltr553_sub_read_data_ps(obj->client, &obj->ps);
                        msleep(50);
                    }
                    sensor_data->values[0] = ltr553_sub_get_ps_value(obj, obj->ps);
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    APS_SUB_ERR("fwq get ps raw_data = %d, sensor_data =%d\n",obj->ps, sensor_data->values[0]);
                }
            }
            break;
        default:
            APS_SUB_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

int ltr553_sub_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
  struct  hwm_sensor_data* sensor_data;
    struct ltr553_sub_priv *obj = (struct ltr553_sub_priv *)self;

    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_SUB_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_SUB_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                if(value)
                {
                    if(err = ltr553_sub_enable_als(obj->client, true))
                    {
                        APS_SUB_ERR("enable als fail: %d\n", err);
                        return -1;
                    }
                    set_bit(CMC_BIT_ALS, &obj->enable);
                }
                else
                {
                    if(err = ltr553_sub_enable_als(obj->client, false))
                    {
                        APS_SUB_ERR("disable als fail: %d\n", err);
                        return -1;
                    }
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
            }
            break;

        case SENSOR_GET_DATA:
            //APS_LOG("fwq get als data !!!!!!\n");
            if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                APS_SUB_ERR("ltr553_sub_als_operate get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (struct hwm_sensor_data *)buff_out;

                if(err = ltr553_sub_read_data_als(obj->client, &obj->als))
                {
                    err = -1;;
                }
                else
                {
                    while(-1 == ltr553_sub_get_als_value(obj, obj->als))
                    {
                        ltr553_sub_read_data_als(obj->client, &obj->als);
                        msleep(50);
                    }
					#if defined(CONFIG_MTK_AAL_SUPPORT)
					sensor_data->values[0] = obj->als;
					#else
                    sensor_data->values[0] = ltr553_sub_get_als_value(obj, obj->als);
					#endif
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    APS_SUB_ERR("ltr553_als_operate get obj->als = %d, sensor_data =%d\n",obj->als, sensor_data->values[0]);
                }
            }
            break;
        default:
            APS_SUB_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}


/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    APS_SUB_FUN();
    strcpy(info->type, ltr553_sub_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ltr553_sub_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
    int err = 0;

    u8 buf = 0;
    int addr = 1;
    int ret = 0;
#ifdef CONFIG_GN_DEVICE_CHECK
    struct gn_device_info gn_dev_info_light = {0};
    struct gn_device_info gn_dev_info_proximity = {0};
#endif
    APS_SUB_ERR();
    printk("[xucs]ltr553_sub_i2c_probe\n");
    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
    ltr553_sub_obj = obj;

    obj->hw = hw;//get_cust_alsps_hw();
	client->addr=obj->hw->i2c_addr[0];//kangqin add for i2c addr from dts

#ifdef GN_MTK_BSP_ALSPS_SUB_INTERRUPT_MODE
    INIT_DELAYED_WORK(&obj->eint_work, ltr553_sub_eint_work);
#endif
    obj->client = client;
    APS_SUB_LOG("addr = %x\n",obj->client->addr);
    i2c_set_clientdata(client, obj);
    atomic_set(&obj->als_debounce, 1000);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 1000);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->init_done,  0);
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALSSUB-eint");

    obj->ps_enable = 0;
    obj->als_enable = 0;
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
    atomic_set(&obj->i2c_retry, 3);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
    atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
    //pre set ps threshold
    obj->ps_thd_val = obj->hw->ps_threshold;
    //pre set window loss
    obj->als_widow_loss = obj->hw->als_window_loss;

    ltr553_sub_i2c_client = client;

    if(err = hwmsen_read_byte_sr(client, APS_RO_PART_ID, &part_id))
    {
        APS_SUB_ERR("ltr553 sub i2c transfer error, part_id:%d\n", part_id);
        goto exit_init_failed;
    }

    if(err = ltr553_sub_init_client(client))
    {
        goto exit_init_failed;
    }

    if(err = ltr553_sub_enable_als(client, false))
    {
        APS_SUB_ERR("disable als sub fail: %d\n", err);
    }
    if(err = ltr553_sub_enable_ps(client, false))
    {
        APS_SUB_ERR("disable ps sub fail: %d\n", err);
    }

    if(err = misc_register(&ltr553_sub_device))
    {
        APS_SUB_ERR("ltr553_sub_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if(err = ltr553_sub_create_attr(&ltr553_sub_init_info.platform_diver_addr->driver))//(&ltr553_alsps_driver.driver))
    {
        APS_SUB_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_ps.self = ltr553_sub_obj;
    APS_SUB_ERR("obj->hw->polling_mode:%d\n",obj->hw->polling_mode);
    if(1 == obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
    }
    obj_ps.sensor_operate = ltr553_sub_ps_operate;
    if(err = hwmsen_attach(ID_PROXIMITY_SUB, &obj_ps))
    {
        APS_SUB_ERR("attach ID_PROXIMITY_SUB fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_als.self = ltr553_sub_obj;
    ltr553_sub_obj->polling = obj->hw->polling_mode;
    if(1 == obj->hw->polling_mode_als)
    {
        obj_als.polling = 1;
        APS_SUB_ERR("polling mode\n");
    }
    else
    {
        obj_als.polling = 0;//interrupt mode
        APS_SUB_ERR("interrupt mode\n");
    }
    obj_als.sensor_operate = ltr553_sub_als_operate;
    if(err = hwmsen_attach(ID_LIGHT_SUB, &obj_als))
    {
        APS_SUB_ERR("attach ID_LIGHT_SUB fail = %d\n", err);
        goto exit_create_attr_failed;
    }

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB,
                   obj->early_drv.suspend = ltr553_sub_early_suspend,
                                  obj->early_drv.resume = ltr553_sub_late_resume,
                                                 register_early_suspend(&obj->early_drv);
#endif

    ltr553_sub_init_flag = 0;
#ifdef CONFIG_GN_DEVICE_CHECK
    gn_dev_info_light.gn_dev_type = GN_DEVICE_TYPE_LIGHT;
    strcpy(gn_dev_info_light.name, ltr553_sub_DEV_NAME);
    gn_set_device_info(gn_dev_info_light);

    gn_dev_info_proximity.gn_dev_type = GN_DEVICE_TYPE_PROXIMITY;
    strcpy(gn_dev_info_proximity.name, ltr553_sub_DEV_NAME);
    gn_set_device_info(gn_dev_info_proximity);
#endif

    APS_SUB_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&ltr553_sub_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
    kfree(obj);
exit:
    ltr553_sub_i2c_client = NULL;

#if defined(CONFIG_OF)
	enable_irq(obj->irq);
#elif defined(CUST_EINT_ALS_TYPE)
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif


    ltr553_sub_init_flag = -1;
    APS_SUB_ERR("%s: err = %d\n", __func__, err);
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr553_sub_i2c_remove(struct i2c_client *client)
{
    int err;

    if(err = ltr553_sub_delete_attr(&ltr553_sub_init_info.platform_diver_addr->driver))//(&ltr553_i2c_driver.driver))
    {
        APS_SUB_ERR("ltr553_delete_attr fail: %d\n", err);
    }

    if(err = misc_deregister(&ltr553_sub_device))
    {
        APS_SUB_ERR("misc_deregister fail: %d\n", err);
    }

    ltr553_sub_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}
/*----------------------------------------------------------------------------*/
#if 1
int ltr553_sub_get_ps_status(void)
{
    int ps_status = 0;
	int err = 0;
    int clear_flag = 0;
    int count = 20; 
	u8 buffer=0;
    struct ltr553_sub_priv *obj = (struct ltr553_sub_priv *)ltr553_sub_obj;

    APS_SUB_LOG("%s\n", __func__);
	
    // Enable PS
    if (!test_bit(CMC_BIT_PS, &obj->enable))
    {
        if((err = ltr553_sub_enable_ps(obj->client, true)))
        {
            APS_SUB_LOG("enable ps fail: %d\n", err); 
            return -1;
        }
        set_bit(CMC_BIT_PS, &obj->enable);

        clear_flag = 1;
    }

    // Wait ps stable
    do 
    {     
        err = hwmsen_read_byte_sr(ltr553_sub_obj->client,APS_RO_ALS_PS_STATUS,&buffer);
		APS_SUB_LOG("buffer>>>%d\n",buffer);

        if (buffer & 0x02)
            break;
        
        mdelay(5);
        count --;
    } while(count > 0);
    
    // Get PS status
    ltr553_sub_read_data_ps(obj->client, &obj->ps);
    ps_status = ltr553_sub_get_ps_value_gesture(obj, obj->ps);	

    // disable PS 
    if (clear_flag)
    {
        if((err = ltr553_sub_enable_ps(obj->client, false)))
        {
            APS_SUB_LOG("disable ps fail: %d\n", err); 
            return -1;
        }
        clear_bit(CMC_BIT_PS, &obj->enable);
    }

    APS_SUB_LOG("ps_status %d, clear_flag=%d\n", ps_status, clear_flag);

    return ps_status;
}
EXPORT_SYMBOL(ltr553_sub_get_ps_status);

#endif


/*----------------------------------------------------------------------------*/
/*
static int ltr553_probe(struct platform_device *pdev)
{
 //   struct alsps_hw *hw = get_cust_alsps_hw();
    APS_FUN();

    ltr553_power(hw,1);
    if(i2c_add_driver(&ltr553_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }

    if(-1 == ltr553_init_flag)
    {
        return -1;
    }

    return 0;
}

static int ltr553_remove(struct platform_device *pdev)
{
  //  struct alsps_hw *hw = get_cust_alsps_hw();
    APS_FUN();
    ltr553_power(hw, 0);
    i2c_del_driver(&ltr553_i2c_driver);
    return 0;
}

static struct platform_driver ltr553_alsps_driver =
{
    .probe  = ltr553_probe,
    .remove = ltr553_remove,
    .driver = {
        .name   = "als_ps",
//      .owner  = THIS_MODULE,
    }
};
*/

static int __init ltr553_sub_init(void)
{
   // struct alsps_hw *hw = get_cust_alsps_hw();
       const char *name = "mediatek,ltr553sub";
   // hw =   get_alsps_dts_func(name, hw);
	//if (!hw)
	printk("[xucs]ltr553_sub_init\n");
	hw =   get_alsps_sub_dts_func(name, hw);
#ifdef CONFIG_MTK_LEGACY
    APS_SUB_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
    //wake_lock_init(&ps_wake_lock,WAKE_LOCK_SUSPEND,"ps module");
struct i2c_board_info i2c_ltr553= { I2C_BOARD_INFO(ltr553_sub_DEV_NAME, (ltr553_I2C_SLAVE_ADDR))};
	i2c_register_board_info(hw->i2c_num, &i2c_ltr553, 1);  //xiaoqian, 20120412, add for alsps
#endif
   #if 1
   alsps_sub_driver_add(&ltr553_sub_init_info);
   #else
    if (platform_driver_register(&ltr553_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
#endif
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr553_sub_exit(void)
{
    APS_SUB_FUN();
	#if 0
    platform_driver_unregister(&ltr553_alsps_driver);
	#endif
    //wake_lock_destroy(&ps_wake_lock);
}
/*----------------------------------------------------------------------------*/
module_init(ltr553_sub_init);
module_exit(ltr553_sub_exit);
/*----------------------------------------------------------------------------*/
MODULE_DESCRIPTION("ltr553 sub light sensor & p sensor driver");
MODULE_LICENSE("GPL");
