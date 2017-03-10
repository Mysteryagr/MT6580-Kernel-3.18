/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
//#include "cust_gpio_usage.h"
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
//#include <asm/uaccess.h>

#include <mt_boot_common.h>

#include <linux/jiffies.h>
//#include <pmic_drv.h>
#include "tpd.h"

#include "synaptics_dsx_i2c.h"
#include "synaptics_dsx.h"
#include "tpd_custom_synaptics.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#include <linux/jiffies.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#define DRIVER_NAME "mtk-tpd" 
//"synaptics_dsx_i2c"
#define INPUT_PHYS_NAME "mtk-tpd/input0" 
//"synaptics_dsx_i2c/input0"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif
#define WAKEUP_GESTURE true

//#define NO_0D_WHILE_2D
/*
#define REPORT_2D_Z

#define REPORT_2D_W
*/
#define F12_DATA_15_WORKAROUND

//#define TINNO_RATATE_180

/*
#define IGNORE_FN_INIT_FAILURE
*/

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 1000 /* ms */
#define SYN_I2C_RETRY_TIMES 3
#define MAX_F11_TOUCH_WIDTH 15

#define CHECK_STATUS_TIMEOUT_MS 100
#define DELAY_S7300_BOOT_READY  160
#define DELAY_S7300_RESET       20
#define DELAY_S7300_RESET_READY 160//90
#define I2C_DMA_LIMIT 252

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)
#define TINNO_DEVICE_INFO
#if 0
#define TP_DBG(fmt, arg...) \
	printk("[CTP-synaptics] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
#else
#define TP_DBG(fmt, arg...) do {} while (0)
#endif
#ifdef FTM_UPDATE_FIRMWARE
#define SYNAPTICS_IOCTLID	0xE0
#define SYNAPTICS_IOCTL_FW_UPDATE  _IOWR(SYNAPTICS_IOCTLID, 1, int)
#define SYNAPTICS_IOCTL_TP_UPGRADE_SET_BIN_BUF  _IOWR(SYNAPTICS_IOCTLID, 2, int)
#define SYNAPTICS_IOCTL_TP_UPGRADE_SET_BIN_LEN  _IOWR(SYNAPTICS_IOCTLID, 3, int)
#define SYNAPTICS_IOCTL_GET_UPDATE_PROGREE  _IOWR(SYNAPTICS_IOCTLID, 4, int)
static int factory_update_bin_size=0;
static uint8_t* factory_update_bin = NULL;
static int in_bootloader_mode=0;
int ftm_force_update=0;
#endif

DEFINE_MUTEX(synaptics_i2c_access);

//yaohua.li start
static struct input_dev *SY_key_dev;
//#define KEY_SY_SENSOR 250
#define KEY_SY_SENSOR 251
extern int bEnTGesture;
extern u8 gTGesture;   // Modified by zhangxian
//u8 gTGesture ='c';
extern char Tg_buf[16];
extern int enable_key ;
u8 bTPGesturetemp =0;
//extern int bEnTGesture;
static struct point {
	int x;
	int raw_x;
	int y;
	int raw_y;
	int z;
	int status;
	int per_status;
	int down_count;
};
struct point G_point[10];//yaohua.li 
//extern int APDS9930_get_ps_status(void ); // by zhangxian
//yaohua.li end
#define F11_CONTINUOUS_MODE 0x00
#define F11_WAKEUP_GESTURE_MODE 0x04
#define F12_CONTINUOUS_MODE 0x00
#define F12_WAKEUP_GESTURE_MODE 0x02
bool TP_Delicacy =false;
// for MTK
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_halt = 0;
static int tpd_flag = 0;


#ifdef FTM_BUTTON //TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
static u8 boot_mode;

static unsigned int touch_irq = 0;

//[mli,2014-02-19::exlusive isr report & suspend free fingers
DEFINE_MUTEX( rmi4_report_mutex );
//mli]
// for DMA accessing
static u8 *gpwDMABuf_va = NULL;
static u32 gpwDMABuf_pa = NULL;
static u8 *gprDMABuf_va = NULL;
static u32 gprDMABuf_pa = NULL;
struct i2c_msg *read_msg;
static struct device *g_dev = NULL;

// for 0D button
static unsigned char cap_button_codes[] = TPD_0D_BUTTON;
static struct synaptics_dsx_cap_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(cap_button_codes),
	.map = cap_button_codes,
};
// extern function
extern int get_synaptics_versionid( void );
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eintno, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eintno, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
void synaptics_chip_reset(void);
void set_Delicacy_enable(void);
void set_Delicacy_unenable(void);
void set_sleep_unenable(void);
void set_tgesture_enable(void);
void set_tgesture_unenable(void);
int Delicacy_status = 0;
static irqreturn_t tpd_eint_handler(unsigned irq, struct irq_desc *desc);

static int touch_event_handler(void *data);
extern s32 syna_gtp_test_sysfs_init(void);
extern void syna_gtp_test_sysfs_deinit(void);

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28);

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
#endif

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#ifdef TINNO_DEVICE_INFO 
static ssize_t synaptics_rmi4_show_tpvendor(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_show_tpchip(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_show_tpfwversion(struct device *dev,
        struct device_attribute *attr, char *buf);
#endif /*TINNO_DEVICE_INFO*/
#if GTP_CHARGER_SWITCH_SYNA
extern int g_bat_init_flag;
bool upmu_is_chr_det(void);
static void gctp_charger_switch(struct synaptics_rmi4_data *rmi4_data);
#endif 
struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_0_5 {
	union {
		struct {
			/* query 0 */
			unsigned char f11_query0_b0__2:3;
			unsigned char has_query_9:1;
			unsigned char has_query_11:1;
			unsigned char has_query_12:1;
			unsigned char has_query_27:1;
			unsigned char has_query_28:1;

			/* query 1 */
			unsigned char num_of_fingers:3;
			unsigned char has_rel:1;
			unsigned char has_abs:1;
			unsigned char has_gestures:1;
			unsigned char has_sensitibity_adjust:1;
			unsigned char f11_query1_b7:1;

			/* query 2 */
			unsigned char num_of_x_electrodes;

			/* query 3 */
			unsigned char num_of_y_electrodes;

			/* query 4 */
			unsigned char max_electrodes:7;
			unsigned char f11_query4_b7:1;

			/* query 5 */
			unsigned char abs_data_size:2;
			unsigned char has_anchored_finger:1;
			unsigned char has_adj_hyst:1;
			unsigned char has_dribble:1;
			unsigned char has_bending_correction:1;
			unsigned char has_large_object_suppression:1;
			unsigned char has_jitter_filter:1;
		} __packed;
		unsigned char data[6];
	};
};

struct synaptics_rmi4_f11_query_7_8 {
	union {
		struct {
			/* query 7 */
			unsigned char has_single_tap:1;
			unsigned char has_tap_and_hold:1;
			unsigned char has_double_tap:1;
			unsigned char has_early_tap:1;
			unsigned char has_flick:1;
			unsigned char has_press:1;
			unsigned char has_pinch:1;
			unsigned char has_chiral_scroll:1;

			/* query 8 */
			unsigned char has_palm_detect:1;
			unsigned char has_rotate:1;
			unsigned char has_touch_shapes:1;
			unsigned char has_scroll_zones:1;
			unsigned char individual_scroll_zones:1;
			unsigned char has_multi_finger_scroll:1;
			unsigned char has_multi_finger_scroll_edge_motion:1;
			unsigned char has_multi_finger_scroll_inertia:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f11_query_9 {
	union {
		struct {
			unsigned char has_pen:1;
			unsigned char has_proximity:1;
			unsigned char has_large_object_sensitivity:1;
			unsigned char has_suppress_on_large_object_detect:1;
			unsigned char has_two_pen_thresholds:1;
			unsigned char has_contact_geometry:1;
			unsigned char has_pen_hover_discrimination:1;
			unsigned char has_pen_hover_and_edge_filters:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_12 {
	union {
		struct {
			unsigned char has_small_object_detection:1;
			unsigned char has_small_object_detection_tuning:1;
			unsigned char has_8bit_w:1;
			unsigned char has_2d_adjustable_mapping:1;
			unsigned char has_general_information_2:1;
			unsigned char has_physical_properties:1;
			unsigned char has_finger_limit:1;
			unsigned char has_linear_cofficient_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_27 {
	union {
		struct {
			unsigned char f11_query27_b0:1;
			unsigned char has_pen_position_correction:1;
			unsigned char has_pen_jitter_filter_coefficient:1;
			unsigned char has_group_decomposition:1;
			unsigned char has_wakeup_gesture:1;
			unsigned char has_small_finger_correction:1;
			unsigned char has_data_37:1;
			unsigned char f11_query27_b7:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_ctrl_6_9 {
	union {
		struct {
			unsigned char sensor_max_x_pos_7_0;
			unsigned char sensor_max_x_pos_11_8:4;
			unsigned char f11_ctrl7_b4__7:4;
			unsigned char sensor_max_y_pos_7_0;
			unsigned char sensor_max_y_pos_11_8:4;
			unsigned char f11_ctrl9_b4__7:4;
		} __packed;
		unsigned char data[4];
	};
};

struct synaptics_rmi4_f11_data_1_5 {
	union {
		struct {
			unsigned char x_position_11_4;
			unsigned char y_position_11_4;
			unsigned char x_position_3_0:4;
			unsigned char y_position_3_0:4;
			unsigned char wx:4;
			unsigned char wy:4;
			unsigned char z;
		} __packed;
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};

struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
#ifdef REPORT_2D_Z
	unsigned char z;
#endif
#ifdef REPORT_2D_W
	unsigned char wx;
	unsigned char wy;
#endif
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char button_int_enable;
	unsigned char multi_button;
	unsigned char *txrx_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char max_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fhandler {
	struct synaptics_rmi4_exp_fn *exp_fn;
	bool insert;
	bool remove;
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

static struct device_attribute attrs[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
	__ATTR(full_pm_cycle, 0664,
			synaptics_rmi4_full_pm_cycle_show,
			synaptics_rmi4_full_pm_cycle_store),
#endif
	__ATTR(reset, 0664,
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, S_IRUGO,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, S_IRUGO,
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, S_IRUGO,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, 0664,
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(suspend, 0664,
			synaptics_rmi4_show_error,
			synaptics_rmi4_suspend_store),
	__ATTR(wake_gesture,0664,
			synaptics_rmi4_wake_gesture_show,
			synaptics_rmi4_wake_gesture_store),
    //Added by zhangxian 20150119show tp information
#ifdef TINNO_DEVICE_INFO    
    __ATTR(tpvendor, S_IRUGO,
    synaptics_rmi4_show_tpvendor,
    synaptics_rmi4_store_error),
    __ATTR(tpchip, S_IRUGO,
    synaptics_rmi4_show_tpchip,
    synaptics_rmi4_store_error),
    __ATTR(tpfwversion, S_IRUGO,
    synaptics_rmi4_show_tpfwversion,
    synaptics_rmi4_store_error),
#endif /*TINNO_DEVICE_INFO*/    
};

#ifdef TINNO_DEVICE_INFO
extern char * synaptics_get_vendor_info(void);
extern int synaptics_get_fw_version(void);

static ssize_t synaptics_rmi4_show_tpvendor(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	char *product_info = synaptics_get_vendor_info();
	if(product_info)
	{
		if( 0 == memcmp(product_info, "DS4 R3.9.6", 10))
		{
			sprintf(buf, "%s\n", "TDI");
		}
		else
		{
			sprintf(buf, "%s\n", product_info);
		}
	}
	else
	{
		sprintf(buf, "%s\n", "UNKNOWN");
	}
}


static ssize_t synaptics_rmi4_show_tpchip(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", "synaptics_tpd");
}

static ssize_t synaptics_rmi4_show_tpfwversion(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        int fw_version = 0;
        fw_version=synaptics_get_fw_version();

        return snprintf(buf, PAGE_SIZE, "%d\n",fw_version);
}
#endif /*TINNO_DEVICE_INFO*/

#if GTP_CHARGER_SWITCH_SYNA
#define CHARGERBIT (1<<5)
static void gctp_charger_switch(struct synaptics_rmi4_data *rmi4_data)
{
        u32 chr_status = 0;
        u64 cfg_timestamp = 0;
        int retval;
        unsigned char device_ctrl;
        
//        TP_DBG("line = %d %s:g_bat_init_flag %d,rmi4_data->f01_ctrl_base_addr=%x\n",__LINE__,__func__,g_bat_init_flag,rmi4_data->f01_ctrl_base_addr);	
//        TP_DBG("rmi4_data->f01_data_base_addr=%x\n",rmi4_data->f01_data_base_addr);	
        if (!g_bat_init_flag) return;

        retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
	TP_DBG("%s: Failed to read device status, error = %d\n",__func__, retval);
	return retval;
	}
        chr_status = upmu_is_chr_det();
//        TP_DBG("line = %d %s:chr_status %d,device_ctrl= %x\n",__LINE__,__func__,chr_status,device_ctrl);
        if(chr_status)
        {
                device_ctrl |= CHARGERBIT;          
        }
        else
        {
                device_ctrl &= ~CHARGERBIT ;          
        }
//        TP_DBG("line = %d %s:device_ctrl= %x\n",__LINE__,__func__,device_ctrl);        
        retval =  synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
        if (retval < 0) {
        TP_DBG("%s: Failed to read device status, error = %d\n",__func__, retval);
        return;
        }		
        
        
}
#endif 
#define SY_IOCTLID				0xC0
#define IOCTL_I2C_SY_ID				_IOR(SY_IOCTLID,  1, int)
#define IOCTL_I2C_SY_IMG_ID			_IOR(SY_IOCTLID,  2, int)
#define IOCTL_I2C_SY_UPDATE			_IO(SY_IOCTLID,  3)
//yaohua.li
struct synaptics_rmi4_data *g_pts = NULL;
extern int get_synaptics_versionid( void );
extern int get_synaptics_image_versionid( void );
extern int update_FW(void);
#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

	return count;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (list_empty(&rmi->support_fn_list))
		return -ENODEV;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_i2c_write(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_resume(dev);
	else
		return -EINVAL;

	return count;
}

//[ gesture sys fucntion
static ssize_t synaptics_rmi4_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->enable_wakeup_gesture);
}

static ssize_t synaptics_rmi4_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		rmi4_data->enable_wakeup_gesture = input;

	return count;
}
//]
 /**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned int address)
{
	int retval = 0;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;
	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 1; retry <SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				TP_DBG(
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(5);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return (retval == PAGE_SELECT_LEN) ? retval : -EIO;
}

#if 0



int tpd_i2c_read_data(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval=0;
	u8 retry = 0;
	u8 *pData = data;
	int tmp_addr = addr;
	int left_len = length;
	//mutex_lock(&(ts->io_ctrl_mutex));

	struct i2c_client *client = rmi4_data->i2c_client;
	
	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		TP_DBG("tpd_set_page fail, retval = %d\n", retval);
		retval = -EIO;
		goto exit;
	}
	u16 old_flag = client->ext_flag;

	client->addr = client->addr & I2C_MASK_FLAG ;
	client->ext_flag =client->ext_flag | I2C_WR_FLAG | I2C_RS_FLAG | I2C_ENEXT_FLAG;
	
	while (left_len > 0) {
		pData[0] = tmp_addr;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			if (left_len > 8) {
					retval = i2c_master_send(client, pData, (8 << 8 | 1));
				
		} else {
				retval = i2c_master_send(client, pData, (left_len << 8 | 1));
			}
			if (retval < 0) {
				TP_DBG(KERN_ERR"xxxx focal reads data error!! xxxx\n");
			}
			
			if (retval > 0) {
				break;
			} else {
				TP_DBG("%s: I2C retry %d\n", __func__, retry + 1);
				msleep(20);
			}
		}
		left_len -= 8;
		pData += 8;
		tmp_addr += 8;
	}
	client->ext_flag = old_flag;
 #ifdef MT6577
	client->addr = client->addr & I2C_MASK_FLAG;
 #endif

exit:
	//mutex_unlock(&(ts->io_ctrl_mutex));
	return retval;
}
EXPORT_SYMBOL(tpd_i2c_read_data);



int tpd_i2c_write_data(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)

{
	int retval=0;
	u8 retry = 0;
	u8 *pData = data;
	u8 buf[5] = {0};
	int tmp_addr = addr;
	int left_len = length;

	struct i2c_client *client = rmi4_data->i2c_client;
	
	//mutex_lock(&(ts->io_ctrl_mutex));
	
	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		TP_DBG("tpd_set_page fail, retval = %d\n", retval);
		retval = -EIO;
		goto exit;
	}

	while (left_len > 0) {	
		buf[0] = tmp_addr;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			if (left_len > 4) {	
					memcpy(buf+1, pData, 4);
					retval = i2c_master_send(client, buf, 5);
			} else {
				memcpy(buf+1, pData, left_len);
				retval = i2c_master_send(client, buf, left_len + 1);
			}
			if (retval < 0) {
				TP_DBG(KERN_ERR"xxxxfocal write data error!! xxxx\n");
			}
			if (retval > 0) {
				break;
			} else {
				TP_DBG("%s: I2C retry %d\n", __func__, retry + 1);
				msleep(20);
			}
		}

		left_len -= 4;
		pData += 4;
		tmp_addr += 4;
	}


exit:
	//mutex_unlock(&(ts->io_ctrl_mutex));
	return retval;
}

/*{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
			.ext_flag = 0,
		},
	};

	mutex_lock(&(ts->io_ctrl_mutex));

	retval = tpd_set_page(client, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		TP_DBG(
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		TP_DBG(
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(ts->io_ctrl_mutex));

	return retval;
}*/
EXPORT_SYMBOL(tpd_i2c_write_data);


#endif

#if 0

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval=0;
	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));
	retval=tpd_i2c_read_data(rmi4_data,addr,data,length);
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval=0;
	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));
	retval=tpd_i2c_write_data(rmi4_data,addr,data,length);
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;

}
#else
 /**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	unsigned char *buf_va = NULL;
	int full = length / I2C_DMA_LIMIT;
	int partial = length % I2C_DMA_LIMIT;
	int total;
	int last;
	int ii;
	static int msg_length;

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));
	if(!gprDMABuf_va){
        gprDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &gprDMABuf_pa, GFP_KERNEL);
        if(!gprDMABuf_va){
			TP_DBG("[Error] Allocate DMA I2C Buffer failed!\n");
	  }
        }

	buf_va = gprDMABuf_va;

	if ((full + 2) > msg_length) {
		kfree(read_msg);
		msg_length = full + 2;
		read_msg = kcalloc(msg_length, sizeof(struct i2c_msg), GFP_KERNEL);
	}

	read_msg[0].addr = rmi4_data->i2c_client->addr;
	read_msg[0].flags = 0;
	read_msg[0].len = 1;
	read_msg[0].buf = &buf;
	read_msg[0].timing = 400;

	if (partial) {
		total = full + 1;
		last = partial;
	} else {
		total = full;
		last = I2C_DMA_LIMIT;
	}

	for (ii = 1; ii <= total; ii++) {
		read_msg[ii].addr = rmi4_data->i2c_client->addr;
		read_msg[ii].flags = I2C_M_RD;
		read_msg[ii].len = (ii == total) ? last : I2C_DMA_LIMIT;
		read_msg[ii].buf = gprDMABuf_pa + I2C_DMA_LIMIT * (ii - 1);
		read_msg[ii].ext_flag = (rmi4_data->i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
		read_msg[ii].timing = 400;
	}

	buf = addr & MASK_8BIT;

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, read_msg, (total + 1)) == (total + 1)) {

			retval = length;
			break;
		}
		TP_DBG(
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		TP_DBG(
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

	memcpy(data, buf_va, length);

exit:
        /* if(gprDMABuf_va){ */
        /*         dma_free_coherent(NULL, 4096, gprDMABuf_va, gprDMABuf_pa); */
        /*         gprDMABuf_va = NULL; */
        /*         gprDMABuf_pa = NULL; */
        /* } */
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	unsigned char *buf_va = NULL;
	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	if(!gpwDMABuf_va){
	gpwDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 1024, &gpwDMABuf_pa, GFP_KERNEL);
        if(!gpwDMABuf_va){
        	TP_DBG("[Error] Allocate DMA I2C Buffer failed!\n");
	  }
        }
	buf_va = gpwDMABuf_va;

	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = gpwDMABuf_pa,
			.ext_flag=(rmi4_data->i2c_client->ext_flag|I2C_ENEXT_FLAG|I2C_DMA_FLAG),
			.timing = 400,
		}
	};

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	buf_va[0] = addr & MASK_8BIT;

	memcpy(&buf_va[1],&data[0] , length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		TP_DBG(
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(5);
}

	if (retry == SYN_I2C_RETRY_TIMES) {
		TP_DBG(
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	/* if(gpwDMABuf_va){ */
    /*             dma_free_coherent(NULL, 1024, gpwDMABuf_va, gpwDMABuf_pa); */
    /*             gpwDMABuf_va = NULL; */
    /*             gpwDMABuf_pa = NULL; */
    /*     } */
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

#endif
 /**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
extern struct tpd_device *tpd;
#define PACKET_MODE 
#ifdef PACKET_MODE
extern int ltr553_get_ps_status(void);
#endif
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char finger_status_reg[3];
	unsigned char detected_gestures[5];
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
	struct synaptics_rmi4_f11_data_1_5 data;
	struct synaptics_rmi4_f11_extra_data *extra_data;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;

	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	if (rmi4_data->sensor_sleep && rmi4_data->enable_wakeup_gesture) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_addr + extra_data->data38_offset,
				&detected_gestures,
				sizeof(detected_gestures));
		if (retval < 0)
			return 0;
        TP_DBG("detected_gestures:%x,%x,%x==\n",detected_gestures[0],detected_gestures[1],detected_gestures[2]);
		if (detected_gestures[0]==0x01) {
                                        #ifdef PACKET_MODE
                                        if (ltr553_get_ps_status()){
                                      gTGesture ='u';
                                      input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
                                      input_report_key(SY_key_dev,KEY_SY_SENSOR, 0);
                                      input_sync(SY_key_dev);
}
                                        #else
                                        input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
			input_sync(rmi4_data->input_dev);
                                        #endif
		//	rmi4_data->sensor_sleep = false;
		}
                           else if(detected_gestures[0]==0x40)
                           {
                               switch (detected_gestures[3])
                               {
                                   case 0x6d:
                                      gTGesture ='m';
                                      input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
                                      input_report_key(SY_key_dev,KEY_SY_SENSOR, 0);
                                      input_sync(SY_key_dev);
                                      break;       
                                   case 0x63:                  
                                      gTGesture ='c';
                                      input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
                                      input_report_key(SY_key_dev,KEY_SY_SENSOR, 0);
                                      input_sync(SY_key_dev);	
                                      break;
                               }
                           }
                           else if(detected_gestures[0]==0x08)
                           {
                                  TP_DBG("detected_gestures[0]=%d,gTGesture=%c\n",detected_gestures[0],gTGesture);
                                  gTGesture ='o';
                                  input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
                                  input_report_key(SY_key_dev,KEY_SY_SENSOR, 0);
                                  input_sync(SY_key_dev);	
                                  //input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
                                  //input_sync(rmi4_data->input_dev);
                                  //input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
                                  //input_sync(rmi4_data->input_dev);
                          //        rmi4_data->sensor_sleep = false;
                           }
		return 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return 0;
	mutex_lock(&rmi4_report_mutex);
	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * sizeof(data.data));
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					data_offset,
					data.data,
					sizeof(data.data));
			if (retval < 0) {
				touch_count = 0;
				goto exit;
			}

			x = (data.x_position_11_4 << 4) | data.x_position_3_0;
			y = (data.y_position_11_4 << 4) | data.y_position_3_0;
			wx = data.wx;
			wy = data.wy;

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif

#ifdef FTM_BUTTON //TPD_HAVE_BUTTON
			if (NORMAL_BOOT != boot_mode)
			{   
				tpd_button(x, y, 1);  
			}	
#endif
			TP_DBG(
					"%s: Finger %d:\n"
					"status = 0x%02x\n"
					"x = %d\n"
					"y = %d\n"
					"wx = %d\n"
					"wy = %d\n",
					__func__, finger,
					finger_status,
					x, y, wx, wy);

			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
#ifdef FTM_BUTTON    //TPD_HAVE_BUTTON
		if (NORMAL_BOOT != boot_mode)
		{   
			tpd_button(x, y, 0); 
		}   
#endif
	}

	input_sync(rmi4_data->input_dev);
exit:
	mutex_unlock(&(rmi4_report_mutex));
	return touch_count;
}

 /**
 * synaptics_rmi4_f12_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int icount =0; 
int old_x=0; 
int old_y=0;
static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char finger_Delicacy_status;
	unsigned char size_of_2d_data;
	unsigned char detected_gestures[5]; // byte0 indicate gesture type, byte1~byte4 are gesture paramter
	unsigned short data_addr;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
#ifdef F12_DATA_15_WORKAROUND
	static unsigned char fingers_already_present;
#endif
	TP_DBG("line = %d %s:boot_mode %d,%d\n",__LINE__,__func__,boot_mode,FACTORY_BOOT);
	if(FACTORY_BOOT == boot_mode)
	//if(NORMAL_BOOT != boot_mode)
	{
		rmi4_data->input_dev = tpd->dev;
	}
	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

//yaohua.li start
//STP_INFO("line = %d %s:gesture %x\n",__LINE__,__func__,data_addr + extra_data->data4_offset);
if (rmi4_data->sensor_sleep && rmi4_data->enable_wakeup_gesture) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_addr + extra_data->data4_offset,
				detected_gestures,
				sizeof(detected_gestures));
		
     //   STP_INFO("detected_gestures:%x,%x,%x==\n",detected_gestures[0],detected_gestures[1],detected_gestures[2]);

            if (retval < 0)
                return 0;
            /*if (detected_gestures[0]) { // here is demo only, customer could decode gesture data and do whatever they want
                input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
                input_sync(rmi4_data->input_dev);
                input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
                input_sync(rmi4_data->input_dev);
                rmi4_data->sensor_sleep = false;
            }*/
			
        if(detected_gestures[0]==0x03)
        {
     //       STP_INFO("=====LGC========status==0x03=\n");
            int ps_status = 0;
//	    ps_status = APDS9930_get_ps_status();   by zhangxian
            //if(ps_status)
	    {
	    	gTGesture ='u';
            	input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
            	input_report_key(SY_key_dev,KEY_SY_SENSOR , 0);
            	input_sync(SY_key_dev);
            	//rmi4_data->sensor_sleep = false;
	    }	
        }
       else if(detected_gestures[0]==0x0B)
       {
	       switch (detected_gestures[2])
		{
                TP_DBG("===synaptics_rmi4_f12_abs_report=====ic%d================\n",detected_gestures[2]);
                case 0x63:
                    gTGesture ='c';
                //    STP_INFO("====LGC===tpd_process status0x64==%d\n",detected_gestures[2]);
                    input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
                    input_report_key(SY_key_dev,KEY_SY_SENSOR, 0);
                    input_sync(SY_key_dev);	
                    //rmi4_data->sensor_sleep = false;
                    break;
                 case 0x65: //e
                    gTGesture ='e';
             //       STP_INFO("====LGC===tpd_process status0x6d==%d\n",detected_gestures[2]);
                    input_report_key(SY_key_dev, KEY_SY_SENSOR, 1);
                    input_report_key(SY_key_dev, KEY_SY_SENSOR, 0);
                    input_sync(SY_key_dev);	
                    //rmi4_data->sensor_sleep = false;
                    break;
                case 0x6d:
                    gTGesture ='m';
                 //   STP_INFO("====LGC===tpd_process status0x6d==%d\n",detected_gestures[2]);
                    input_report_key(SY_key_dev, KEY_SY_SENSOR, 1);
                    input_report_key(SY_key_dev, KEY_SY_SENSOR, 0);
                    input_sync(SY_key_dev);
                    //rmi4_data->sensor_sleep = false;
                    break;
                case 0x73:
                    gTGesture ='s';
               //     STP_INFO("====LGC===tpd_process status0x6d==%d\n",detected_gestures[2]);
                    input_report_key(SY_key_dev, KEY_SY_SENSOR, 1);
                    input_report_key(SY_key_dev, KEY_SY_SENSOR, 0);
                    input_sync(SY_key_dev);	
                    //rmi4_data->sensor_sleep = false;
                    break;
                
            }
      }	
       else if(detected_gestures[0]==0x0A)
       {
                switch (detected_gestures[2])
                {
                TP_DBG("===synaptics_rmi4_f12_abs_report=====ic%d================\n",detected_gestures[2]);
                case 0x02: // v
                    gTGesture ='v';
                 //   STP_INFO("====LGC===tpd_process status0x64==%d\n",detected_gestures[2]);
                    input_report_key(SY_key_dev,KEY_SY_SENSOR, 1);
                    input_report_key(SY_key_dev,KEY_SY_SENSOR, 0);
                    input_sync(SY_key_dev);
                    //rmi4_data->sensor_sleep = false;
                    break;                
                }
      }
      //yaohau.li end
		
                
                return 0;
	}


	/* Determine the total number of fingers to process */
	if (extra_data->data15_size) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_addr + extra_data->data15_offset,
				extra_data->data15_data,
				extra_data->data15_size);
		if (retval < 0)
			return 0;

		/* Start checking from the highest bit */
		temp = extra_data->data15_size - 1; /* Highest byte */
		finger = (fingers_to_process - 1) % 8; /* Highest bit */
		do {
			if (extra_data->data15_data[temp] & (1 << finger))
				break;

			if (finger) {
				finger--;
			} else {
				temp--; /* Move to the next lower byte */
				finger = 7;
			}

			fingers_to_process--;
		} while (fingers_to_process);

		TP_DBG(
			"%s: Number of fingers to process = %d\n",
			__func__, fingers_to_process);
	}

#ifdef F12_DATA_15_WORKAROUND
	fingers_to_process = max(fingers_to_process, fingers_already_present);
#endif

	if (!fingers_to_process) {
		synaptics_rmi4_free_fingers(rmi4_data);
		return 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (retval < 0)
		return 0;

	data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;
	mutex_lock(&rmi4_report_mutex);
	for (finger = 0; finger < fingers_to_process; finger++) 
       {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status & MASK_1BIT;
		finger_Delicacy_status = finger_data->object_type_and_status & 0X06;
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif
		if ((!Delicacy_status&&finger_status) | (Delicacy_status &&finger_Delicacy_status)) 
	{
#ifdef F12_DATA_15_WORKAROUND
			fingers_already_present = finger + 1;
#endif
			x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
			y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
			
			#if 0	//zhangdongfang cancel it  for tp calibration 
			if (NORMAL_BOOT != boot_mode)                       
				y = (y*128)/140;
			#endif
			
#ifdef REPORT_2D_W
			wx = finger_data->wx;
			wy = finger_data->wy;
#endif
		    if(Delicacy_status &&(finger_Delicacy_status==6)&&(x>=10&&y>=110)&&(x<=700&&y<=525)) 
		      {
			TP_DBG("==================tpd_down==========Delicacy_status====================finger=%d=x%d=y%d====\n",finger,x,y);
			input_report_abs(rmi4_data->input_dev, ABS_MT_TRACKING_ID, finger);
			input_report_key(rmi4_data->input_dev,BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,ABS_MT_TOUCH_MINOR, min(wx, wy));

#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			}
                        else if(!Delicacy_status&&finger_status)
			{
			input_report_abs(rmi4_data->input_dev, ABS_MT_TRACKING_ID, finger);
			input_report_key(rmi4_data->input_dev,BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			}
			if (NORMAL_BOOT != boot_mode)
			{   
				if(y >1100)
			        {
				  old_x =x;			
				  old_y =y;
			        }
			}
			TP_DBG("line = %d %s: Finger %d:status = 0x%02x x = %d y = %d wx = %d wy = %d\n",__LINE__,__func__, finger,finger_status,x, y, wx, wy);
			touch_count++;
		}
	}
////////////////////////////////
#ifdef FTM_BUTTON //TPD_HAVE_BUTTON 
if (NORMAL_BOOT != boot_mode&&!finger_status)

{           if(old_y >1100)
			{
			tpd_button(old_x, old_y, 1); 
                        input_sync(rmi4_data->input_dev);
                        tpd_button(old_x, old_y, 0);         	          
                        input_sync(rmi4_data->input_dev);
			old_x =old_y =0;
		}
}
#endif
	if (Delicacy_status &&finger_Delicacy_status&&touch_count==0) 
	{
		TP_DBG("==================tpd_up==========Delicacy_status=====finger==%d=======x%d=y%d=\n",finger,x,y);
		int ifnum =0;
	       for(ifnum=0;ifnum<10;ifnum++)
		{
		input_mt_slot(rmi4_data->input_dev,finger);
		input_report_key(rmi4_data->input_dev,BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 0);
	#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
	#endif
	} 
	} 
	else if (touch_count==0) 
	{
		int ifnum =0;
	       for(ifnum=0;ifnum<10;ifnum++)

	{	input_mt_slot(rmi4_data->input_dev,ifnum);
		input_report_key(rmi4_data->input_dev,BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 0);
	#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
	#endif
	}
	} 	
	input_sync(rmi4_data->input_dev);
	mutex_unlock(&rmi4_report_mutex);
	return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to read button data registers\n",
				__func__);
		return;
	}

	data = f1a->button_data_buffer;
	mutex_lock(&rmi4_report_mutex);
	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;

		TP_DBG(
				"%s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			touch_count++;
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				touch_count++;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		touch_count++;
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
#endif
	}

	if (touch_count)
		input_sync(rmi4_data->input_dev);
	mutex_unlock(&rmi4_report_mutex);
	return;
}

 /**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	unsigned char touch_count_2d;

	TP_DBG(
			"%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F12:
		touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,
				fhandler);
		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F1A:
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;
	default:
		break;
	}

	return;
}

 /**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to read interrupt status\n",
				__func__);
		return;
	}

	status.data[0] = data[0];
	if (status.unconfigured && !status.flash_prog) {
		TP_DBG("%s: spontaneous reset detected\n", __func__);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			TP_DBG(
					"%s: Failed to reinit device\n",
					__func__);
		}
		return;
	}

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (!exp_fhandler->insert &&
					!exp_fhandler->remove &&
					(exp_fhandler->exp_fn->attn != NULL))
				exp_fhandler->exp_fn->attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */

static irqreturn_t tpd_eint_handler(unsigned irq, struct irq_desc *desc)
{
        TP_DBG("tpd_eint_handler:zx start\n");
//	TPD_DEBUG_PRINT_INT;
	//disable_irq_nosync(touch_irq);

	tpd_flag=1;
	wake_up_interruptible(&waiter);
		return IRQ_HANDLED;
}

static int touch_event_handler(void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
             TP_DBG("touch_event_handler:zx start\n");
	sched_setscheduler(current, SCHED_RR, &param);
	do{
		set_current_state(TASK_INTERRUPTIBLE);



		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
//		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		mutex_lock(&synaptics_i2c_access);

		if (tpd_halt) {
			mutex_unlock(&synaptics_i2c_access);
		//	enable_irq(touch_irq);
			TPD_DEBUG("return for interrupt after suspend...");
			continue;
		}

		if (!rmi4_data->touch_stopped)
			synaptics_rmi4_sensor_report(rmi4_data);


                          #if GTP_CHARGER_SWITCH_SYNA
                          if (!rmi4_data->touch_stopped)
	                          gctp_charger_switch(rmi4_data);
                          #endif
		 mutex_unlock(&synaptics_i2c_access);
		//enable_irq(touch_irq);
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);


	}while(1);

	return 0;
}
unsigned int synap_tpd_eint_gpio;

 static int tpd_irq_registration(void)
 {
	 struct device_node *node = NULL;
	 int ret = 0;
	 u32 ints[2] = { 0, 0 };
	 tpd_gpio_as_int(GTP_INT_PORT);
 
	 msleep(50);
 
	 TPD_DEBUG("Device Tree Tpd_irq_registration!");
	 //node = of_find_compatible_node(NULL, NULL, "mediatek,cap_touch");
	 //node = of_find_compatible_node(NULL, NULL, "mediatek,touch_panel-eint");
 
	 node = of_find_matching_node(node, touch_of_match);
 
	 if (node)
 
		 {
		 of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		 gpio_set_debounce(ints[0], ints[1]);
 
		 
		 //touch_irq = gpio_to_irq(tpd_int_gpio_number);
		 touch_irq = irq_of_parse_and_map(node, 0);
		 TPD_DEBUG("touch_irq number %d\n", touch_irq);
 
		 ret = request_irq(touch_irq, tpd_eint_handler, IRQF_TRIGGER_FALLING,
					 TPD_DEVICE, NULL);
			 if (ret > 0)
				 TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			 if (of_property_read_u32_index(node , "tpd_eint_gpio", 0, &synap_tpd_eint_gpio))
				 {
				 TPD_DMESG("tpd get  tpd_eint_gpio error .");
			 
				 }
			 disable_irq(touch_irq);

			 
	 }
	 
	 else {
		 TPD_DMESG("tpd request_irq can not find touch eint device node!.");
	 }
 
	 return ret;
 }

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char intr_status[MAX_INTR_REGISTERS];

	if (enable) {


		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_status,
				rmi4_data->num_of_intr_regs);

		if (retval < 0)
			return retval;

		// set up irq
		/*if (!rmi4_data->irq_enabled) {
			mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
			mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_LOW, tpd_eint_handler, 0);
			}*/
		enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		rmi4_data->irq_enabled = true;
	} else {
		if (rmi4_data->irq_enabled) {
			disable_irq(touch_irq);//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
			rmi4_data->irq_enabled = false;
		}
	}

	return retval;
}

static void synaptics_rmi4_set_intr_mask(struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	unsigned char ii;
	unsigned char intr_offset;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	return;
}

static int synaptics_rmi4_f01_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f01_query_base_addr = fd->query_base_addr;
	rmi4_data->f01_ctrl_base_addr = fd->ctrl_base_addr;
	rmi4_data->f01_data_base_addr = fd->data_base_addr;
	rmi4_data->f01_cmd_base_addr = fd->cmd_base_addr;

	return 0;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char offset;
	unsigned char fingers_supported;
	struct synaptics_rmi4_f11_extra_data *extra_data;
	struct synaptics_rmi4_f11_query_0_5 query_0_5;
	struct synaptics_rmi4_f11_query_7_8 query_7_8;
	struct synaptics_rmi4_f11_query_9 query_9;
	struct synaptics_rmi4_f11_query_12 query_12;
	struct synaptics_rmi4_f11_query_27 query_27;
	struct synaptics_rmi4_f11_ctrl_6_9 control_6_9;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	if (!fhandler->extra) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fhandle->extra\n",
				__func__);
		return -ENOMEM;
	}
	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			query_0_5.data,
			sizeof(query_0_5.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if (query_0_5.num_of_fingers <= 4)
		fhandler->num_of_data_points = query_0_5.num_of_fingers + 1;
	else if (query_0_5.num_of_fingers == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + 6,
			control_6_9.data,
			sizeof(control_6_9.data));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
rmi4_data->sensor_max_x = control_6_9.sensor_max_x_pos_7_0 |
			(control_6_9.sensor_max_x_pos_11_8 << 8);
	rmi4_data->sensor_max_y = control_6_9.sensor_max_y_pos_7_0 |
			(control_6_9.sensor_max_y_pos_11_8 << 8);

#ifdef TPD_HAVE_BUTTON
	rmi4_data->sensor_max_y = rmi4_data->sensor_max_y * TPD_DISPLAY_HEIGH_RATIO / TPD_TOUCH_HEIGH_RATIO;
#endif
	TP_DBG(
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	fhandler->data = NULL;

	offset = sizeof(query_0_5.data);

	/* query 6 */
	if (query_0_5.has_rel)
		offset += 1;

	/* queries 7 8 */
	if (query_0_5.has_gestures) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_7_8.data,
				sizeof(query_7_8.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_7_8.data);
	}

	/* query 9 */
	if (query_0_5.has_query_9) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_9.data,
				sizeof(query_9.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_9.data);
	}

	/* query 10 */
	if (query_0_5.has_gestures && query_7_8.has_touch_shapes)
		offset += 1;

	/* query 11 */
	if (query_0_5.has_query_11)
		offset += 1;

	/* query 12 */
	if (query_0_5.has_query_12) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_12.data,
				sizeof(query_12.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_12.data);
	}

	/* query 13 */
	if (query_0_5.has_jitter_filter)
		offset += 1;

	/* query 14 */
	if (query_0_5.has_query_12 && query_12.has_general_information_2)
		offset += 1;

	/* queries 15 16 17 18 19 20 21 22 23 24 25 26*/
	if (query_0_5.has_query_12 && query_12.has_physical_properties)
		offset += 12;

	/* query 27 */
	if (query_0_5.has_query_27) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_27.data,
				sizeof(query_27.data));
		if (retval < 0)
			return retval;

		rmi4_data->f11_wakeup_gesture = query_27.has_wakeup_gesture;
	}

	if (!rmi4_data->f11_wakeup_gesture)
		return retval;

	/* data 0 */
	fingers_supported = fhandler->num_of_data_points;
	offset = (fingers_supported + 3) / 4;

	/* data 1 2 3 4 5 */
	offset += 5 * fingers_supported;

	/* data 6 7 */
	if (query_0_5.has_rel)
		offset += 2 * fingers_supported;

	/* data 8 */
	if (query_0_5.has_gestures && query_7_8.data[0])
		offset += 1;

	/* data 9 */
	if (query_0_5.has_gestures && (query_7_8.data[0] || query_7_8.data[1]))
		offset += 1;

	/* data 10 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_pinch || query_7_8.has_flick))
		offset += 1;

	/* data 11 12 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_flick || query_7_8.has_rotate))
		offset += 2;

	/* data 13 */
	if (query_0_5.has_gestures && query_7_8.has_touch_shapes)
		offset += (fingers_supported + 3) / 4;

	/* data 14 15 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_scroll_zones ||
			query_7_8.has_multi_finger_scroll ||
			query_7_8.has_chiral_scroll))
		offset += 2;

	/* data 16 17 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_scroll_zones &&
			query_7_8.individual_scroll_zones))
		offset += 2;

	/* data 18 19 20 21 22 23 24 25 26 27 */
	if (query_0_5.has_query_9 && query_9.has_contact_geometry)
		offset += 10 * fingers_supported;

	/* data 28 */
	if (query_0_5.has_bending_correction ||
			query_0_5.has_large_object_suppression)
		offset += 1;

	/* data 29 30 31 */
	if (query_0_5.has_query_9 && query_9.has_pen_hover_discrimination)
		offset += 3;

	/* data 32 */
	if (query_0_5.has_query_12 &&
			query_12.has_small_object_detection_tuning)
		offset += 1;

	/* data 33 34 */
	if (query_0_5.has_query_27 && query_27.f11_query27_b0)
		offset += 2;

	/* data 35 */
	if (query_0_5.has_query_12 && query_12.has_8bit_w)
		offset += fingers_supported;

	/* data 36 */
	if (query_0_5.has_bending_correction)
		offset += 1;

	/* data 37 */
	if (query_0_5.has_query_27 && query_27.has_data_37)
		offset += 1;

	/* data 38 */
	if (query_0_5.has_query_27 && query_27.has_wakeup_gesture)
		extra_data->data38_offset = offset;

	return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	int retval;
	static unsigned short ctrl_28_address;

	if (ctrl28)
		ctrl_28_address = ctrl28;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
	if (retval < 0)
		return retval;

	return retval;
}

 /**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_0_offset;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_20_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_27_offset;
	unsigned char ctrl_26_offset;
	unsigned char ctrl_28_offset;
	unsigned char num_of_fingers;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_query_5 query_5;
	struct synaptics_rmi4_f12_query_8 query_8;
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
	struct synaptics_rmi4_f12_ctrl_23 ctrl_23;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	if (!fhandler->extra) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fhandler->extra\n",
				__func__);
		return -ENOMEM;
	}
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5.data,
			sizeof(query_5.data));
	if (retval < 0)
		return retval;
       	ctrl_0_offset = 0;
	ctrl_8_offset = query_5.ctrl0_is_present +
			query_5.ctrl1_is_present +
			query_5.ctrl2_is_present +
			query_5.ctrl3_is_present +
			query_5.ctrl4_is_present +
			query_5.ctrl5_is_present +
			query_5.ctrl6_is_present +
			query_5.ctrl7_is_present;

	ctrl_20_offset = ctrl_8_offset +query_5.ctrl8_is_present +
			query_5.ctrl9_is_present +
			query_5.ctrl10_is_present +
			query_5.ctrl11_is_present +
			query_5.ctrl12_is_present +
			query_5.ctrl13_is_present +
			query_5.ctrl14_is_present +
			query_5.ctrl15_is_present +
			query_5.ctrl16_is_present +
			query_5.ctrl17_is_present +
			query_5.ctrl18_is_present +
			query_5.ctrl19_is_present;

	ctrl_23_offset = ctrl_20_offset +query_5.ctrl20_is_present + query_5.ctrl21_is_present +query_5.ctrl22_is_present;
	ctrl_26_offset = ctrl_23_offset +query_5.ctrl23_is_present +query_5.ctrl24_is_present +	query_5.ctrl25_is_present;
	ctrl_27_offset = ctrl_23_offset +query_5.ctrl23_is_present +query_5.ctrl24_is_present +	query_5.ctrl25_is_present +query_5.ctrl26_is_present;
	ctrl_28_offset = ctrl_27_offset + query_5.ctrl27_is_present;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23.data,
			sizeof(ctrl_23.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8.data,
			size_of_query8);
	if (retval < 0)
		return retval;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8.data0_is_present;

	if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
		extra_data->data15_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present +
				query_8.data6_is_present +
				query_8.data7_is_present +
				query_8.data8_is_present +
				query_8.data9_is_present +
				query_8.data10_is_present +
				query_8.data11_is_present +
				query_8.data12_is_present +
				query_8.data13_is_present +
				query_8.data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

	rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
	rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

	retval = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x =
			((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_x_coord_msb << 8);
	rmi4_data->sensor_max_y =
			((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_y_coord_msb << 8);
#ifdef TPD_HAVE_BUTTON
	rmi4_data->sensor_max_y = rmi4_data->sensor_max_y * TPD_DISPLAY_HEIGH_RATIO / TPD_TOUCH_HEIGH_RATIO;
#endif
	TP_DBG(
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->num_of_rx = ctrl_8.num_of_rx;
	rmi4_data->num_of_tx = ctrl_8.num_of_tx;
	rmi4_data->max_touch_width = max(rmi4_data->num_of_rx,
			rmi4_data->num_of_tx);
	rmi4_data->f12_wakeup_gesture = query_5.ctrl27_is_present;
	        extra_data->ctrl_0_offset = ctrl_0_offset;
 	        extra_data->ctrl23_offset = ctrl_23_offset;
		extra_data->ctrl27_offset = ctrl_27_offset;
		extra_data->ctrl26_offset = ctrl_26_offset;
	if (rmi4_data->f12_wakeup_gesture) {
		extra_data->ctrl20_offset = ctrl_20_offset;
		extra_data->data4_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present;
               
	}	
	//yaohua.li start	
		unsigned char gesture_enable = 0x7F;  // enable double tap
		TP_DBG("[syna]:ctrl_base:%x, ctrl27_offset:%x\n", fhandler->full_addr.ctrl_base,extra_data->ctrl27_offset);
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base+extra_data->ctrl27_offset,
			&gesture_enable,
			sizeof(gesture_enable));
		if (retval < 0) {
		TP_DBG(
				"%s: Failed to enable single gesture\n",
				__func__);
		return;
		}		
	//yaohua.li end

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);
	if (!fhandler->data) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fhandler->data\n",
				__func__);
		return -ENOMEM;
	}

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		TP_DBG(
				"%s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;
	fhandler->extra = NULL;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->max_count = f1a->button_query.max_button_count + 1;

	f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
	if (!f1a->button_control.txrx_map) {
		TP_DBG(
				"%s: Failed to alloc mem for tx rx mapping\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		TP_DBG(
				"%s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->max_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		TP_DBG(
				"%s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char ii;
	unsigned char mapping_offset = 0;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	mapping_offset = f1a->button_query.has_general_control +
			f1a->button_query.has_interrupt_enable +
			f1a->button_query.has_multibutton_select;

	if (f1a->button_query.has_tx_rx_map) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.ctrl_base + mapping_offset,
				f1a->button_control.txrx_map,
				sizeof(f1a->button_control.txrx_map));
		if (retval < 0) {
			TP_DBG(
					"%s: Failed to read tx rx mapping\n",
					__func__);
			return retval;
		}

		rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
	}

	if (cap_button_map.map) {
		if (cap_button_map.nbuttons != f1a->max_count) {
			f1a->valid_button_count = min(f1a->max_count,
					cap_button_map.nbuttons);
		} else {
			f1a->valid_button_count = f1a->max_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] = cap_button_map.map[ii];
	}
	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_control.txrx_map);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_fn *fhandler_temp;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry_safe(fhandler,
				fhandler_temp,
				&rmi->support_fn_list,
				link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			list_del(&fhandler->link);
			kfree(fhandler);
		}
	}
	INIT_LIST_HEAD(&rmi->support_fn_list);

	return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode)
{
	int retval;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	unsigned char command = 0x01;
	unsigned char intr_status;
	struct synaptics_rmi4_f01_device_status status;

	/* Do a device reset first */
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	msleep(DELAY_S7300_RESET_READY);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (retval < 0)
		return retval;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			msleep(20);

		else
			return -1;

		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0)
			return retval;

		timeout -= 20;
	}

	if (timeout != CHECK_STATUS_TIMEOUT_MS)
		*was_in_bl_mode = true;

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
                           #ifdef FTM_UPDATE_FIRMWARE
                           in_bootloader_mode=1;
                           #endif
		TP_DBG("%s: In flash prog mode, status = 0x%02x\n",
				__func__,
				status.status_code);
	} else {
		rmi4_data->flash_prog_mode = false;
                           #ifdef FTM_UPDATE_FIRMWARE
                           ftm_force_update=0;
                           in_bootloader_mode=0;
                           #endif
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			&intr_status,
			sizeof(intr_status));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to read interrupt status\n",
				__func__);
		return retval;
	}

	return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to set configured\n",
				__func__);
		return;
	}

	rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
	device_ctrl |= CONFIGURED;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to set configured\n",
				__func__);
	}

	return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
	bool was_in_bl_mode;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

rescan_pdt:
	was_in_bl_mode = false;
	intr_count = 0;
	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				TP_DBG(
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			TP_DBG(
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					TP_DBG(
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f01_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;

				retval = synaptics_rmi4_check_status(rmi4_data,
						&was_in_bl_mode);
				if (retval < 0) {
					TP_DBG(
							"%s: Failed to check status\n",
							__func__);
					return retval;
				}

				if (was_in_bl_mode) {
					kfree(fhandler);
					fhandler = NULL;
					goto rescan_pdt;
				}

				if (rmi4_data->flash_prog_mode)
					goto flash_prog_mode;

				break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					TP_DBG(
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F12:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					TP_DBG(
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f12_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					TP_DBG(
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
					kfree(fhandler);
					fhandler = NULL;
#else
					return retval;
#endif
				}
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	TP_DBG(
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
			(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
			(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		TP_DBG(
				"%s: Non-Synaptics device found, manufacturer ID = %d\n",
				__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0)
		return retval;

	rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}
	
	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		rmi4_data->enable_wakeup_gesture = WAKEUP_GESTURE;
	else
		rmi4_data->enable_wakeup_gesture = false;

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			TP_DBG(
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				return retval;
		}
	}

	synaptics_rmi4_set_configured(rmi4_data);

	return 0;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL

	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers,0);

#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture) {
		set_bit(KEY_POWER, rmi4_data->input_dev->keybit);
		input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_POWER);
	}
	return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		TP_DBG(
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}

	rmi4_data->input_dev->name = DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->dev.parent = &rmi4_data->i2c_client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	synaptics_rmi4_set_params(rmi4_data);

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		TP_DBG(
				"%s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}

	return 0;

err_register_input:
err_query_device:
	//synaptics_rmi4_empty_fn_list(rmi4_data);
	//input_free_device(rmi4_data->input_dev);

err_input_device:
	return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
mutex_lock(&rmi4_report_mutex);
#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(rmi4_data->input_dev);
#endif
	input_sync(rmi4_data->input_dev);
mutex_unlock(&rmi4_report_mutex);
	rmi4_data->fingers_on_2d = false;

	return 0;
}

static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned short intr_addr;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
				synaptics_rmi4_f12_set_enables(rmi4_data, 0);
				break;
			}
		}
	}

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			TP_DBG("%s: Interrupt enable mask %d = 0x%02x\n",
				__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				goto exit;
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reinit != NULL)
				exp_fhandler->exp_fn->reinit(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	synaptics_rmi4_set_configured(rmi4_data);

	retval = 0;

exit:
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
	unsigned char command = 0x01;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	rmi4_data->touch_stopped = true;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	msleep(DELAY_S7300_RESET_READY);


	synaptics_rmi4_free_fingers(rmi4_data);

	synaptics_rmi4_empty_fn_list(rmi4_data);

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to query device\n",
				__func__);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	synaptics_rmi4_set_params(rmi4_data);

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reset != NULL)
				exp_fhandler->exp_fn->reset(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->touch_stopped = false;

	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

	return 0;
}

/**
* synaptics_rmi4_exp_fn_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a work thread that checks for the insertion and
* removal of other expansion Function modules such as rmi_dev and calls
* their initialization and removal callback functions accordingly.
*/
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler_temp;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;


	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				exp_fhandler_temp,
				&exp_data.list,
				link) {
			if ((exp_fhandler->exp_fn->init != NULL) &&
					exp_fhandler->insert) {
				retval = exp_fhandler->exp_fn->init(rmi4_data);
				if (retval < 0) {
					list_del(&exp_fhandler->link);
					kfree(exp_fhandler);
				} else {
					exp_fhandler->insert = false;
				}
			} else if ((exp_fhandler->exp_fn->remove != NULL) &&
					exp_fhandler->remove) {
				exp_fhandler->exp_fn->remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);


	return;
}

/**
* synaptics_rmi4_new_function()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_function(struct synaptics_rmi4_exp_fn *exp_fn,
		bool insert)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			TP_DBG("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->exp_fn = exp_fn;
		exp_fhandler->insert = true;
		exp_fhandler->remove = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->exp_fn->fn_type == exp_fn->fn_type) {
				exp_fhandler->insert = false;
				exp_fhandler->remove = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
				&exp_data.work,
				msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

/*
	ADD YAOHUA.LI
*/

int sy_open(struct inode *inode, struct file *filp){ 

      TP_DBG("[SY]into sy_open\n");
      if (g_pts == NULL)  
	  		TP_DBG("private_ts is NULL~~~");
                   
      return 0;
}
int sy_release(struct inode *inode, struct file *filp){    
      return 0;
}
static ssize_t sy_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
    int ret;
    char *tmp;
    TP_DBG("[SY]into elan_iap_write\n");
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }  
    ret = i2c_master_send(g_pts->i2c_client, tmp, count); 
    kfree(tmp);
    return 0;//(ret == 1) ? count : ret;

}
ssize_t sy_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    char *tmp;
    int ret;  
    long rc;

    TP_DBG("[SY]into SY_read\n");
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;  
 /*   ret = i2c_master_recv(g_pts->i2c_client, tmp, count);  
    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;*/
    return 0;         
}
void synaptics_chip_reset(void)
{
 /*   mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN,GPIO_CTP_RST_PIN_M_GPIO);// GPIO_CTP_EN_PIN_M_GPIO); 
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
*/

        /*regulator_disable(tpd->reg);
        msleep(200);
        regulator_set_voltage(tpd->reg, 2800000, 2800000);  // set 1.8v
        regulator_enable(tpd->reg);  //enable regulator
*/

    msleep(DELAY_S7300_BOOT_READY);
    tpd_gpio_output(GTP_RST_PORT, 0);
    msleep(DELAY_S7300_RESET);
   tpd_gpio_output(GTP_RST_PORT, 1);
    msleep(DELAY_S7300_RESET_READY);
  //  mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN); 
//   mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_LOW, tpd_eint_handler,0);
    enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}
static long sy_ioctl(/*struct inode *inode,*/ struct file *filp,    unsigned int cmd, unsigned long arg)
{
  int __user *ip = (int __user *)arg;
        TP_DBG("[ELAN]into elan_iap_ioctl\n");
	TP_DBG("cmd value %x, %ld\n", cmd, arg);
	uint64_t checksum;
	int flag;
	int rc = 0;
	int New_FW_ID= 0;                
        int New_FW_VER= 0;
	uint8_t buf[64]={0};
         switch (cmd) 
	{     
		case IOCTL_I2C_SY_ID:
		     flag=get_synaptics_versionid();
		     TP_DBG("=====================IOCTL_I2C_SY_INFO======%d=======================\n",flag);
                 if(copy_to_user( ip,&flag, sizeof(int))!=0)
		{
		   TP_DBG("copy_to_user error");
		   rc = -EFAULT;
		}
                 break;
		case IOCTL_I2C_SY_IMG_ID:
		 flag=0;
		 flag =  get_synaptics_image_versionid();
                 TP_DBG("=====================IOCTL_I2C_SY_IMG_ID======%d=======================\n",flag);
               if(copy_to_user( ip,&flag, sizeof(int))!=0)
		{
		   TP_DBG("copy_to_user error");
		   rc = -EFAULT;
		}
		 break;
                    #ifdef FTM_UPDATE_FIRMWARE
                            case SYNAPTICS_IOCTL_TP_UPGRADE_SET_BIN_LEN:
                            TP_DBG("FTM_UPDATE_FIRMWARE\n");
                            ftm_force_update=1;
                            factory_update_bin_size= (int __user)arg;
                            break;
                            case SYNAPTICS_IOCTL_TP_UPGRADE_SET_BIN_BUF:
                            TP_DBG("FTM_UPDATE_FIRMWARE\n");
                            factory_update_bin=(uint8_t __user *)arg;
                            
                            break;
                     case SYNAPTICS_IOCTL_GET_UPDATE_PROGREE:
                            return in_bootloader_mode;
                            break;
                     case SYNAPTICS_IOCTL_FW_UPDATE:
                    #endif
	        case IOCTL_I2C_SY_UPDATE:
 		flag=0;

			 TP_DBG("=====================UPDATE============================\n");   
                           update_FW();
                          
		synaptics_chip_reset();   
	
                           return synaptics_get_fw_version();
		 break;
                    
                   default:            
                            break; 
	}

}
struct file_operations sy_touch_fops = {    
        .open =            	sy_open,    
        .write =         	sy_write,    
        .read =          	sy_read,    
        .release =        	sy_release,  
        .unlocked_ioctl = 	sy_ioctl, 
 };
 /**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */

static int synaptics_rmi4_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)

{
	int retval;
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		TP_DBG(
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

	// gpio setting
	/*mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
  	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	*/
		tpd_gpio_output(GTP_INT_PORT, 0);	
	tpd_gpio_output(GTP_RST_PORT, 1);


	// power up sequence
/*
	retval = regulator_enable(tpd->reg);
	if (retval != 0) {
		dev_err(&client->dev, "Failed to enable reg-vgp6: %d\n", retval);
		return -EIO;

	}*/



                msleep(DELAY_S7300_BOOT_READY);
                tpd_gpio_output(GTP_RST_PORT, 1);
                msleep(DELAY_S7300_BOOT_READY);
                tpd_gpio_output(GTP_RST_PORT, 0);
                msleep(DELAY_S7300_RESET);
                msleep(DELAY_S7300_RESET_READY);
                tpd_gpio_output(GTP_RST_PORT, 1);
                msleep(DELAY_S7300_RESET_READY);




	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		TP_DBG(
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->touch_stopped = false;
	rmi4_data->sensor_sleep = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = false;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;
         
	//yaohua.li

         rmi4_data->firmware.minor = MISC_DYNAMIC_MINOR;
         rmi4_data->firmware.name = "sy-dev";
         rmi4_data->firmware.fops = &sy_touch_fops;
         rmi4_data->firmware.mode = S_IRWXUGO; 
         g_pts =rmi4_data;
         if (misc_register(&rmi4_data->firmware) < 0)
                   TP_DBG("mtk-tpd:[SY] misc_register failed!!\n");
         else
                   TP_DBG("[SY] misc_register finished!!\n"); 
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data->rmi4_reset_mutex));

	i2c_set_clientdata(client, rmi4_data);
	retval = synaptics_rmi4_set_input_dev(rmi4_data);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to set up input device\n",
				__func__);
		goto err_set_input_dev;
	}
//add by yaohua.li
//	  strcpy(Tg_buf,"mcs");
      SY_key_dev= input_allocate_device();
	if (! SY_key_dev) 
	{
		TP_DBG("[STP]SY_key_dev: fail!\n");
	}
        __set_bit(EV_KEY,  SY_key_dev->evbit);
	__set_bit(KEY_SY_SENSOR,  SY_key_dev->keybit);
	SY_key_dev->id.bustype = BUS_HOST;
	SY_key_dev->name = "TPD_GESTURE";
	if(input_register_device(SY_key_dev))
	{
		TP_DBG("[STP]SY_key_dev register : fail!\n");
	}else
	{
		TP_DBG("[STP]SY_key_dev register : success!!\n");
	} 
//end by yaohua.li
        syna_gtp_test_sysfs_init();
#ifdef CONFIG_HAS_EARLYSUSPEND
	rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
	rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
	register_early_suspend(&rmi4_data->early_suspend);
#endif

	thread = kthread_run(touch_event_handler, rmi4_data, "synaptics-tpd");
	if ( IS_ERR(thread) ) {
		retval = PTR_ERR(thread);
		TP_DBG(" %s: failed to create kernel thread: %d\n",__func__, retval);
	}




	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}


	exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			TP_DBG(
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}
		tpd_irq_registration();
	retval = synaptics_rmi4_irq_enable(rmi4_data, true);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to enable attention interrupt\n",
				__func__);
		goto err_enable_irq;
	}

	tpd_load_status = 1;
	g_dev = &rmi4_data->input_dev->dev;
	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	synaptics_rmi4_irq_enable(rmi4_data, false);

err_enable_irq:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif

	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

err_set_input_dev:
	//kfree(rmi4_data);

	return retval;
}

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */

static int synaptics_rmi4_remove(struct i2c_client *client)

{
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	synaptics_rmi4_irq_enable(rmi4_data, false);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif

	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

	kfree(rmi4_data);
             syna_gtp_test_sysfs_deinit();
	return 0;
}

#ifdef CONFIG_PM
static void synaptics_rmi4_f11_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char reporting_control;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F11)
			break;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	reporting_control = (reporting_control & ~MASK_3BIT);
	if (enable)
		reporting_control |= F11_WAKEUP_GESTURE_MODE;
	else
		reporting_control |= F11_CONTINUOUS_MODE;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	return;
}

static void synaptics_rmi4_f12_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char offset;
	unsigned char reporting_control[3];
	TP_DBG("=====synaptics_rmi4_f12_wg===start===\n");
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======synaptics_rmi4_f12_wg======find====\n");
			break;
             }
	}

	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	offset = extra_data->ctrl20_offset;
	
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}
	TP_DBG("======enables==%d===========\n",enable);
	if (enable)
	{
		
		reporting_control[2] = F12_WAKEUP_GESTURE_MODE;
	}
	else
	{
		reporting_control[2] = F12_CONTINUOUS_MODE;
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////
	retval = synaptics_rmi4_i2c_write(rmi4_data,fhandler->full_addr.ctrl_base + offset,reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		TP_DBG("========%s: Failed to change reporting mode\n",__func__);
		return;
	}
//yaohua.li
TP_DBG("=====synaptics_rmi4_f12_wg===end===\n");
	return;
}

static void synaptics_rmi4_wakeup_gesture(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
TP_DBG("f11=%d==f12=%d,enable=%d\n",rmi4_data->f11_wakeup_gesture,rmi4_data->f12_wakeup_gesture,enable);
	if (rmi4_data->f11_wakeup_gesture)
		synaptics_rmi4_f11_wg(rmi4_data, enable);
	else if (rmi4_data->f12_wakeup_gesture)
		synaptics_rmi4_f12_wg(rmi4_data, enable);

	return;
}
 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}

	return;
}

 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	} else {
		rmi4_data->sensor_sleep = false;
	}

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 /**
 * synaptics_rmi4_early_suspend()
 *
 * Called by the kernel during the early suspend phase when the system
 * enters suspend.
 *
 * This function calls synaptics_rmi4_sensor_sleep() to stop finger
 * data acquisition and put the sensor to sleep.
 */
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	if (rmi4_data->stay_awake) {
		rmi4_data->staying_awake = true;
		return;
	} else {
		rmi4_data->staying_awake = false;
	}
	TP_DBG("synaptics_rmi4_early_suspend");
	if (rmi4_data->enable_wakeup_gesture) {
		synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		goto exit;
	}
	

	rmi4_data->touch_stopped = true;
	synaptics_rmi4_irq_enable(rmi4_data, false);
	synaptics_rmi4_sensor_sleep(rmi4_data);
	synaptics_rmi4_free_fingers(rmi4_data);

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->early_suspend != NULL)
				exp_fhandler->exp_fn->early_suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));
	
exit:
	rmi4_data->sensor_sleep = false;
	return;
}

 /**
 * synaptics_rmi4_late_resume()
 *
 * Called by the kernel during the late resume phase when the system
 * wakes up from suspend.
 *
 * This function goes through the sensor wake process if the system wakes
 * up from early suspend (without going into suspend).
 */
static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	if (rmi4_data->staying_awake)
		return;
             TP_DBG("synaptics_rmi4_late_resume");
	if (rmi4_data->enable_wakeup_gesture) {
		synaptics_rmi4_wakeup_gesture(rmi4_data, false);
		goto exit;
	}
	
	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

	if (rmi4_data->sensor_sleep == true) {
		synaptics_rmi4_sensor_wake(rmi4_data);
		synaptics_rmi4_irq_enable(rmi4_data, true);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			TP_DBG(
					"%s: Failed to reinit device\n",
					__func__);
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->late_resume != NULL)
				exp_fhandler->exp_fn->late_resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

exit:
	rmi4_data->sensor_sleep = false;
	rmi4_data->touch_stopped = false;

	return;
}
#endif
//yaohua.l
//yaohua.l
void set_tgesture_enable_f11(void)
{    
        int retval = 0;
        struct synaptics_rmi4_fn *fhandler;
        struct synaptics_rmi4_device_info *rmi;
        struct synaptics_rmi4_f11_extra_data *extra_data; 	
        struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
        rmi = &(rmi4_data->rmi4_mod_info);	
	list_for_each_entry(fhandler, &rmi->support_fn_list, link) 
	{
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======set_tgesture_enable======find====\n");
			break;
             }
 	}
	       extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;
	//yaohua.li start	
		unsigned char gesture_enable = 0x7F;  // enable double tap
TP_DBG("[syna]:ctrl_base:%x\n", fhandler->full_addr.ctrl_base);
#if 0
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&gesture_enable,
			sizeof(gesture_enable));
		if (retval < 0) 
               {
		   TP_DBG("%s: Failed to enable single gesture\n",__func__);
		   return;
	   }	
#endif
	//yaohua.li end
 /*     TP_DBG("==========set_tgesture_enable==============\n");
 	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	synaptics_rmi4_wakeup_gesture(rmi4_data, true);*/
	
	return;
}

void set_tgesture_enable(void)
{    
	int retval = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
        struct synaptics_rmi4_f12_extra_data *extra_data; 	
        struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	rmi = &(rmi4_data->rmi4_mod_info);	
	list_for_each_entry(fhandler, &rmi->support_fn_list, link) 
	{
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======set_tgesture_enable======find====\n");
			break;
             }
 	}
	       extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	//yaohua.li start	
		unsigned char gesture_enable = 0x7F;  // enable double tap
		TP_DBG("[syna]:ctrl_base:%x, ctrl27_offset:%x\n", fhandler->full_addr.ctrl_base,extra_data->ctrl27_offset);
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base+extra_data->ctrl27_offset,
			&gesture_enable,
			sizeof(gesture_enable));
		if (retval < 0) 
               {
		   TP_DBG("%s: Failed to enable single gesture\n",__func__);
		   return;
	   }		
	//yaohua.li end
 /*     TP_DBG("==========set_tgesture_enable==============\n");
 	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	synaptics_rmi4_wakeup_gesture(rmi4_data, true);*/
	
	return;
}
void set_tgesture_unenable(void)
{    
	int retval = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
        struct synaptics_rmi4_f12_extra_data *extra_data; 	
        struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	rmi = &(rmi4_data->rmi4_mod_info);	
	list_for_each_entry(fhandler, &rmi->support_fn_list, link) 
	{
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======et_tgesture_unenable======find====\n");
			break;
             }
 	}
	       extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	//yaohua.li start	
		unsigned char gesture_enable = 0x0;  // enable double tap
		TP_DBG("[syna]:ctrl_base:%x, ctrl27_offset:%x\n", fhandler->full_addr.ctrl_base,extra_data->ctrl27_offset);
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base+extra_data->ctrl27_offset,
			&gesture_enable,
			sizeof(gesture_enable));
		if (retval < 0) 
               {
		   TP_DBG("%s: Failed to enable single gesture\n",__func__);
		   return;
	   }		
	//yaohua.li end*/
 /*	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
         TP_DBG("==========get_tgesture_unenable==============\n");
	synaptics_rmi4_wakeup_gesture(rmi4_data, false);*/
	 return 0;
}

void set_Delicacy_enable(void)
{    
	int retval = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
        struct synaptics_rmi4_f12_extra_data *extra_data; 	
        struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	rmi = &(rmi4_data->rmi4_mod_info);	
	list_for_each_entry(fhandler, &rmi->support_fn_list, link) 
	{
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======synaptics_rmi4_f12_wg======find====\n");
			break;
             }
 	}
	       extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	       unsigned char Delicacy = 0x21;  // enable double tap
		TP_DBG("===========synaptics_rmi4_f12_abs_report==TP_Delicacy===================true start====\n");
		TP_DBG("[syna]:ctrl_base:%x, ctrl23_offset:%x\n", fhandler->full_addr.ctrl_base,extra_data->ctrl23_offset);
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,fhandler->full_addr.ctrl_base+extra_data->ctrl23_offset,&Delicacy,sizeof(Delicacy));
		if (retval < 0) 
		{
			//TP_DBG(&(rmi4_data->input_dev->dev),"%s: Failed to enable single gesture\n",__func__);
	                TP_DBG("===========synaptics_rmi4_f12_abs_report==TP_Delicacy===================true end====\n");
			return 0;
		}
		unsigned char Delicacy_enable = 0x01;  // enable double tap
		retval =  synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base+extra_data->ctrl26_offset,
			&Delicacy_enable,
			sizeof(Delicacy_enable));
		if (retval < 0) 
		{
			TP_DBG("%s: Failed to enable single gesture\n",__func__);
			return;
		}
      TP_DBG("==========set__Delicacy_enable==============\n");
	return;
}
void set_Delicacy_unenable(void)
{
       TP_DBG("==========set__Delicacy_unenable==============\n");
	int retval = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
        struct synaptics_rmi4_f12_extra_data *extra_data; 	
        struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	rmi = &(rmi4_data->rmi4_mod_info);	
	list_for_each_entry(fhandler, &rmi->support_fn_list, link) 
	{
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======synaptics_rmi4_f12_wg======find====\n");
			break;
             }
 	}
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	       unsigned char Delicacy = 0x03;  // enable double tap
		TP_DBG("===========synaptics_rmi4_f12_abs_report==TP_Delicacy===================true start====\n");
		TP_DBG("[syna]:ctrl_base:%x, ctrl23_offset:%x\n", fhandler->full_addr.ctrl_base,extra_data->ctrl23_offset);
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,fhandler->full_addr.ctrl_base+extra_data->ctrl23_offset,&Delicacy,sizeof(Delicacy));
		if (retval < 0) 
		{
//			TP_DBG(&(rmi4_data->input_dev->dev),"%s: Failed to enable single gesture\n",__func__);
	                TP_DBG("===========synaptics_rmi4_f12_abs_report==TP_Delicacy===================true end====\n");
			return 0;
		}
		unsigned char Delicacy_enable = 0x00;  // enable double tap
		retval =  synaptics_rmi4_i2c_write(rmi4_data,
			fhandler->full_addr.ctrl_base+extra_data->ctrl26_offset,
			&Delicacy_enable,
			sizeof(Delicacy_enable));
		if (retval < 0) 
		{
			TP_DBG("%s: Failed to enable single gesture\n",__func__);
			return;
		}
      TP_DBG("==========set__Delicacy_unenable==============\n");
	return;
}
void set_sleep_unenable(void)
{
       TP_DBG("==========set_sleep_unenable==============\n");
	int retval = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
        struct synaptics_rmi4_f12_extra_data *extra_data; 	
        struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	rmi = &(rmi4_data->rmi4_mod_info);	
	list_for_each_entry(fhandler, &rmi->support_fn_list, link) 
	{
	     if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
	     {
		TP_DBG("=======synaptics_rmi4_f12_wg======find====\n");
			break;
             }
 	}
	       extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	       unsigned char no_sleep = 0x00;  // enable double tap
		TP_DBG("===========synaptics_rmi4_f12_abs_report==set_sleep===================true start====\n");
		//TP_DBG("[syna]:ctrl_base:%x, ctrl23_offset:%x\n", fhandler->full_addr.ctrl_base,extra_data->ctrl_0_offset);
 		retval =  synaptics_rmi4_i2c_write(rmi4_data,fhandler->full_addr.ctrl_base,&no_sleep,sizeof(no_sleep));
		if (retval < 0) 
		{
			TP_DBG("%s: Failed to enable single gesture\n",__func__);
			return 0;
		}
      TP_DBG("==========set__Delicacy_unenable==============\n");
	return;
}
 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);

	if (rmi4_data->staying_awake)
		return 0;
	TP_DBG("============synaptics_rmi4_suspend===%d====\n",rmi4_data->enable_wakeup_gesture);
	bTPGesturetemp =bEnTGesture;
             TP_DBG("============synaptics_rmi4_suspend==bEnTGesture=%d====\n",bEnTGesture);
	if (bTPGesturetemp) {	
	//synaptics_chip_reset();
        //@xuchunsheng changed start for unlock screen slowly in 14/10/2015	
        //regulator_enable(tpd->reg);  //enable regulator
        //msleep(120);
        tpd_gpio_output(GTP_RST_PORT, 0);
        msleep(20);
        tpd_gpio_output(GTP_RST_PORT, 1);
        msleep(25);
        //enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        //@xuchunsheng changed end in 14/10/2015
	//set_tgesture_enable_f11();
	if (rmi4_data->enable_wakeup_gesture) {
		synaptics_rmi4_wakeup_gesture(rmi4_data, true);    
		goto exit;
	}
	}
	else
	{
	    if (rmi4_data->enable_wakeup_gesture) 
                {		
                TP_DBG("====rmi4_data->enable_wakeup_gesture===%d==\n",rmi4_data->enable_wakeup_gesture);
                synaptics_rmi4_wakeup_gesture(rmi4_data, false);
                }
	}
	if (!rmi4_data->sensor_sleep) {
		rmi4_data->touch_stopped = true;
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
		synaptics_rmi4_free_fingers(rmi4_data);
	}
        //regulator_disable(tpd->reg);
        msleep(200);	
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->suspend != NULL)
				exp_fhandler->exp_fn->suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);
	tpd_halt = 1;

exit:
	rmi4_data->sensor_sleep = true;
	return 0;
}

 /**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
         TP_DBG("============synaptics_rmi4_resume========\n");
	if (rmi4_data->staying_awake)
		return 0;
        TP_DBG("============synaptics_rmi4_resume===%d====\n",rmi4_data->enable_wakeup_gesture);
        //@xuchunsheng changed start for unlock screen slowly in 14/10/2015	
        //regulator_enable(tpd->reg);  //enable regulator
        msleep(120);
        tpd_gpio_output(GTP_RST_PORT, 0);
        msleep(20);
        tpd_gpio_output(GTP_RST_PORT, 1);
        msleep(50);
        enable_irq(touch_irq);//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        //@xuchunsheng changed end in 14/10/2015
	if (bTPGesturetemp) 
	{	
                
                if (rmi4_data->enable_wakeup_gesture) 
                {		
                TP_DBG("====rmi4_data->enable_wakeup_gesture===%d==\n",rmi4_data->enable_wakeup_gesture);
                synaptics_rmi4_wakeup_gesture(rmi4_data, false);
                goto exit;
                }
	}
	else
	{
                 //synaptics_chip_reset();
                 TP_DBG("============synaptics_rmi4_resume====set_tgesture_enable==\n");
	}
	synaptics_rmi4_sensor_wake(rmi4_data);
	synaptics_rmi4_irq_enable(rmi4_data, true);
	retval = synaptics_rmi4_reinit_device(rmi4_data);
	if (retval < 0) {
		TP_DBG(
				"%s: Failed to reinit device\n",
				__func__);
		return retval;
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->resume != NULL)
				exp_fhandler->exp_fn->resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);
///////////////////////////////////////////////////////////////////////////////
		if (1==Delicacy_status)
		{
                        synaptics_chip_reset();
  			set_tgesture_unenable();
                        TP_DBG("============set_tgesture_unenable=====================\n");
			set_Delicacy_enable();			
                        TP_DBG("============set_Delicacy_enable=====================\n");
			
		}		
/*		else if(0==Delicacy_status)
		{                       
                        synaptics_chip_reset();
		        set_Delicacy_unenable();
 			TP_DBG("============set_Delicacy_unenable=====================\n");
			set_tgesture_enable();
                        TP_DBG("============set_tgesture_unenable=====================\n");
		}
*/
exit:
	rmi4_data->sensor_sleep = false;
	rmi4_data->touch_stopped = false;
	tpd_halt = 0;

	return 0;
}

static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,
};
#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{"mtk-tpd", 0},
	{},
};
unsigned short force[] = {0,TPD_I2C_ADDR,I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
//static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);

MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);
static const struct of_device_id synaptics_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

static struct i2c_driver tpd_i2c_driver = {
	.probe = synaptics_rmi4_probe,

	.remove = synaptics_rmi4_remove,
	.driver = {
		   .name = TPD_DEVICE,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(synaptics_dt_match),
		   },

	//.detect = tpd_detect,
	.id_table = synaptics_rmi4_id_table,
	.address_list = (const unsigned short*) forces,
};

static int tpd_local_init(void)
{
	int retval;

	TP_DBG("synaptics I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	/*tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	//tpd->io_reg = regulator_get(tpd->tpd_dev, "vtouchio");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}*/
	if (tpd_load_status == 1) {
		TPD_DMESG("touch panel driver have inited.\n");

		return -1;
	}

	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TP_DBG("tangjie Error unable to add i2c driver.\n");
		return -1;
	}
#ifdef FTM_BUTTON  //TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif 
	boot_mode = get_boot_mode();
              // 7=ALARM_BOOT 9=LOW_POWER_OFF_CHARGING_BOOT 8=KERNEL_POWER_OFF_CHARGING_BOOT
	if (boot_mode == 3||boot_mode == 7||boot_mode == 9||boot_mode == 8) {  
		boot_mode = NORMAL_BOOT;
	}  
		tpd_type_cap = 1;
	return 0;
}
char* get_synaptics_fw_name(void)
{
	char bufsy[32]= {"DiJing"};
	static char buf[32]= {0};
	TP_DBG("===synaptics=======fw==name");
	strcpy(buf, bufsy);
	return buf;
}
char* get_synaptics_IC_name(void)
{
	char bufsy[32]= {"s2333"};
	static char buf[32]= {0};
	TP_DBG("===synaptics=======IC==name");
	strcpy(buf, bufsy);
	return buf;
}

char* get_synaptics_TpName( void )
{
   char tpd_desc_info[50]={0};  
   sprintf(tpd_desc_info, "%s-s5250-%s-0x%x",get_synaptics_fw_name(),get_synaptics_IC_name(),get_synaptics_versionid());
   TP_DBG("=======tpd_desc_info=%s===",tpd_desc_info);
   return tpd_desc_info;
}
char* get_synaptics_Version( void )
{
   char tpd_desc_info[50]={0};  
   sprintf(tpd_desc_info, "%s-s5250-synaptics-0x%x",get_synaptics_fw_name(),get_synaptics_versionid());
   TP_DBG("=======tpd_desc_info=%s===",tpd_desc_info);
   return get_synaptics_versionid();
}
//LINE<JIRA_ID><DATE20130422><add multi tp>zenghaihui
void synaptics_tpd_get_fw_version( char * fw_vendor_numb )
{
    u8 version_info = 0;
    u8 cfg_vs;
    
    //return synaptics_get_fw_version();
    version_info = synaptics_get_fw_version();
    sprintf(fw_vendor_numb, "%d", version_info);
}

void synaptics_tpd_get_fw_vendor_name(char * fw_vendor_name)
{
    char *product_id = synaptics_get_vendor_info();
    unsigned char  name[7]={"iTOUCH"},index=0;
     TP_DBG("=======synaptics_tpd_get_fw_vendor_name=%s===",product_id);
    if(product_id)
    {
        if( 0 == memcmp(product_id, "JTOUCH8811", 10))
        {
            //product_id=name;
            for(index=0; index < 6;index++)
            {
                name[index] = *product_id;
                product_id++;
            }
            name[index] = '\0';
            product_id = name;
        }
        sprintf(fw_vendor_name, "%s", "DIJING");
    }
    else
    {
        sprintf(fw_vendor_name, "%s", "UNKNOWN");
    }
}

void synaptics_ftm_force_update(char * ftm_update){
printk("geroge   synaptics ftm  force  update \n");
        ftm_force_update = true;
         update_FW();
          TP_DBG("=====================UPDATE============================\n");   
		 synaptics_chip_reset();    

}
 
static struct tpd_driver_t synaptics_rmi4_driver = {
	.tpd_device_name = "S2333",
	.tpd_local_init = tpd_local_init,
	.suspend = synaptics_rmi4_suspend,
	.resume = synaptics_rmi4_resume,
#ifdef FTM_BUTTON //TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
        .tpd_get_fw_version = synaptics_tpd_get_fw_version,
        .tpd_get_fw_vendor_name = synaptics_tpd_get_fw_vendor_name,
        .tpd_ftm_force_update=synaptics_ftm_force_update,

};

static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO("mtk-tpd", (TPD_I2C_ADDR))};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
//        TP_DBG("synaptics_rmi4_init i2c=%d",I2C_CAP_TOUCH_CHANNEL);
	//i2c_register_board_info(I2C_CAP_TOUCH_CHANNEL, &i2c_tpd, 1);
	if(tpd_driver_add(&synaptics_rmi4_driver) < 0){
		TP_DBG("Fail to add tpd driver\n");
		return -1;
	}

	return 0;
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_driver);
	return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL v2");
