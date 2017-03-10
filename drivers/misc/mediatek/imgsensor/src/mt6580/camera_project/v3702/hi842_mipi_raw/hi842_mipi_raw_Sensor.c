/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 Hi842_mipi_raw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *  
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include <linux/types.h>
#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi842_mipi_raw_Sensor.h"

#define PFX "HI842_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#define LOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)


#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define SCALE 1    // for hs_video 1 use scale, 0 use crop



#define HI842_OTP_Enable
#ifdef HI842_OTP_Enable
extern int read_Hi842_otp(void);

extern int RG_ratio_unit;
extern int RG_ratio_golden;
extern int BG_ratio_unit; 
extern int BG_ratio_golden;
#endif
static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = HI842_SENSOR_ID,
	
	.checksum_value = 0xa340b362,
	
	.pre = {
		.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2504,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 288000000,
		.linelength = 3800,	
		.framelength = 2504,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 288000000,
		.linelength = 3800,	
		.framelength = 5048,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
    },
	.normal_video = {

		.pclk = 288000000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2504,			//record different mode's framelength
		.startx =0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 3264,		//record different mode's width of grabwindow
		.grabwindow_height = 2448,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	 .hs_video = {
        .pclk = 288000000,
        .linelength = 3800,				//record different mode's linelength
		.framelength = 630,			//record different mode's framelength
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640 ,		//record different mode's width of grabwindow
		.grabwindow_height = 480 ,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
    },
    .slim_video = {
       .pclk = 288000000,
		.linelength = 3800,
		.framelength = 2504,
		.startx = 0,
		.starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
	.margin = 6,
	.min_shutter = 6,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
    .hs_video_delay_frame = 3,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 3,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40,0xff},
};



static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x0100,					//current shutter
	.gain = 0xe0,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
 .test_pattern = KAL_FALSE, 
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
 .ihdr_en = 0, 
	.i2c_write_id = 0x40,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 3264, 2448,    0,    0, 3264, 2448,  1632, 1224, 0000, 0000, 1632, 1224,      0,    0, 1632, 1224}, // preview
 { 3264, 2448,      0,    0, 3264, 2448, 3264, 2448, 0000, 0000, 3264, 2448,      0,    0,  3264, 2448}, // capture
 { 3264, 2448,     0,    0, 3264, 2448,  1624, 1224, 0000, 0000, 1624, 1224,      0,    0, 1632, 1224}, // video  //1632--->1624
 { 3264, 2448,     352,  504, 2560, 1440, 1280, 720, 0000, 0000, 1280,  720,      0,    0, 640,  480}, //hight speed video
 { 3264, 2448,     352,  504, 2560, 1440, 1280, 720, 0000, 0000, 1280,  720,      0,    0, 1280,  720}};// slim video







void write_cmos_sensor_16(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}


void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

 kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}



static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x0006, imgsensor.frame_length );
	//write_cmos_sensor(0x0007, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0008, imgsensor.line_length );
	//write_cmos_sensor(0x0009, imgsensor.line_length & 0xFF);
	 write_cmos_sensor(0x0046, 0x0000);
  
}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{

	LOG_INF("write_shutter");
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk * 10 / (imgsensor.line_length * imgsensor.frame_length);
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else{
			  write_cmos_sensor(0x0046, 0x0100);
		    write_cmos_sensor(0x0006, imgsensor.frame_length);
		    //write_cmos_sensor(0x0004, (shutter) );
        write_cmos_sensor(0x0046, 0x0000);
	}
	} else {
		// Extend frame length
				write_cmos_sensor(0x0046, 0x0100);
		    write_cmos_sensor(0x0006, imgsensor.frame_length);
		    //write_cmos_sensor(0x0004, (shutter) );
        write_cmos_sensor(0x0046, 0x0000);
	}

	// Update Shutter
	write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x0004, shutter);	  
	write_cmos_sensor(0x0046, 0x0000);
	
	LOG_INF("shutter =%d, framelength =%d", shutter,imgsensor.frame_length);


         LOG_INF("0x0508 is 0x%x, 0x0509 is 0x%x", read_cmos_sensor(0x0508), read_cmos_sensor(0x0509)); //20151202 add 

          LOG_INF("0x050a is 0x%x, 0x050b is 0x%x", read_cmos_sensor(0x050a), read_cmos_sensor(0x050b)); //20151202 add 

          LOG_INF("0x050c is 0x%x, 0x050d is 0x%x", read_cmos_sensor(0x050c), read_cmos_sensor(0x050d)); //20151202 add 

          LOG_INF("0x050e is 0x%x, 0x050f is 0x%x", read_cmos_sensor(0x050e), read_cmos_sensor(0x050f)); //20151202 add

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{

	LOG_INF("set_shutter");
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
  kal_uint16 reg_gain = 0x0000;
	reg_gain = gain / 4 - 16;

	return (kal_uint16)reg_gain;
}
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(0x0046, 0x0100);
	  write_cmos_sensor(0x003a,reg_gain<<8);   
	  write_cmos_sensor(0x0046, 0x0000);

    return gain;
		
}	/*	set_gain  */
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	#if 0
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        // Extend frame length first
        write_cmos_sensor(0x0006, imgsensor.frame_length);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

        set_gain(gain);
    }
#endif
}




static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0000,0x0000);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0000,0x0100);
			
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0000,0x0200);
			
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0000,0x0300);

			break;
		default:
			LOG_INF("Error image_mirror setting");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
#ifdef HI842_OTP_Enable
static void sensor_wb_gain_setting(void)
{

	int R_gain = 0x200; 
	int G_gain = 0x200; 
	int B_gain = 0x200; 

	R_gain = 0x200 * RG_ratio_golden / RG_ratio_unit; 
	B_gain = 0x200 * BG_ratio_golden / BG_ratio_unit; 

	LOG_INF("G_gain is 0x%x - before\n",G_gain);
	LOG_INF("R_gain is 0x%x - before\n",R_gain);
	LOG_INF("B_gain is 0x%x - before\n",B_gain);

	if(R_gain < B_gain)
	{
		if(R_gain < 0x200)
		{
		  B_gain = 0x200 * B_gain / R_gain; 
	  	  G_gain = 0x200 * G_gain / R_gain; 
	  	  R_gain = 0x200;
		}
	}
	else
	{
		if(B_gain < 0x200)
		{
		  R_gain = 0x200 * R_gain / B_gain; 
	  	  G_gain = 0x200 * G_gain / B_gain; 
	  	  B_gain = 0x200;
		}
	}

	  

    if((R_gain < 0x200) || (G_gain < 0x200) || (B_gain < 0x200)) // ?? 

    {

        R_gain = 0x200; 

       G_gain = 0x200; 

       B_gain = 0x200; 

    }

    



	LOG_INF("G_gain is 0x%x - after\n",G_gain);
	LOG_INF("R_gain is 0x%x - after\n",R_gain);
	LOG_INF("B_gain is 0x%x - after\n",B_gain);

     #if 0
	write_cmos_sensor(0x0508, (kal_uint8)G_gain>>8);
	write_cmos_sensor(0x0509, (kal_uint8)G_gain&0xff);
	write_cmos_sensor(0x050a, (kal_uint8)G_gain>>8);
	write_cmos_sensor(0x050b, (kal_uint8)G_gain&0xff);

	write_cmos_sensor(0x050c, (kal_uint8)R_gain>>8);
	write_cmos_sensor(0x050d, (kal_uint8)R_gain&0xff);
	write_cmos_sensor(0x050e, (kal_uint8)B_gain>>8);
	write_cmos_sensor(0x050f, (kal_uint8)B_gain&0xff);
	#else
	//write_cmos_sensor() is 2D2A I2C function

	write_cmos_sensor(0x0508, G_gain);
       write_cmos_sensor(0x050a, G_gain);
       write_cmos_sensor(0x050c, R_gain);
       write_cmos_sensor(0x050e, B_gain);

      #endif
       
         LOG_INF("wb0x0508 is 0x%x, 0x0509 is 0x%x", read_cmos_sensor(0x0508), read_cmos_sensor(0x0509)); //20151202 add 

          LOG_INF("wb0x050a is 0x%x, 0x050b is 0x%x", read_cmos_sensor(0x050a), read_cmos_sensor(0x050b)); //20151202 add 

          LOG_INF("wb0x050c is 0x%x, 0x050d is 0x%x", read_cmos_sensor(0x050c), read_cmos_sensor(0x050d)); //20151202 add 

          LOG_INF("wb0x050e is 0x%x, 0x050f is 0x%x", read_cmos_sensor(0x050e), read_cmos_sensor(0x050f)); //20151202 add

}
#endif

static void sensor_init(void)
{
	
	LOG_INF("Enter init\n");

// Initial setting ********
write_cmos_sensor(0x0a00, 0x0000);
write_cmos_sensor(0x0e00, 0x0002);	
write_cmos_sensor(0x0e02, 0x0002); 
//// Firmware tg_pt              
write_cmos_sensor(0x2000, 0x7400); 
write_cmos_sensor(0x2002, 0x1796); 
write_cmos_sensor(0x2004, 0x7006);   
write_cmos_sensor(0x2006, 0x1796);
write_cmos_sensor(0x2008, 0x09d0); 
write_cmos_sensor(0x200a, 0x5001); 
write_cmos_sensor(0x200c, 0x5021); 
write_cmos_sensor(0x200e, 0x0041); 
write_cmos_sensor(0x2010, 0x0043); 
write_cmos_sensor(0x2012, 0x02c3); 
write_cmos_sensor(0x2014, 0x13cd); 
write_cmos_sensor(0x2016, 0x204f); 
write_cmos_sensor(0x2018, 0x004f);
write_cmos_sensor(0x201a, 0x0bcb);
write_cmos_sensor(0x201c, 0x008c); 
write_cmos_sensor(0x201e, 0x01cd); 
write_cmos_sensor(0x2020, 0x0063);
write_cmos_sensor(0x2022, 0x020e); 
write_cmos_sensor(0x2024, 0x7002);
write_cmos_sensor(0x2026, 0x8283); 
write_cmos_sensor(0x2028, 0x0158);
write_cmos_sensor(0x202a, 0x0024); 
write_cmos_sensor(0x202c, 0x0017); 
write_cmos_sensor(0x202e, 0x0ba4); 
write_cmos_sensor(0x2030, 0x0017); 
write_cmos_sensor(0x2032, 0x20e2); 
write_cmos_sensor(0x2034, 0x1604);
write_cmos_sensor(0x2036, 0x7010); 
write_cmos_sensor(0x2038, 0x13c4);
write_cmos_sensor(0x203a, 0x018f); 
write_cmos_sensor(0x203c, 0x0795); 
write_cmos_sensor(0x203e, 0x00d6); 
write_cmos_sensor(0x2040, 0x00d5); 
write_cmos_sensor(0x2042, 0x00d6); 
write_cmos_sensor(0x2044, 0x04d8); 
write_cmos_sensor(0x2046, 0x20e4); 
write_cmos_sensor(0x2048, 0x0017); 
write_cmos_sensor(0x204a, 0x7086); 
write_cmos_sensor(0x204c, 0x2fe2);
write_cmos_sensor(0x204e, 0x17d7); 
write_cmos_sensor(0x2050, 0x0024); 
write_cmos_sensor(0x2052, 0x8319); 
write_cmos_sensor(0x2054, 0x211c); 
write_cmos_sensor(0x2056, 0x2104); 
write_cmos_sensor(0x2058, 0x2223); 
write_cmos_sensor(0x205a, 0x2320);
write_cmos_sensor(0x205c, 0x7800); 
write_cmos_sensor(0x205e, 0x3104); 
write_cmos_sensor(0x2060, 0x01c0); 
write_cmos_sensor(0x2062, 0x01c3); 
write_cmos_sensor(0x2064, 0x01c4);
write_cmos_sensor(0x2066, 0x01c0); 
write_cmos_sensor(0x2068, 0x2704); 
write_cmos_sensor(0x206a, 0x3304); 
write_cmos_sensor(0x206c, 0x7800);
write_cmos_sensor(0x206e, 0x4031); 
write_cmos_sensor(0x2070, 0x83d2); 
write_cmos_sensor(0x2072, 0xb3e2);
write_cmos_sensor(0x2074, 0x00d0); 
write_cmos_sensor(0x2076, 0x2423); 
write_cmos_sensor(0x2078, 0xb3e2); 
write_cmos_sensor(0x207a, 0x00bd); 
write_cmos_sensor(0x207c, 0x2419); 
write_cmos_sensor(0x207e, 0x40b2);
write_cmos_sensor(0x2080, 0x1805); 
write_cmos_sensor(0x2082, 0x0b82); 
write_cmos_sensor(0x2084, 0x0c0a); 
write_cmos_sensor(0x2086, 0x40b2); 
write_cmos_sensor(0x2088, 0x3540); 
write_cmos_sensor(0x208a, 0x0b84); 
write_cmos_sensor(0x208c, 0x0c0a); 
write_cmos_sensor(0x208e, 0x40b2); 
write_cmos_sensor(0x2090, 0x3540);
write_cmos_sensor(0x2092, 0x0b86); 
write_cmos_sensor(0x2094, 0x0c0a); 
write_cmos_sensor(0x2096, 0x4382); 
write_cmos_sensor(0x2098, 0x0b88); 
write_cmos_sensor(0x209a, 0x0c0a);
write_cmos_sensor(0x209c, 0x4382); 
write_cmos_sensor(0x209e, 0x0b8a);
write_cmos_sensor(0x20a0, 0x0c0a); 
write_cmos_sensor(0x20a2, 0x4382); 
write_cmos_sensor(0x20a4, 0x0b8c); 
write_cmos_sensor(0x20a6, 0x0c0a);
write_cmos_sensor(0x20a8, 0x40b2); 
write_cmos_sensor(0x20aa, 0x0600); 
write_cmos_sensor(0x20ac, 0x0b8e); 
write_cmos_sensor(0x20ae, 0x0c0a); 
write_cmos_sensor(0x20b0, 0x40f2); 
write_cmos_sensor(0x20b2, 0x000a); 
write_cmos_sensor(0x20b4, 0x0f90);
write_cmos_sensor(0x20b6, 0x43c2); 
write_cmos_sensor(0x20b8, 0x0f82); 
write_cmos_sensor(0x20ba, 0xc3e2); 
write_cmos_sensor(0x20bc, 0x00d0);
write_cmos_sensor(0x20be, 0x0900); 
write_cmos_sensor(0x20c0, 0x7312); 
write_cmos_sensor(0x20c2, 0x4392);
write_cmos_sensor(0x20c4, 0x7326); 
write_cmos_sensor(0x20c6, 0xd3e2); 
write_cmos_sensor(0x20c8, 0x00d0); 
write_cmos_sensor(0x20ca, 0xb2e2); 
write_cmos_sensor(0x20cc, 0x00bd); 
write_cmos_sensor(0x20ce, 0x241c); 
write_cmos_sensor(0x20d0, 0x40b2); 
write_cmos_sensor(0x20d2, 0x9885); 
write_cmos_sensor(0x20d4, 0x0b82); 
write_cmos_sensor(0x20d6, 0x0c0a);
write_cmos_sensor(0x20d8, 0x40b2); 
write_cmos_sensor(0x20da, 0xc540);
write_cmos_sensor(0x20dc, 0x0b84); 
write_cmos_sensor(0x20de, 0x0c0a); 
write_cmos_sensor(0x20e0, 0x40b2); 
write_cmos_sensor(0x20e2, 0xb540); 
write_cmos_sensor(0x20e4, 0x0b86); 
write_cmos_sensor(0x20e6, 0x0c0a); 
write_cmos_sensor(0x20e8, 0x40b2);
write_cmos_sensor(0x20ea, 0xc085); 
write_cmos_sensor(0x20ec, 0x0b88); 
write_cmos_sensor(0x20ee, 0x0c0a); 
write_cmos_sensor(0x20f0, 0x40b2); 
write_cmos_sensor(0x20f2, 0xd701);
write_cmos_sensor(0x20f4, 0x0b8a); 
write_cmos_sensor(0x20f6, 0x0c0a); 
write_cmos_sensor(0x20f8, 0x40b2); 
write_cmos_sensor(0x20fa, 0x0420); 
write_cmos_sensor(0x20fc, 0x0b8c);
write_cmos_sensor(0x20fe, 0x0c0a); 
write_cmos_sensor(0x2100, 0x40b2); 
write_cmos_sensor(0x2102, 0xc600); 
write_cmos_sensor(0x2104, 0x0b8e); 
write_cmos_sensor(0x2106, 0x0c0a);
write_cmos_sensor(0x2108, 0x43d2); 
write_cmos_sensor(0x210a, 0x0f82); 
write_cmos_sensor(0x210c, 0x0cff); 
write_cmos_sensor(0x210e, 0x0cff); 
write_cmos_sensor(0x2110, 0x0cff);
write_cmos_sensor(0x2112, 0x0cff); 
write_cmos_sensor(0x2114, 0x0cff); 
write_cmos_sensor(0x2116, 0x0cff); 
write_cmos_sensor(0x2118, 0x0cff); 
write_cmos_sensor(0x211a, 0x0cff);
write_cmos_sensor(0x211c, 0x0cff); 
write_cmos_sensor(0x211e, 0x0cff); 
write_cmos_sensor(0x2120, 0x0cff); 
write_cmos_sensor(0x2122, 0x0cff); 
write_cmos_sensor(0x2124, 0x0cff);
write_cmos_sensor(0x2126, 0x0cff); 
write_cmos_sensor(0x2128, 0x0cff); 
write_cmos_sensor(0x212a, 0x0cff); 
write_cmos_sensor(0x212c, 0x0cff); 
write_cmos_sensor(0x212e, 0x0cff);
write_cmos_sensor(0x2130, 0x40f2); 
write_cmos_sensor(0x2132, 0x000e); 
write_cmos_sensor(0x2134, 0x0f90); 
write_cmos_sensor(0x2136, 0x4392); 
write_cmos_sensor(0x2138, 0x7326);
write_cmos_sensor(0x213a, 0x90f2); 
write_cmos_sensor(0x213c, 0x0010); 
write_cmos_sensor(0x213e, 0x00be); 
write_cmos_sensor(0x2140, 0x2002); 
write_cmos_sensor(0x2142, 0x4030);
write_cmos_sensor(0x2144, 0xf58c); 
write_cmos_sensor(0x2146, 0x4392); 
write_cmos_sensor(0x2148, 0x7f10); 
write_cmos_sensor(0x214a, 0x403f); 
write_cmos_sensor(0x214c, 0x0a27);
write_cmos_sensor(0x214e, 0x4f82); 
write_cmos_sensor(0x2150, 0x7100); 
write_cmos_sensor(0x2152, 0x0800); 
write_cmos_sensor(0x2154, 0x7f08); 
write_cmos_sensor(0x2156, 0x4382);
write_cmos_sensor(0x2158, 0x7f00); 
write_cmos_sensor(0x215a, 0x40b2); 
write_cmos_sensor(0x215c, 0xf05e); 
write_cmos_sensor(0x215e, 0x7f02); 
write_cmos_sensor(0x2160, 0x0900);
write_cmos_sensor(0x2162, 0x7f04); 
write_cmos_sensor(0x2164, 0x832f); 
write_cmos_sensor(0x2166, 0xb03f); 
write_cmos_sensor(0x2168, 0x8000); 
write_cmos_sensor(0x216a, 0x37f1);
write_cmos_sensor(0x216c, 0x4382); 
write_cmos_sensor(0x216e, 0x7f10); 
write_cmos_sensor(0x2170, 0x4392); 
write_cmos_sensor(0x2172, 0x807e); 
write_cmos_sensor(0x2174, 0x40b2);
write_cmos_sensor(0x2176, 0x02bc); 
write_cmos_sensor(0x2178, 0x731e); 
write_cmos_sensor(0x217a, 0x43a2); 
write_cmos_sensor(0x217c, 0x8092); 
write_cmos_sensor(0x217e, 0xb3e2);
write_cmos_sensor(0x2180, 0x00b4); 
write_cmos_sensor(0x2182, 0x2402); 
write_cmos_sensor(0x2184, 0x4392); 
write_cmos_sensor(0x2186, 0x8092); 
write_cmos_sensor(0x2188, 0x43a1);
write_cmos_sensor(0x218a, 0x002a); 
write_cmos_sensor(0x218c, 0xb3d2); 
write_cmos_sensor(0x218e, 0x00b4); 
write_cmos_sensor(0x2190, 0x2002); 
write_cmos_sensor(0x2192, 0x4030);
write_cmos_sensor(0x2194, 0xf57a); 
write_cmos_sensor(0x2196, 0x4381); 
write_cmos_sensor(0x2198, 0x002a); 
write_cmos_sensor(0x219a, 0x4382); 
write_cmos_sensor(0x219c, 0x82de);
write_cmos_sensor(0x219e, 0x43c2); 
write_cmos_sensor(0x21a0, 0x82b4); 
write_cmos_sensor(0x21a2, 0x43c2); 
write_cmos_sensor(0x21a4, 0x808b); 
write_cmos_sensor(0x21a6, 0x40b2);
write_cmos_sensor(0x21a8, 0x0005); 
write_cmos_sensor(0x21aa, 0x7320); 
write_cmos_sensor(0x21ac, 0x4392); 
write_cmos_sensor(0x21ae, 0x7326); 
write_cmos_sensor(0x21b0, 0x403e);
write_cmos_sensor(0x21b2, 0x00b2); 
write_cmos_sensor(0x21b4, 0x4e6f); 
write_cmos_sensor(0x21b6, 0xc312); 
write_cmos_sensor(0x21b8, 0x104f); 
write_cmos_sensor(0x21ba, 0x114f);
write_cmos_sensor(0x21bc, 0x114f); 
write_cmos_sensor(0x21be, 0x114f); 
write_cmos_sensor(0x21c0, 0x4f4d); 
write_cmos_sensor(0x21c2, 0x4e6f); 
write_cmos_sensor(0x21c4, 0xf07f);
write_cmos_sensor(0x21c6, 0x000f); 
write_cmos_sensor(0x21c8, 0xf37f); 
write_cmos_sensor(0x21ca, 0x5f0d); 
write_cmos_sensor(0x21cc, 0x403c); 
write_cmos_sensor(0x21ce, 0x00b3);
write_cmos_sensor(0x21d0, 0x4c6f); 
write_cmos_sensor(0x21d2, 0xc312); 
write_cmos_sensor(0x21d4, 0x104f); 
write_cmos_sensor(0x21d6, 0x114f); 
write_cmos_sensor(0x21d8, 0x114f);
write_cmos_sensor(0x21da, 0x114f); 
write_cmos_sensor(0x21dc, 0xf37f); 
write_cmos_sensor(0x21de, 0x5f0d); 
write_cmos_sensor(0x21e0, 0x4c6f); 
write_cmos_sensor(0x21e2, 0xf07f);
write_cmos_sensor(0x21e4, 0x000f); 
write_cmos_sensor(0x21e6, 0xf37f); 
write_cmos_sensor(0x21e8, 0x5f0d); 
write_cmos_sensor(0x21ea, 0x110d); 
write_cmos_sensor(0x21ec, 0x110d);
write_cmos_sensor(0x21ee, 0x4d82); 
write_cmos_sensor(0x21f0, 0x82a6); 
write_cmos_sensor(0x21f2, 0x4e6f); 
write_cmos_sensor(0x21f4, 0xf07f); 
write_cmos_sensor(0x21f6, 0x000f);
write_cmos_sensor(0x21f8, 0x4f4e); 
write_cmos_sensor(0x21fa, 0x4c6f); 
write_cmos_sensor(0x21fc, 0xf07f); 
write_cmos_sensor(0x21fe, 0x000f); 
write_cmos_sensor(0x2200, 0xf37f);
write_cmos_sensor(0x2202, 0x5f0e); 
write_cmos_sensor(0x2204, 0x533e); 
write_cmos_sensor(0x2206, 0x4e82); 
write_cmos_sensor(0x2208, 0x8090); 
write_cmos_sensor(0x220a, 0x4038);
write_cmos_sensor(0x220c, 0x0098); 
write_cmos_sensor(0x220e, 0x4039); 
write_cmos_sensor(0x2210, 0x0092); 
write_cmos_sensor(0x2212, 0x482f); 
write_cmos_sensor(0x2214, 0x892f);
write_cmos_sensor(0x2216, 0x531f); 
write_cmos_sensor(0x2218, 0x4f82); 
write_cmos_sensor(0x221a, 0x0a86); 
write_cmos_sensor(0x221c, 0x421f); 
write_cmos_sensor(0x221e, 0x00ac);
write_cmos_sensor(0x2220, 0x821f); 
write_cmos_sensor(0x2222, 0x00a6); 
write_cmos_sensor(0x2224, 0x531f); 
write_cmos_sensor(0x2226, 0x4f0c); 
write_cmos_sensor(0x2228, 0x4d0a);
write_cmos_sensor(0x222a, 0x12b0); 
write_cmos_sensor(0x222c, 0xff8a); 
write_cmos_sensor(0x222e, 0x4c82); 
write_cmos_sensor(0x2230, 0x0a88); 
write_cmos_sensor(0x2232, 0x40b2);
write_cmos_sensor(0x2234, 0x003a); 
write_cmos_sensor(0x2236, 0x8076); 
write_cmos_sensor(0x2238, 0x40b2); 
write_cmos_sensor(0x223a, 0x0034); 
write_cmos_sensor(0x223c, 0x7000);
write_cmos_sensor(0x223e, 0x49a2); 
write_cmos_sensor(0x2240, 0x0a8c); 
write_cmos_sensor(0x2242, 0x48a2); 
write_cmos_sensor(0x2244, 0x0a9e); 
write_cmos_sensor(0x2246, 0xb3e2);
write_cmos_sensor(0x2248, 0x0080); 
write_cmos_sensor(0x224a, 0x2402); 
write_cmos_sensor(0x224c, 0x4392); 
write_cmos_sensor(0x224e, 0x82de); 
write_cmos_sensor(0x2250, 0x43d2);
write_cmos_sensor(0x2252, 0x0180); 
write_cmos_sensor(0x2254, 0x12b0); 
write_cmos_sensor(0x2256, 0xfc14); 
write_cmos_sensor(0x2258, 0x430e); 
write_cmos_sensor(0x225a, 0x403f);
write_cmos_sensor(0x225c, 0x002e); 
write_cmos_sensor(0x225e, 0x12b0); 
write_cmos_sensor(0x2260, 0xfc26); 
write_cmos_sensor(0x2262, 0x4f82); 
write_cmos_sensor(0x2264, 0x8070);
write_cmos_sensor(0x2266, 0x430e); 
write_cmos_sensor(0x2268, 0x403f); 
write_cmos_sensor(0x226a, 0x002f); 
write_cmos_sensor(0x226c, 0x12b0); 
write_cmos_sensor(0x226e, 0xfc26);
write_cmos_sensor(0x2270, 0xf37f); 
write_cmos_sensor(0x2272, 0x108f); 
write_cmos_sensor(0x2274, 0xd21f); 
write_cmos_sensor(0x2276, 0x8070); 
write_cmos_sensor(0x2278, 0x4f82);
write_cmos_sensor(0x227a, 0x8070); 
write_cmos_sensor(0x227c, 0x930f); 
write_cmos_sensor(0x227e, 0x242b); 
write_cmos_sensor(0x2280, 0x430b); 
write_cmos_sensor(0x2282, 0x421f);
write_cmos_sensor(0x2284, 0x8070);
write_cmos_sensor(0x2286, 0x5392);
write_cmos_sensor(0x2288, 0x8070);
write_cmos_sensor(0x228a, 0x430e);
write_cmos_sensor(0x228c, 0x12b0);
write_cmos_sensor(0x228e, 0xfc26);
write_cmos_sensor(0x2290, 0x4fcb);
write_cmos_sensor(0x2292, 0x8072);
write_cmos_sensor(0x2294, 0x531b);
write_cmos_sensor(0x2296, 0x922b);
write_cmos_sensor(0x2298, 0x3bf4);
write_cmos_sensor(0x229a, 0x425d);
write_cmos_sensor(0x229c, 0x8075);
write_cmos_sensor(0x229e, 0x108d);
write_cmos_sensor(0x22a0, 0x425f);
write_cmos_sensor(0x22a2, 0x8074);
write_cmos_sensor(0x22a4, 0xdf0d);
write_cmos_sensor(0x22a6, 0x4d82);
write_cmos_sensor(0x22a8, 0x82a8);
write_cmos_sensor(0x22aa, 0x425b); 
write_cmos_sensor(0x22ac, 0x8073); 
write_cmos_sensor(0x22ae, 0x4b4e); 
write_cmos_sensor(0x22b0, 0x108e); 
write_cmos_sensor(0x22b2, 0x425c); 
write_cmos_sensor(0x22b4, 0x8072); 
write_cmos_sensor(0x22b6, 0x4c4f); 
write_cmos_sensor(0x22b8, 0xdf0e); 
write_cmos_sensor(0x22ba, 0x4e82); 
write_cmos_sensor(0x22bc, 0x82dc); 
write_cmos_sensor(0x22be, 0x930d); 
write_cmos_sensor(0x22c0, 0x2002); 
write_cmos_sensor(0x22c2, 0x930e); 
write_cmos_sensor(0x22c4, 0x2408); 
write_cmos_sensor(0x22c6, 0x934b); 
write_cmos_sensor(0x22c8, 0x2403); 
write_cmos_sensor(0x22ca, 0x4e8d); 
write_cmos_sensor(0x22cc, 0x0000); 
write_cmos_sensor(0x22ce, 0x3fd8); 
write_cmos_sensor(0x22d0, 0x4ccd); 
write_cmos_sensor(0x22d2, 0x0000); 
write_cmos_sensor(0x22d4, 0x3fd5); 
write_cmos_sensor(0x22d6, 0x4392); 
write_cmos_sensor(0x22d8, 0x7610); 
write_cmos_sensor(0x22da, 0x403e); 
write_cmos_sensor(0x22dc, 0x0030); 
write_cmos_sensor(0x22de, 0x403f); 
write_cmos_sensor(0x22e0, 0x0180); 
write_cmos_sensor(0x22e2, 0x12b0); 
write_cmos_sensor(0x22e4, 0xfdb4); 
write_cmos_sensor(0x22e6, 0x0261); 
write_cmos_sensor(0x22e8, 0x0000); 
write_cmos_sensor(0x22ea, 0x43c2); 
write_cmos_sensor(0x22ec, 0x0180); 
write_cmos_sensor(0x22ee, 0x4074); 
write_cmos_sensor(0x22f0, 0xff8b); 
write_cmos_sensor(0x22f2, 0x4392); 
write_cmos_sensor(0x22f4, 0x7326); 
write_cmos_sensor(0x22f6, 0x12b0); 
write_cmos_sensor(0x22f8, 0xf9b4);
write_cmos_sensor(0x22fa, 0x4392);
write_cmos_sensor(0x22fc, 0x7f06);
write_cmos_sensor(0x22fe, 0x43a2);
write_cmos_sensor(0x2300, 0x7f0a);	
write_cmos_sensor(0x2302, 0x0800); 
write_cmos_sensor(0x2304, 0x7f08);
write_cmos_sensor(0x2306, 0x40b2);
write_cmos_sensor(0x2308, 0x000e);
write_cmos_sensor(0x230a, 0x7f00);
write_cmos_sensor(0x230c, 0x40b2);
write_cmos_sensor(0x230e, 0xf000);
write_cmos_sensor(0x2310, 0x7f02);
write_cmos_sensor(0x2312, 0x0800);
write_cmos_sensor(0x2314, 0x7f08);
write_cmos_sensor(0x2316, 0x40b2);
write_cmos_sensor(0x2318, 0x01e9);
write_cmos_sensor(0x231a, 0x7f00);
write_cmos_sensor(0x231c, 0x40b2);
write_cmos_sensor(0x231e, 0xf000);
write_cmos_sensor(0x2320, 0x7f02);
write_cmos_sensor(0x2322, 0x4392);
write_cmos_sensor(0x2324, 0x731c);
write_cmos_sensor(0x2326, 0x9382);
write_cmos_sensor(0x2328, 0x807e);
write_cmos_sensor(0x232a, 0x240c);
write_cmos_sensor(0x232c, 0x430b);
write_cmos_sensor(0x232e, 0x4b0e);
write_cmos_sensor(0x2330, 0x5e0e);
write_cmos_sensor(0x2332, 0x4e0f);
write_cmos_sensor(0x2334, 0x510f);
write_cmos_sensor(0x2336, 0x4e9f);
write_cmos_sensor(0x2338, 0x0b00);
write_cmos_sensor(0x233a, 0x0000);
write_cmos_sensor(0x233c, 0x531b);
write_cmos_sensor(0x233e, 0x903b);
write_cmos_sensor(0x2340, 0x0016);
write_cmos_sensor(0x2342, 0x3bf5);
write_cmos_sensor(0x2344, 0xb2e2);
write_cmos_sensor(0x2346, 0x00d0);
write_cmos_sensor(0x2348, 0x240f);
write_cmos_sensor(0x234a, 0x430b);
write_cmos_sensor(0x234c, 0x903b);
write_cmos_sensor(0x234e, 0x0009);
write_cmos_sensor(0x2350, 0x2406);
write_cmos_sensor(0x2352, 0x4b0e);
write_cmos_sensor(0x2354, 0x5e0e);
write_cmos_sensor(0x2356, 0x4e0f);
write_cmos_sensor(0x2358, 0x510f);
write_cmos_sensor(0x235a, 0x4fae);
write_cmos_sensor(0x235c, 0x0b80);
write_cmos_sensor(0x235e, 0x0c0a);
write_cmos_sensor(0x2360, 0x531b);
write_cmos_sensor(0x2362, 0x903b);
write_cmos_sensor(0x2364, 0x0016);
write_cmos_sensor(0x2366, 0x3bf2);
write_cmos_sensor(0x2368, 0x9382);
write_cmos_sensor(0x236a, 0x807e);
write_cmos_sensor(0x236c, 0x2007);
write_cmos_sensor(0x236e, 0x0b00);
write_cmos_sensor(0x2370, 0x7302);
write_cmos_sensor(0x2372, 0x0258);
write_cmos_sensor(0x2374, 0x0900);
write_cmos_sensor(0x2376, 0x7308);
write_cmos_sensor(0x2378, 0x12b0);
write_cmos_sensor(0x237a, 0xf9b4);
write_cmos_sensor(0x237c, 0x4382);
write_cmos_sensor(0x237e, 0x8078);
write_cmos_sensor(0x2380, 0x12b0);
write_cmos_sensor(0x2382, 0xf900);
write_cmos_sensor(0x2384, 0x42b2);
write_cmos_sensor(0x2386, 0x7706);
write_cmos_sensor(0x2388, 0x0900);
write_cmos_sensor(0x238a, 0x7328);
write_cmos_sensor(0x238c, 0x4192);
write_cmos_sensor(0x238e, 0x002a);
write_cmos_sensor(0x2390, 0x7114);
write_cmos_sensor(0x2392, 0x4392);
write_cmos_sensor(0x2394, 0x7f0c);
write_cmos_sensor(0x2396, 0x4392);
write_cmos_sensor(0x2398, 0x7f10);
write_cmos_sensor(0x239a, 0x4392);
write_cmos_sensor(0x239c, 0x770a);
write_cmos_sensor(0x239e, 0x4392);
write_cmos_sensor(0x23a0, 0x770e);
write_cmos_sensor(0x23a2, 0x40b2);
write_cmos_sensor(0x23a4, 0x0006);
write_cmos_sensor(0x23a6, 0x805c);
write_cmos_sensor(0x23a8, 0x9392);
write_cmos_sensor(0x23aa, 0x7114);
write_cmos_sensor(0x23ac, 0x2061);
write_cmos_sensor(0x23ae, 0x0b00);
write_cmos_sensor(0x23b0, 0x7302);
write_cmos_sensor(0x23b2, 0x0258);
write_cmos_sensor(0x23b4, 0x0800);
write_cmos_sensor(0x23b6, 0x7118);
write_cmos_sensor(0x23b8, 0x421e);
write_cmos_sensor(0x23ba, 0x0cb4);
write_cmos_sensor(0x23bc, 0x421f);
write_cmos_sensor(0x23be, 0x0cb2);
write_cmos_sensor(0x23c0, 0x12b0);
write_cmos_sensor(0x23c2, 0xfc68);
write_cmos_sensor(0x23c4, 0x1230);
write_cmos_sensor(0x23c6, 0x0800);
write_cmos_sensor(0x23c8, 0x1230);
write_cmos_sensor(0x23ca, 0x0cce);
write_cmos_sensor(0x23cc, 0x403c);
write_cmos_sensor(0x23ce, 0x82b2);
write_cmos_sensor(0x23d0, 0x421d);
write_cmos_sensor(0x23d2, 0x0caa);
write_cmos_sensor(0x23d4, 0x12b0);
write_cmos_sensor(0x23d6, 0xf816);
write_cmos_sensor(0x23d8, 0x4f82);
write_cmos_sensor(0x23da, 0x0c9a);
write_cmos_sensor(0x23dc, 0x421e);
write_cmos_sensor(0x23de, 0x0cb8);
write_cmos_sensor(0x23e0, 0x421f);
write_cmos_sensor(0x23e2, 0x0cb6);
write_cmos_sensor(0x23e4, 0x12b0);
write_cmos_sensor(0x23e6, 0xfc68);
write_cmos_sensor(0x23e8, 0x1230);
write_cmos_sensor(0x23ea, 0x0800);
write_cmos_sensor(0x23ec, 0x1230);
write_cmos_sensor(0x23ee, 0x0cd0);
write_cmos_sensor(0x23f0, 0x403c);
write_cmos_sensor(0x23f2, 0x82e8);
write_cmos_sensor(0x23f4, 0x421d);
write_cmos_sensor(0x23f6, 0x0cac);
write_cmos_sensor(0x23f8, 0x12b0);
write_cmos_sensor(0x23fa, 0xf816);
write_cmos_sensor(0x23fc, 0x4f82);
write_cmos_sensor(0x23fe, 0x0c9c);
write_cmos_sensor(0x2400, 0x421e);
write_cmos_sensor(0x2402, 0x0cbc);
write_cmos_sensor(0x2404, 0x421f);
write_cmos_sensor(0x2406, 0x0cba);
write_cmos_sensor(0x2408, 0x12b0);
write_cmos_sensor(0x240a, 0xfc68);
write_cmos_sensor(0x240c, 0x1230);
write_cmos_sensor(0x240e, 0x0800);
write_cmos_sensor(0x2410, 0x1230);
write_cmos_sensor(0x2412, 0x0cd2);
write_cmos_sensor(0x2414, 0x403c);
write_cmos_sensor(0x2416, 0x82ba);
write_cmos_sensor(0x2418, 0x421d);
write_cmos_sensor(0x241a, 0x0cae);
write_cmos_sensor(0x241c, 0x12b0);
write_cmos_sensor(0x241e, 0xf816);
write_cmos_sensor(0x2420, 0x4f82);
write_cmos_sensor(0x2422, 0x0c9e);
write_cmos_sensor(0x2424, 0x421e);
write_cmos_sensor(0x2426, 0x0cc0);
write_cmos_sensor(0x2428, 0x421f);
write_cmos_sensor(0x242a, 0x0cbe);
write_cmos_sensor(0x242c, 0x12b0);
write_cmos_sensor(0x242e, 0xfc68);
write_cmos_sensor(0x2430, 0x1230);
write_cmos_sensor(0x2432, 0x0800);
write_cmos_sensor(0x2434, 0x1230);
write_cmos_sensor(0x2436, 0x0cd4);
write_cmos_sensor(0x2438, 0x403c);
write_cmos_sensor(0x243a, 0x8096);
write_cmos_sensor(0x243c, 0x421d);
write_cmos_sensor(0x243e, 0x0cb0);
write_cmos_sensor(0x2440, 0x12b0);
write_cmos_sensor(0x2442, 0xf816);
write_cmos_sensor(0x2444, 0x4f82);
write_cmos_sensor(0x2446, 0x0ca0);
write_cmos_sensor(0x2448, 0x425f);
write_cmos_sensor(0x244a, 0x0c80);
write_cmos_sensor(0x244c, 0xf35f);
write_cmos_sensor(0x244e, 0x5031);
write_cmos_sensor(0x2450, 0x0010);
write_cmos_sensor(0x2452, 0x934f);
write_cmos_sensor(0x2454, 0x2008);
write_cmos_sensor(0x2456, 0x4382);
write_cmos_sensor(0x2458, 0x0cce);
write_cmos_sensor(0x245a, 0x4382);
write_cmos_sensor(0x245c, 0x0cd0);
write_cmos_sensor(0x245e, 0x4382);
write_cmos_sensor(0x2460, 0x0cd2);
write_cmos_sensor(0x2462, 0x4382);
write_cmos_sensor(0x2464, 0x0cd4);
write_cmos_sensor(0x2466, 0x0900);
write_cmos_sensor(0x2468, 0x7112);
write_cmos_sensor(0x246a, 0x12b0);
write_cmos_sensor(0x246c, 0xf788);
write_cmos_sensor(0x246e, 0x3f9c);
write_cmos_sensor(0x2470, 0x0b00);
write_cmos_sensor(0x2472, 0x7302);
write_cmos_sensor(0x2474, 0x0000);
write_cmos_sensor(0x2476, 0x93a2);
write_cmos_sensor(0x2478, 0x7114);
write_cmos_sensor(0x247a, 0x2407);
write_cmos_sensor(0x247c, 0x12b0);
write_cmos_sensor(0x247e, 0xf788);
write_cmos_sensor(0x2480, 0x930f);
write_cmos_sensor(0x2482, 0x2792);
write_cmos_sensor(0x2484, 0x4382);
write_cmos_sensor(0x2486, 0x807e);
write_cmos_sensor(0x2488, 0x3f4e);
write_cmos_sensor(0x248a, 0x4307);
write_cmos_sensor(0x248c, 0x4308);
write_cmos_sensor(0x248e, 0x4306);
write_cmos_sensor(0x2490, 0x421b);
write_cmos_sensor(0x2492, 0x7100);
write_cmos_sensor(0x2494, 0x503b);
write_cmos_sensor(0x2496, 0xffc0);
write_cmos_sensor(0x2498, 0x4b0d);
write_cmos_sensor(0x249a, 0xf31d);
write_cmos_sensor(0x249c, 0x4d81);
write_cmos_sensor(0x249e, 0x002c);
write_cmos_sensor(0x24a0, 0x4b05);
write_cmos_sensor(0x24a2, 0xf325);
write_cmos_sensor(0x24a4, 0x4039);
write_cmos_sensor(0x24a6, 0x0304);
write_cmos_sensor(0x24a8, 0x421d);
write_cmos_sensor(0x24aa, 0x8078);
write_cmos_sensor(0x24ac, 0x4d0f);
write_cmos_sensor(0x24ae, 0x521f);
write_cmos_sensor(0x24b0, 0x805c);
write_cmos_sensor(0x24b2, 0x9f0d);
write_cmos_sensor(0x24b4, 0x2c17);
write_cmos_sensor(0x24b6, 0x4d0f);
write_cmos_sensor(0x24b8, 0x5f0f);
write_cmos_sensor(0x24ba, 0x5f0f);
write_cmos_sensor(0x24bc, 0x4f0c);
write_cmos_sensor(0x24be, 0x503c);
write_cmos_sensor(0x24c0, 0x80a6);
write_cmos_sensor(0x24c2, 0x4f1a);
write_cmos_sensor(0x24c4, 0x80a6);
write_cmos_sensor(0x24c6, 0x4a0e);
write_cmos_sensor(0x24c8, 0xf03e);
write_cmos_sensor(0x24ca, 0x0fff);
write_cmos_sensor(0x24cc, 0x4b0f);
write_cmos_sensor(0x24ce, 0xf03f);
write_cmos_sensor(0x24d0, 0x0fff);
write_cmos_sensor(0x24d2, 0x9f0e);
write_cmos_sensor(0x24d4, 0x2445);
write_cmos_sensor(0x24d6, 0x531d);
write_cmos_sensor(0x24d8, 0x421f);
write_cmos_sensor(0x24da, 0x8078);
write_cmos_sensor(0x24dc, 0x521f);
write_cmos_sensor(0x24de, 0x805c);
write_cmos_sensor(0x24e0, 0x9f0d);
write_cmos_sensor(0x24e2, 0x2be9);
write_cmos_sensor(0x24e4, 0x9192);
write_cmos_sensor(0x24e6, 0x002c);
write_cmos_sensor(0x24e8, 0x82de);
write_cmos_sensor(0x24ea, 0x2437);
write_cmos_sensor(0x24ec, 0x5782);
write_cmos_sensor(0x24ee, 0x8078);
write_cmos_sensor(0x24f0, 0x421c);
write_cmos_sensor(0x24f2, 0x805c);
write_cmos_sensor(0x24f4, 0x421e);
write_cmos_sensor(0x24f6, 0x8078);
write_cmos_sensor(0x24f8, 0x5c0e);
write_cmos_sensor(0x24fa, 0x421d);
write_cmos_sensor(0x24fc, 0x8084);
write_cmos_sensor(0x24fe, 0x4d0f);
write_cmos_sensor(0x2500, 0x533f);
write_cmos_sensor(0x2502, 0x9e0f);
write_cmos_sensor(0x2504, 0x2c05);
write_cmos_sensor(0x2506, 0x9d0c);
write_cmos_sensor(0x2508, 0x2c03);
write_cmos_sensor(0x250a, 0x8c0d);
write_cmos_sensor(0x250c, 0x4d82);
write_cmos_sensor(0x250e, 0x8078);
write_cmos_sensor(0x2510, 0x4682);
write_cmos_sensor(0x2512, 0x039c);
write_cmos_sensor(0x2514, 0xb0b2);
write_cmos_sensor(0x2516, 0x000f);
write_cmos_sensor(0x2518, 0x7300);
write_cmos_sensor(0x251a, 0x23b0);
write_cmos_sensor(0x251c, 0x444e);
write_cmos_sensor(0x251e, 0xc312);
write_cmos_sensor(0x2520, 0x104e);
write_cmos_sensor(0x2522, 0x444f);
write_cmos_sensor(0x2524, 0xf35f);
write_cmos_sensor(0x2526, 0xe37f);
write_cmos_sensor(0x2528, 0x535f);
write_cmos_sensor(0x252a, 0xf07f);
write_cmos_sensor(0x252c, 0xffb8);
write_cmos_sensor(0x252e, 0x4e4d);
write_cmos_sensor(0x2530, 0xef4d);
write_cmos_sensor(0x2532, 0x4dc2);
write_cmos_sensor(0x2534, 0x0cda);
write_cmos_sensor(0x2536, 0x4d44);
write_cmos_sensor(0x2538, 0x4dc2);
write_cmos_sensor(0x253a, 0x0cdb);
write_cmos_sensor(0x253c, 0x4d4f);
write_cmos_sensor(0x253e, 0xe37f);
write_cmos_sensor(0x2540, 0x4fc2);
write_cmos_sensor(0x2542, 0x0cdc);
write_cmos_sensor(0x2544, 0x4fc2);
write_cmos_sensor(0x2546, 0x0cdd);
write_cmos_sensor(0x2548, 0x4fc2);
write_cmos_sensor(0x254a, 0x0cde);
write_cmos_sensor(0x254c, 0x4fc2);
write_cmos_sensor(0x254e, 0x0cdf);
write_cmos_sensor(0x2550, 0x44c2);
write_cmos_sensor(0x2552, 0x0ce0);
write_cmos_sensor(0x2554, 0x44c2);
write_cmos_sensor(0x2556, 0x0ce1);
write_cmos_sensor(0x2558, 0x3f91);
write_cmos_sensor(0x255a, 0x5882);
write_cmos_sensor(0x255c, 0x8078);
write_cmos_sensor(0x255e, 0x3fc8);
write_cmos_sensor(0x2560, 0x4c99);
write_cmos_sensor(0x2562, 0x0002);
write_cmos_sensor(0x2564, 0x0000);
write_cmos_sensor(0x2566, 0x5606);
write_cmos_sensor(0x2568, 0xd316);
write_cmos_sensor(0x256a, 0x5329);
write_cmos_sensor(0x256c, 0x5317);
write_cmos_sensor(0x256e, 0x4a0f);
write_cmos_sensor(0x2570, 0xf32f);
write_cmos_sensor(0x2572, 0x950f);
write_cmos_sensor(0x2574, 0x23b0);
write_cmos_sensor(0x2576, 0x5318);
write_cmos_sensor(0x2578, 0x3fae);
write_cmos_sensor(0x257a, 0xb3e2);
write_cmos_sensor(0x257c, 0x00b4);
write_cmos_sensor(0x257e, 0x2002);
write_cmos_sensor(0x2580, 0x4030);
write_cmos_sensor(0x2582, 0xf19a);
write_cmos_sensor(0x2584, 0x4391);
write_cmos_sensor(0x2586, 0x002a);
write_cmos_sensor(0x2588, 0x4030);
write_cmos_sensor(0x258a, 0xf19a);
write_cmos_sensor(0x258c, 0x43d2);
write_cmos_sensor(0x258e, 0x0180);
write_cmos_sensor(0x2590, 0x4392);
write_cmos_sensor(0x2592, 0x760e);
write_cmos_sensor(0x2594, 0x9382);
write_cmos_sensor(0x2596, 0x760c);
write_cmos_sensor(0x2598, 0x2002);
write_cmos_sensor(0x259a, 0x0c64);
write_cmos_sensor(0x259c, 0x3ffb);
write_cmos_sensor(0x259e, 0x421f);
write_cmos_sensor(0x25a0, 0x760a);
write_cmos_sensor(0x25a2, 0x932f);
write_cmos_sensor(0x25a4, 0x2017);
write_cmos_sensor(0x25a6, 0x4292);
write_cmos_sensor(0x25a8, 0x018a);
write_cmos_sensor(0x25aa, 0x809c);
write_cmos_sensor(0x25ac, 0x4292);
write_cmos_sensor(0x25ae, 0x809c);
write_cmos_sensor(0x25b0, 0x7600);
write_cmos_sensor(0x25b2, 0x12b0);
write_cmos_sensor(0x25b4, 0xfc14);
write_cmos_sensor(0x25b6, 0x90b2);
write_cmos_sensor(0x25b8, 0x1000);
write_cmos_sensor(0x25ba, 0x809c);
write_cmos_sensor(0x25bc, 0x2808);
write_cmos_sensor(0x25be, 0x403f);
write_cmos_sensor(0x25c0, 0x8028);
write_cmos_sensor(0x25c2, 0x12b0);
write_cmos_sensor(0x25c4, 0xfc3e);
write_cmos_sensor(0x25c6, 0x4292);
write_cmos_sensor(0x25c8, 0x809c);
write_cmos_sensor(0x25ca, 0x80a4);
write_cmos_sensor(0x25cc, 0x3fe1);
write_cmos_sensor(0x25ce, 0x403f);
write_cmos_sensor(0x25d0, 0x0028);
write_cmos_sensor(0x25d2, 0x3ff7);
write_cmos_sensor(0x25d4, 0x903f);
write_cmos_sensor(0x25d6, 0x0003);
write_cmos_sensor(0x25d8, 0x28cd);
write_cmos_sensor(0x25da, 0x903f);
write_cmos_sensor(0x25dc, 0x0102);
write_cmos_sensor(0x25de, 0x20a2);
write_cmos_sensor(0x25e0, 0x43c2);
write_cmos_sensor(0x25e2, 0x018c);
write_cmos_sensor(0x25e4, 0x425f);
write_cmos_sensor(0x25e6, 0x0186);
write_cmos_sensor(0x25e8, 0x4fc2);
write_cmos_sensor(0x25ea, 0x807a);
write_cmos_sensor(0x25ec, 0x43c2);
write_cmos_sensor(0x25ee, 0x807b);
write_cmos_sensor(0x25f0, 0x93d2);
write_cmos_sensor(0x25f2, 0x018f);
write_cmos_sensor(0x25f4, 0x2495);
write_cmos_sensor(0x25f6, 0x425f);
write_cmos_sensor(0x25f8, 0x018f);
write_cmos_sensor(0x25fa, 0xf37f);
write_cmos_sensor(0x25fc, 0x4f82);
write_cmos_sensor(0x25fe, 0x82b6);
write_cmos_sensor(0x2600, 0x421e);
write_cmos_sensor(0x2602, 0x809c);
write_cmos_sensor(0x2604, 0x108e);
write_cmos_sensor(0x2606, 0xf37e);
write_cmos_sensor(0x2608, 0xc312);
write_cmos_sensor(0x260a, 0x100e);
write_cmos_sensor(0x260c, 0x110e);
write_cmos_sensor(0x260e, 0x110e);
write_cmos_sensor(0x2610, 0x110e);
write_cmos_sensor(0x2612, 0x421f);
write_cmos_sensor(0x2614, 0x80a4);
write_cmos_sensor(0x2616, 0x108f);
write_cmos_sensor(0x2618, 0xf37f);
write_cmos_sensor(0x261a, 0xc312);
write_cmos_sensor(0x261c, 0x100f);
write_cmos_sensor(0x261e, 0x110f);
write_cmos_sensor(0x2620, 0x110f);
write_cmos_sensor(0x2622, 0x110f);
write_cmos_sensor(0x2624, 0x9f0e);
write_cmos_sensor(0x2626, 0x240f);
write_cmos_sensor(0x2628, 0x0261);
write_cmos_sensor(0x262a, 0x0000);
write_cmos_sensor(0x262c, 0x4292);
write_cmos_sensor(0x262e, 0x809c);
write_cmos_sensor(0x2630, 0x7600);
write_cmos_sensor(0x2632, 0x12b0);
write_cmos_sensor(0x2634, 0xfc14);
write_cmos_sensor(0x2636, 0x90b2);
write_cmos_sensor(0x2638, 0x1000);
write_cmos_sensor(0x263a, 0x809c);
write_cmos_sensor(0x263c, 0x286e);
write_cmos_sensor(0x263e, 0x403f);
write_cmos_sensor(0x2640, 0x8028);
write_cmos_sensor(0x2642, 0x12b0);
write_cmos_sensor(0x2644, 0xfc3e);
write_cmos_sensor(0x2646, 0x430b);
write_cmos_sensor(0x2648, 0x4219);
write_cmos_sensor(0x264a, 0x807a);
write_cmos_sensor(0x264c, 0x421c);
write_cmos_sensor(0x264e, 0x8056);
write_cmos_sensor(0x2650, 0x4218);
write_cmos_sensor(0x2652, 0x82b6);
write_cmos_sensor(0x2654, 0x421a);
write_cmos_sensor(0x2656, 0x809c);
write_cmos_sensor(0x2658, 0x431f);
write_cmos_sensor(0x265a, 0x4b0e);
write_cmos_sensor(0x265c, 0x930e);
write_cmos_sensor(0x265e, 0x2403);
write_cmos_sensor(0x2660, 0x5f0f);
write_cmos_sensor(0x2662, 0x831e);
write_cmos_sensor(0x2664, 0x23fd);
write_cmos_sensor(0x2666, 0xf90f);
write_cmos_sensor(0x2668, 0x244b);
write_cmos_sensor(0x266a, 0x430c);
write_cmos_sensor(0x266c, 0x480e);
write_cmos_sensor(0x266e, 0x9318);
write_cmos_sensor(0x2670, 0x282f);
write_cmos_sensor(0x2672, 0x4a0c);
write_cmos_sensor(0x2674, 0x490d);
write_cmos_sensor(0x2676, 0x430f);
write_cmos_sensor(0x2678, 0x4c82);
write_cmos_sensor(0x267a, 0x7600);
write_cmos_sensor(0x267c, 0x4b82);
write_cmos_sensor(0x267e, 0x7602);
write_cmos_sensor(0x2680, 0x4d82);
write_cmos_sensor(0x2682, 0x7604);
write_cmos_sensor(0x2684, 0x0264);
write_cmos_sensor(0x2686, 0x0000);
write_cmos_sensor(0x2688, 0x0224);
write_cmos_sensor(0x268a, 0x0000);
write_cmos_sensor(0x268c, 0x0264);
write_cmos_sensor(0x268e, 0x0000);
write_cmos_sensor(0x2690, 0x0260);
write_cmos_sensor(0x2692, 0x0000);
write_cmos_sensor(0x2694, 0x0268);
write_cmos_sensor(0x2696, 0x0000);
write_cmos_sensor(0x2698, 0x0c47);
write_cmos_sensor(0x269a, 0x02e8);
write_cmos_sensor(0x269c, 0x0000);
write_cmos_sensor(0x269e, 0x0c90);
write_cmos_sensor(0x26a0, 0x02a8);
write_cmos_sensor(0x26a2, 0x0000);
write_cmos_sensor(0x26a4, 0x0c8e);
write_cmos_sensor(0x26a6, 0x0c8e);
write_cmos_sensor(0x26a8, 0x0c8e);
write_cmos_sensor(0x26aa, 0x0c8e);
write_cmos_sensor(0x26ac, 0x0c8e);
write_cmos_sensor(0x26ae, 0x0c8e);
write_cmos_sensor(0x26b0, 0x0c8e);
write_cmos_sensor(0x26b2, 0x0c8e);
write_cmos_sensor(0x26b4, 0x0c00);
write_cmos_sensor(0x26b6, 0x02e8);
write_cmos_sensor(0x26b8, 0x0000);
write_cmos_sensor(0x26ba, 0x0c8e);
write_cmos_sensor(0x26bc, 0x0268);
write_cmos_sensor(0x26be, 0x0000);
write_cmos_sensor(0x26c0, 0x0c51);
write_cmos_sensor(0x26c2, 0x0260);
write_cmos_sensor(0x26c4, 0x0000);
write_cmos_sensor(0x26c6, 0x0c51);
write_cmos_sensor(0x26c8, 0x531f);
write_cmos_sensor(0x26ca, 0x9e0f);
write_cmos_sensor(0x26cc, 0x2bd5);
write_cmos_sensor(0x26ce, 0x4f0c);
write_cmos_sensor(0x26d0, 0x4b82);
write_cmos_sensor(0x26d2, 0x7602);
write_cmos_sensor(0x26d4, 0x4a82);
write_cmos_sensor(0x26d6, 0x7600);
write_cmos_sensor(0x26d8, 0x0270);
write_cmos_sensor(0x26da, 0x0000);
write_cmos_sensor(0x26dc, 0x0c18);
write_cmos_sensor(0x26de, 0x0270);
write_cmos_sensor(0x26e0, 0x0001);
write_cmos_sensor(0x26e2, 0x421f);
write_cmos_sensor(0x26e4, 0x7606);
write_cmos_sensor(0x26e6, 0x4f4e);
write_cmos_sensor(0x26e8, 0x431f);
write_cmos_sensor(0x26ea, 0x4b0d);
write_cmos_sensor(0x26ec, 0x930d);
write_cmos_sensor(0x26ee, 0x2403);
write_cmos_sensor(0x26f0, 0x5f0f);
write_cmos_sensor(0x26f2, 0x831d);
write_cmos_sensor(0x26f4, 0x23fd);
write_cmos_sensor(0x26f6, 0xff4e);
write_cmos_sensor(0x26f8, 0xdec2);
write_cmos_sensor(0x26fa, 0x018c);
write_cmos_sensor(0x26fc, 0x0260);
write_cmos_sensor(0x26fe, 0x0000);
write_cmos_sensor(0x2700, 0x531b);
write_cmos_sensor(0x2702, 0x923b);
write_cmos_sensor(0x2704, 0x3ba9);
write_cmos_sensor(0x2706, 0x4c82);
write_cmos_sensor(0x2708, 0x8056);
write_cmos_sensor(0x270a, 0x0260);
write_cmos_sensor(0x270c, 0x0000);
write_cmos_sensor(0x270e, 0x4292);
write_cmos_sensor(0x2710, 0x809c);
write_cmos_sensor(0x2712, 0x80a4);
write_cmos_sensor(0x2714, 0x5392);
write_cmos_sensor(0x2716, 0x809c);
write_cmos_sensor(0x2718, 0x3f3b);
write_cmos_sensor(0x271a, 0x403f);
write_cmos_sensor(0x271c, 0x0028);
write_cmos_sensor(0x271e, 0x3f91);
write_cmos_sensor(0x2720, 0x432f);
write_cmos_sensor(0x2722, 0x3f6c);
write_cmos_sensor(0x2724, 0x903f);
write_cmos_sensor(0x2726, 0x0201);
write_cmos_sensor(0x2728, 0x2333);
write_cmos_sensor(0x272a, 0x5392);
write_cmos_sensor(0x272c, 0x809c);
write_cmos_sensor(0x272e, 0x421e);
write_cmos_sensor(0x2730, 0x809c);
write_cmos_sensor(0x2732, 0x108e);
write_cmos_sensor(0x2734, 0xf37e);
write_cmos_sensor(0x2736, 0xc312);
write_cmos_sensor(0x2738, 0x100e);
write_cmos_sensor(0x273a, 0x110e);
write_cmos_sensor(0x273c, 0x110e);
write_cmos_sensor(0x273e, 0x110e);
write_cmos_sensor(0x2740, 0x421f);
write_cmos_sensor(0x2742, 0x80a4);
write_cmos_sensor(0x2744, 0x108f);
write_cmos_sensor(0x2746, 0xf37f);
write_cmos_sensor(0x2748, 0xc312);
write_cmos_sensor(0x274a, 0x100f);
write_cmos_sensor(0x274c, 0x110f);
write_cmos_sensor(0x274e, 0x110f);
write_cmos_sensor(0x2750, 0x110f);
write_cmos_sensor(0x2752, 0x9f0e);
write_cmos_sensor(0x2754, 0x2407);
write_cmos_sensor(0x2756, 0x0261);
write_cmos_sensor(0x2758, 0x0000);
write_cmos_sensor(0x275a, 0x4292);
write_cmos_sensor(0x275c, 0x809c);
write_cmos_sensor(0x275e, 0x7600);
write_cmos_sensor(0x2760, 0x12b0);
write_cmos_sensor(0x2762, 0xfc14);
write_cmos_sensor(0x2764, 0x430e);
write_cmos_sensor(0x2766, 0x421f);
write_cmos_sensor(0x2768, 0x809c);
write_cmos_sensor(0x276a, 0x12b0);
write_cmos_sensor(0x276c, 0xfc26);
write_cmos_sensor(0x276e, 0x4fc2);
write_cmos_sensor(0x2770, 0x0188);
write_cmos_sensor(0x2772, 0x3f29);
write_cmos_sensor(0x2774, 0x931f);
write_cmos_sensor(0x2776, 0x230c);
write_cmos_sensor(0x2778, 0x4292);
write_cmos_sensor(0x277a, 0x018a);
write_cmos_sensor(0x277c, 0x809c);
write_cmos_sensor(0x277e, 0x3fed);
write_cmos_sensor(0x2780, 0x5031);
write_cmos_sensor(0x2782, 0x002e);
write_cmos_sensor(0x2784, 0x4030);
write_cmos_sensor(0x2786, 0xff62);
write_cmos_sensor(0x2788, 0x421f);
write_cmos_sensor(0x278a, 0x7316);
write_cmos_sensor(0x278c, 0xc312);
write_cmos_sensor(0x278e, 0x100f);
write_cmos_sensor(0x2790, 0x110f);
write_cmos_sensor(0x2792, 0x503f);
write_cmos_sensor(0x2794, 0xffab);
write_cmos_sensor(0x2796, 0x4f82);
write_cmos_sensor(0x2798, 0x7334);
write_cmos_sensor(0x279a, 0x0f00);
write_cmos_sensor(0x279c, 0x7302);
write_cmos_sensor(0x279e, 0x421f);
write_cmos_sensor(0x27a0, 0x7100);
write_cmos_sensor(0x27a2, 0x9382);
write_cmos_sensor(0x27a4, 0x7112);
write_cmos_sensor(0x27a6, 0x2406);
write_cmos_sensor(0x27a8, 0x93a2);
write_cmos_sensor(0x27aa, 0x7114);
write_cmos_sensor(0x27ac, 0x2423);
write_cmos_sensor(0x27ae, 0x5292);
write_cmos_sensor(0x27b0, 0x8092);
write_cmos_sensor(0x27b2, 0x7114);
write_cmos_sensor(0x27b4, 0x921f);
write_cmos_sensor(0x27b6, 0x82aa);
write_cmos_sensor(0x27b8, 0x241a);
write_cmos_sensor(0x27ba, 0x921f);
write_cmos_sensor(0x27bc, 0x82b0);
write_cmos_sensor(0x27be, 0x2414);
write_cmos_sensor(0x27c0, 0x921f);
write_cmos_sensor(0x27c2, 0x807c);
write_cmos_sensor(0x27c4, 0x240e);
write_cmos_sensor(0x27c6, 0x921f);
write_cmos_sensor(0x27c8, 0x8054);
write_cmos_sensor(0x27ca, 0x2408);
write_cmos_sensor(0x27cc, 0xb0b2);
write_cmos_sensor(0x27ce, 0x000f);
write_cmos_sensor(0x27d0, 0x7300);
write_cmos_sensor(0x27d2, 0x2002);
write_cmos_sensor(0x27d4, 0x43d2);
write_cmos_sensor(0x27d6, 0x0cd8);
write_cmos_sensor(0x27d8, 0x430f);
write_cmos_sensor(0x27da, 0x4130);
write_cmos_sensor(0x27dc, 0x4382);
write_cmos_sensor(0x27de, 0x7004);
write_cmos_sensor(0x27e0, 0x3ff5);
write_cmos_sensor(0x27e2, 0x4392);
write_cmos_sensor(0x27e4, 0x7004);
write_cmos_sensor(0x27e6, 0x3fef);
write_cmos_sensor(0x27e8, 0x4382);
write_cmos_sensor(0x27ea, 0x7004);
write_cmos_sensor(0x27ec, 0x3fe9);
write_cmos_sensor(0x27ee, 0x4392);
write_cmos_sensor(0x27f0, 0x7004);
write_cmos_sensor(0x27f2, 0x3fe3);
write_cmos_sensor(0x27f4, 0x4382);
write_cmos_sensor(0x27f6, 0x7f0c);
write_cmos_sensor(0x27f8, 0x4382);
write_cmos_sensor(0x27fa, 0x7f10);
write_cmos_sensor(0x27fc, 0x40b2);
write_cmos_sensor(0x27fe, 0x0003);
write_cmos_sensor(0x2800, 0x7114);
write_cmos_sensor(0x2802, 0x4382);
write_cmos_sensor(0x2804, 0x7334);
write_cmos_sensor(0x2806, 0x0f00);
write_cmos_sensor(0x2808, 0x7302);
write_cmos_sensor(0x280a, 0x4382);
write_cmos_sensor(0x280c, 0x770a);
write_cmos_sensor(0x280e, 0x4392);
write_cmos_sensor(0x2810, 0x7708);
write_cmos_sensor(0x2812, 0x431f);
write_cmos_sensor(0x2814, 0x4130);
write_cmos_sensor(0x2816, 0x120b);
write_cmos_sensor(0x2818, 0x120a);
write_cmos_sensor(0x281a, 0x1209);
write_cmos_sensor(0x281c, 0x1208);
write_cmos_sensor(0x281e, 0x1207);
write_cmos_sensor(0x2820, 0x1206);
write_cmos_sensor(0x2822, 0x1205);
write_cmos_sensor(0x2824, 0x1204);
write_cmos_sensor(0x2826, 0x403b);
write_cmos_sensor(0x2828, 0x0012);
write_cmos_sensor(0x282a, 0x510b);
write_cmos_sensor(0x282c, 0x4c06);
write_cmos_sensor(0x282e, 0x4b24);
write_cmos_sensor(0x2830, 0x4b15);
write_cmos_sensor(0x2832, 0x0002);
write_cmos_sensor(0x2834, 0x5e0e);
write_cmos_sensor(0x2836, 0x6f0f);
write_cmos_sensor(0x2838, 0x5e0e);
write_cmos_sensor(0x283a, 0x6f0f);
write_cmos_sensor(0x283c, 0x5e0e);
write_cmos_sensor(0x283e, 0x6f0f);
write_cmos_sensor(0x2840, 0x5e0e);
write_cmos_sensor(0x2842, 0x6f0f);
write_cmos_sensor(0x2844, 0x5e0e);
write_cmos_sensor(0x2846, 0x6f0f);
write_cmos_sensor(0x2848, 0x5e0e);
write_cmos_sensor(0x284a, 0x6f0f);
write_cmos_sensor(0x284c, 0x4d0a);
write_cmos_sensor(0x284e, 0x430b);
write_cmos_sensor(0x2850, 0x4e0c);
write_cmos_sensor(0x2852, 0x4f0d);
write_cmos_sensor(0x2854, 0x12b0);
write_cmos_sensor(0x2856, 0xffa6);
write_cmos_sensor(0x2858, 0x4c0a);
write_cmos_sensor(0x285a, 0x4d0b);
write_cmos_sensor(0x285c, 0x4c08);
write_cmos_sensor(0x285e, 0x9382);
write_cmos_sensor(0x2860, 0x8086);
write_cmos_sensor(0x2862, 0x202a);
write_cmos_sensor(0x2864, 0x9382);
write_cmos_sensor(0x2866, 0x8052);
write_cmos_sensor(0x2868, 0x2027);
write_cmos_sensor(0x286a, 0x9382);
write_cmos_sensor(0x286c, 0x807e);
write_cmos_sensor(0x286e, 0x2024);
write_cmos_sensor(0x2870, 0x4037);
write_cmos_sensor(0x2872, 0x0c98);
write_cmos_sensor(0x2874, 0x476f);
write_cmos_sensor(0x2876, 0x4f4e);
write_cmos_sensor(0x2878, 0x430f);
write_cmos_sensor(0x287a, 0x4e0c);
write_cmos_sensor(0x287c, 0x4f0d);
write_cmos_sensor(0x287e, 0x12b0);
write_cmos_sensor(0x2880, 0xff66);
write_cmos_sensor(0x2882, 0x4e08);
write_cmos_sensor(0x2884, 0x4f09);
write_cmos_sensor(0x2886, 0x462c);
write_cmos_sensor(0x2888, 0x430d);
write_cmos_sensor(0x288a, 0x476f);
write_cmos_sensor(0x288c, 0x4f4e);
write_cmos_sensor(0x288e, 0x403f);
write_cmos_sensor(0x2890, 0x0100);
write_cmos_sensor(0x2892, 0x8e0f);
write_cmos_sensor(0x2894, 0x4f0e);
write_cmos_sensor(0x2896, 0x4e0f);
write_cmos_sensor(0x2898, 0x5f0f);
write_cmos_sensor(0x289a, 0x7f0f);
write_cmos_sensor(0x289c, 0xe33f);
write_cmos_sensor(0x289e, 0x4c0a);
write_cmos_sensor(0x28a0, 0x4d0b);
write_cmos_sensor(0x28a2, 0x4e0c);
write_cmos_sensor(0x28a4, 0x4f0d);
write_cmos_sensor(0x28a6, 0x12b0);
write_cmos_sensor(0x28a8, 0xff66);
write_cmos_sensor(0x28aa, 0x5e08);
write_cmos_sensor(0x28ac, 0x6f09);
write_cmos_sensor(0x28ae, 0x1088);
write_cmos_sensor(0x28b0, 0x1089);
write_cmos_sensor(0x28b2, 0xe948);
write_cmos_sensor(0x28b4, 0xe908);
write_cmos_sensor(0x28b6, 0xf379);
write_cmos_sensor(0x28b8, 0x4886);
write_cmos_sensor(0x28ba, 0x0000);
write_cmos_sensor(0x28bc, 0x480f);
write_cmos_sensor(0x28be, 0xf03f);
write_cmos_sensor(0x28c0, 0x003f);
write_cmos_sensor(0x28c2, 0x4f0a);
write_cmos_sensor(0x28c4, 0x450c);
write_cmos_sensor(0x28c6, 0x430b);
write_cmos_sensor(0x28c8, 0x430d);
write_cmos_sensor(0x28ca, 0x12b0);
write_cmos_sensor(0x28cc, 0xff86);
write_cmos_sensor(0x28ce, 0xc312);
write_cmos_sensor(0x28d0, 0x100f);
write_cmos_sensor(0x28d2, 0x100e);
write_cmos_sensor(0x28d4, 0x110f);
write_cmos_sensor(0x28d6, 0x100e);
write_cmos_sensor(0x28d8, 0x110f);
write_cmos_sensor(0x28da, 0x100e);
write_cmos_sensor(0x28dc, 0x110f);
write_cmos_sensor(0x28de, 0x100e);
write_cmos_sensor(0x28e0, 0x110f);
write_cmos_sensor(0x28e2, 0x100e);
write_cmos_sensor(0x28e4, 0x110f);
write_cmos_sensor(0x28e6, 0x100e);
write_cmos_sensor(0x28e8, 0x4e84);
write_cmos_sensor(0x28ea, 0x0000);
write_cmos_sensor(0x28ec, 0x480f);
write_cmos_sensor(0x28ee, 0x4134);
write_cmos_sensor(0x28f0, 0x4135);
write_cmos_sensor(0x28f2, 0x4136);
write_cmos_sensor(0x28f4, 0x4137);
write_cmos_sensor(0x28f6, 0x4138);
write_cmos_sensor(0x28f8, 0x4139);
write_cmos_sensor(0x28fa, 0x413a);
write_cmos_sensor(0x28fc, 0x413b);
write_cmos_sensor(0x28fe, 0x4130);
write_cmos_sensor(0x2900, 0x42a2);
write_cmos_sensor(0x2902, 0x8060);
write_cmos_sensor(0x2904, 0x40b2);
write_cmos_sensor(0x2906, 0xff1f);
write_cmos_sensor(0x2908, 0x82bc);
write_cmos_sensor(0x290a, 0x42a2);
write_cmos_sensor(0x290c, 0x8088);
write_cmos_sensor(0x290e, 0x40b2);
write_cmos_sensor(0x2910, 0x01ce);
write_cmos_sensor(0x2912, 0x8062);
write_cmos_sensor(0x2914, 0x40b2);
write_cmos_sensor(0x2916, 0x01e0);
write_cmos_sensor(0x2918, 0x8064);
write_cmos_sensor(0x291a, 0x40b2);
write_cmos_sensor(0x291c, 0x03aa);
write_cmos_sensor(0x291e, 0x8066);
write_cmos_sensor(0x2920, 0x40b2);
write_cmos_sensor(0x2922, 0xfd07);
write_cmos_sensor(0x2924, 0x82be);
write_cmos_sensor(0x2926, 0x40b2);
write_cmos_sensor(0x2928, 0xff2f);
write_cmos_sensor(0x292a, 0x82c0);
write_cmos_sensor(0x292c, 0x40b2);
write_cmos_sensor(0x292e, 0xfe07);
write_cmos_sensor(0x2930, 0x82c2);
write_cmos_sensor(0x2932, 0x40b2);
write_cmos_sensor(0x2934, 0xff4f);
write_cmos_sensor(0x2936, 0x82c4);
write_cmos_sensor(0x2938, 0x40b2);
write_cmos_sensor(0x293a, 0xfd07);
write_cmos_sensor(0x293c, 0x82c6);
write_cmos_sensor(0x293e, 0x40b2);
write_cmos_sensor(0x2940, 0xff8f);
write_cmos_sensor(0x2942, 0x82c8);
write_cmos_sensor(0x2944, 0x40b2);
write_cmos_sensor(0x2946, 0xfeff); 
write_cmos_sensor(0x2948, 0x82ca);
write_cmos_sensor(0x294a, 0x421f);
write_cmos_sensor(0x294c, 0x82de);
write_cmos_sensor(0x294e, 0x930f);
write_cmos_sensor(0x2950, 0x2408);
write_cmos_sensor(0x2952, 0x40b2);
write_cmos_sensor(0x2954, 0xfe07);
write_cmos_sensor(0x2956, 0x82ca);
write_cmos_sensor(0x2958, 0x40b2);
write_cmos_sensor(0x295a, 0xfeff);
write_cmos_sensor(0x295c, 0x82c2);
write_cmos_sensor(0x295e, 0x930f);
write_cmos_sensor(0x2960, 0x2005);
write_cmos_sensor(0x2962, 0x430e);
write_cmos_sensor(0x2964, 0x421f);
write_cmos_sensor(0x2966, 0x8088);
write_cmos_sensor(0x2968, 0x12b0);
write_cmos_sensor(0x296a, 0xf98a);
write_cmos_sensor(0x296c, 0x421e);
write_cmos_sensor(0x296e, 0x8088);
write_cmos_sensor(0x2970, 0x4e0f);
write_cmos_sensor(0x2972, 0x12b0);
write_cmos_sensor(0x2974, 0xf98a);
write_cmos_sensor(0x2976, 0x9382);
write_cmos_sensor(0x2978, 0x82de);
write_cmos_sensor(0x297a, 0x2001);
write_cmos_sensor(0x297c, 0x4130);
write_cmos_sensor(0x297e, 0x430e);
write_cmos_sensor(0x2980, 0x421f);
write_cmos_sensor(0x2982, 0x8088);
write_cmos_sensor(0x2984, 0x12b0);
write_cmos_sensor(0x2986, 0xf98a);
write_cmos_sensor(0x2988, 0x3ff9);
write_cmos_sensor(0x298a, 0x4f0c);
write_cmos_sensor(0x298c, 0x430d);
write_cmos_sensor(0x298e, 0x9f0d);
write_cmos_sensor(0x2990, 0x2c10);
write_cmos_sensor(0x2992, 0x4d0f);
write_cmos_sensor(0x2994, 0x5e0f);
write_cmos_sensor(0x2996, 0x5f0f);
write_cmos_sensor(0x2998, 0x4f92);
write_cmos_sensor(0x299a, 0x82bc);
write_cmos_sensor(0x299c, 0x7700);
write_cmos_sensor(0x299e, 0x4d0f);
write_cmos_sensor(0x29a0, 0x5f0f);
write_cmos_sensor(0x29a2, 0x4f92);
write_cmos_sensor(0x29a4, 0x8060);
write_cmos_sensor(0x29a6, 0x7702);
write_cmos_sensor(0x29a8, 0x4392);
write_cmos_sensor(0x29aa, 0x7704);
write_cmos_sensor(0x29ac, 0x531d);
write_cmos_sensor(0x29ae, 0x9c0d);
write_cmos_sensor(0x29b0, 0x2bf0);
write_cmos_sensor(0x29b2, 0x4130);
write_cmos_sensor(0x29b4, 0x4292);
write_cmos_sensor(0x29b6, 0x8086);
write_cmos_sensor(0x29b8, 0x8052);
write_cmos_sensor(0x29ba, 0x430e);
write_cmos_sensor(0x29bc, 0xb392);
write_cmos_sensor(0x29be, 0x732a);
write_cmos_sensor(0x29c0, 0x2003);
write_cmos_sensor(0x29c2, 0xb3a2);
write_cmos_sensor(0x29c4, 0x732a);
write_cmos_sensor(0x29c6, 0x2401);
write_cmos_sensor(0x29c8, 0x431e);
write_cmos_sensor(0x29ca, 0x4e82);
write_cmos_sensor(0x29cc, 0x8086);
write_cmos_sensor(0x29ce, 0xb3e2);
write_cmos_sensor(0x29d0, 0x0080);
write_cmos_sensor(0x29d2, 0x251a);
write_cmos_sensor(0x29d4, 0x4392);
write_cmos_sensor(0x29d6, 0x82de);
write_cmos_sensor(0x29d8, 0x421d);
write_cmos_sensor(0x29da, 0x82de);
write_cmos_sensor(0x29dc, 0x930d);
write_cmos_sensor(0x29de, 0x2505);
write_cmos_sensor(0x29e0, 0x4292);
write_cmos_sensor(0x29e2, 0x00a8);
write_cmos_sensor(0x29e4, 0x807c);
write_cmos_sensor(0x29e6, 0x421f);
write_cmos_sensor(0x29e8, 0x00a2);
write_cmos_sensor(0x29ea, 0x533f);
write_cmos_sensor(0x29ec, 0x4f82);
write_cmos_sensor(0x29ee, 0x8054);
write_cmos_sensor(0x29f0, 0x4292);
write_cmos_sensor(0x29f2, 0x00ac);
write_cmos_sensor(0x29f4, 0x82aa);
write_cmos_sensor(0x29f6, 0x421f);
write_cmos_sensor(0x29f8, 0x00a6);
write_cmos_sensor(0x29fa, 0x533f);
write_cmos_sensor(0x29fc, 0x4f82);
write_cmos_sensor(0x29fe, 0x82b0);
write_cmos_sensor(0x2a00, 0x40b2);
write_cmos_sensor(0x2a02, 0x0005);
write_cmos_sensor(0x2a04, 0x7534);
write_cmos_sensor(0x2a06, 0x93c2);
write_cmos_sensor(0x2a08, 0x00cd);
write_cmos_sensor(0x2a0a, 0x2403);
write_cmos_sensor(0x2a0c, 0x40b2);
write_cmos_sensor(0x2a0e, 0x002f);
write_cmos_sensor(0x2a10, 0x7534);
write_cmos_sensor(0x2a12, 0x43b2);
write_cmos_sensor(0x2a14, 0x7810);
write_cmos_sensor(0x2a16, 0x43b2);
write_cmos_sensor(0x2a18, 0x7812);
write_cmos_sensor(0x2a1a, 0x4382);
write_cmos_sensor(0x2a1c, 0x752a); 
write_cmos_sensor(0x2a1e, 0x4382);
write_cmos_sensor(0x2a20, 0x752e);
write_cmos_sensor(0x2a22, 0x40b2);
write_cmos_sensor(0x2a24, 0x01db);
write_cmos_sensor(0x2a26, 0x780e);
write_cmos_sensor(0x2a28, 0x430e);
write_cmos_sensor(0x2a2a, 0x93c2);
write_cmos_sensor(0x2a2c, 0x00c6);
write_cmos_sensor(0x2a2e, 0x2001);
write_cmos_sensor(0x2a30, 0x431e);
write_cmos_sensor(0x2a32, 0x421f);
write_cmos_sensor(0x2a34, 0x732a);
write_cmos_sensor(0x2a36, 0xe31f);
write_cmos_sensor(0x2a38, 0xf31f);
write_cmos_sensor(0x2a3a, 0xfe0f);
write_cmos_sensor(0x2a3c, 0x425e);
write_cmos_sensor(0x2a3e, 0x00c7);
write_cmos_sensor(0x2a40, 0xf35e);
write_cmos_sensor(0x2a42, 0xf37e);
write_cmos_sensor(0x2a44, 0xde0f);
write_cmos_sensor(0x2a46, 0xd21f);
write_cmos_sensor(0x2a48, 0x807e);
write_cmos_sensor(0x2a4a, 0x930f);
write_cmos_sensor(0x2a4c, 0x240c);
write_cmos_sensor(0x2a4e, 0x425f);
write_cmos_sensor(0x2a50, 0x00d0);
write_cmos_sensor(0x2a52, 0xf35f);
write_cmos_sensor(0x2a54, 0x4f4e);
write_cmos_sensor(0x2a56, 0x108e);
write_cmos_sensor(0x2a58, 0x425f);
write_cmos_sensor(0x2a5a, 0x00ba);
write_cmos_sensor(0x2a5c, 0xf37f);
write_cmos_sensor(0x2a5e, 0xdf0e);
write_cmos_sensor(0x2a60, 0x4e82);
write_cmos_sensor(0x2a62, 0x0b92);
write_cmos_sensor(0x2a64, 0x0c0a);
write_cmos_sensor(0x2a66, 0x930d);
write_cmos_sensor(0x2a68, 0x24bd);
write_cmos_sensor(0x2a6a, 0x40f2);
write_cmos_sensor(0x2a6c, 0x0003);
write_cmos_sensor(0x2a6e, 0x00b5);
write_cmos_sensor(0x2a70, 0x40b2);
write_cmos_sensor(0x2a72, 0x003c);
write_cmos_sensor(0x2a74, 0x7814);
write_cmos_sensor(0x2a76, 0x40b2);
write_cmos_sensor(0x2a78, 0x0c01);
write_cmos_sensor(0x2a7a, 0x7500);
write_cmos_sensor(0x2a7c, 0x40b2);
write_cmos_sensor(0x2a7e, 0x0803);
write_cmos_sensor(0x2a80, 0x7502);
write_cmos_sensor(0x2a82, 0x40b2);
write_cmos_sensor(0x2a84, 0x0807);
write_cmos_sensor(0x2a86, 0x7504);
write_cmos_sensor(0x2a88, 0x40b2);
write_cmos_sensor(0x2a8a, 0x4803);
write_cmos_sensor(0x2a8c, 0x7506);
write_cmos_sensor(0x2a8e, 0x40b2);
write_cmos_sensor(0x2a90, 0x0801);
write_cmos_sensor(0x2a92, 0x7508);
write_cmos_sensor(0x2a94, 0x40b2);
write_cmos_sensor(0x2a96, 0x0805);
write_cmos_sensor(0x2a98, 0x750a);
write_cmos_sensor(0x2a9a, 0x40b2);
write_cmos_sensor(0x2a9c, 0x4801);
write_cmos_sensor(0x2a9e, 0x750c);
write_cmos_sensor(0x2aa0, 0x40b2);
write_cmos_sensor(0x2aa2, 0x0803);
write_cmos_sensor(0x2aa4, 0x750e);
write_cmos_sensor(0x2aa6, 0x40b2);
write_cmos_sensor(0x2aa8, 0x0802);
write_cmos_sensor(0x2aaa, 0x7510);
write_cmos_sensor(0x2aac, 0x40b2);
write_cmos_sensor(0x2aae, 0x0800);
write_cmos_sensor(0x2ab0, 0x7512);
write_cmos_sensor(0x2ab2, 0x930d);
write_cmos_sensor(0x2ab4, 0x2490);
write_cmos_sensor(0x2ab6, 0x40b2);
write_cmos_sensor(0x2ab8, 0x09fb);
write_cmos_sensor(0x2aba, 0x7530);
write_cmos_sensor(0x2abc, 0x40b2);
write_cmos_sensor(0x2abe, 0x09fd);
write_cmos_sensor(0x2ac0, 0x7532);
write_cmos_sensor(0x2ac2, 0x425f);
write_cmos_sensor(0x2ac4, 0x00db);
write_cmos_sensor(0x2ac6, 0xf37f);
write_cmos_sensor(0x2ac8, 0x421e);
write_cmos_sensor(0x2aca, 0x0086);
write_cmos_sensor(0x2acc, 0x12b0);
write_cmos_sensor(0x2ace, 0xfc68);
write_cmos_sensor(0x2ad0, 0x4e82);
write_cmos_sensor(0x2ad2, 0x82ac);
write_cmos_sensor(0x2ad4, 0x4f82);
write_cmos_sensor(0x2ad6, 0x82ae);
write_cmos_sensor(0x2ad8, 0x4292);
write_cmos_sensor(0x2ada, 0x0088);
write_cmos_sensor(0x2adc, 0x7316);
write_cmos_sensor(0x2ade, 0x4382);
write_cmos_sensor(0x2ae0, 0x7542);
write_cmos_sensor(0x2ae2, 0x40b2);
write_cmos_sensor(0x2ae4, 0x01db);
write_cmos_sensor(0x2ae6, 0x7544);
write_cmos_sensor(0x2ae8, 0x43b2);
write_cmos_sensor(0x2aea, 0x7546);
write_cmos_sensor(0x2aec, 0x43b2);
write_cmos_sensor(0x2aee, 0x7548);
write_cmos_sensor(0x2af0, 0x93c2);
write_cmos_sensor(0x2af2, 0x00cd);
write_cmos_sensor(0x2af4, 0x2406);
write_cmos_sensor(0x2af6, 0x40b2);
write_cmos_sensor(0x2af8, 0x0074);
write_cmos_sensor(0x2afa, 0x754c);
write_cmos_sensor(0x2afc, 0x40b2);
write_cmos_sensor(0x2afe, 0x024f);
write_cmos_sensor(0x2b00, 0x754e);
write_cmos_sensor(0x2b02, 0x425f);
write_cmos_sensor(0x2b04, 0x00d7);
write_cmos_sensor(0x2b06, 0xf37f);
write_cmos_sensor(0x2b08, 0x403d);
write_cmos_sensor(0x2b0a, 0x82e4);
write_cmos_sensor(0x2b0c, 0x421e);
write_cmos_sensor(0x2b0e, 0x0084);
write_cmos_sensor(0x2b10, 0x12b0);
write_cmos_sensor(0x2b12, 0xfc74);
write_cmos_sensor(0x2b14, 0x93c2);
write_cmos_sensor(0x2b16, 0x00cd);
write_cmos_sensor(0x2b18, 0x2054);
write_cmos_sensor(0x2b1a, 0x434d);
write_cmos_sensor(0x2b1c, 0x403e);
write_cmos_sensor(0x2b1e, 0x82b4);
write_cmos_sensor(0x2b20, 0x403f);
write_cmos_sensor(0x2b22, 0x82e4);
write_cmos_sensor(0x2b24, 0x12b0);
write_cmos_sensor(0x2b26, 0xfcf0);
write_cmos_sensor(0x2b28, 0x93c2);
write_cmos_sensor(0x2b2a, 0x00cd);
write_cmos_sensor(0x2b2c, 0x2042);
write_cmos_sensor(0x2b2e, 0x4382);
write_cmos_sensor(0x2b30, 0x733e);
write_cmos_sensor(0x2b32, 0x93c2);
write_cmos_sensor(0x2b34, 0x00cd);
write_cmos_sensor(0x2b36, 0x240b);
write_cmos_sensor(0x2b38, 0x421e);
write_cmos_sensor(0x2b3a, 0x82e4);
write_cmos_sensor(0x2b3c, 0x421f);
write_cmos_sensor(0x2b3e, 0x82e6);
write_cmos_sensor(0x2b40, 0x821e);
write_cmos_sensor(0x2b42, 0x80a0);
write_cmos_sensor(0x2b44, 0x721f);
write_cmos_sensor(0x2b46, 0x80a2);
write_cmos_sensor(0x2b48, 0x2c02);
write_cmos_sensor(0x2b4a, 0x4392);
write_cmos_sensor(0x2b4c, 0x733e);
write_cmos_sensor(0x2b4e, 0x40b2);
write_cmos_sensor(0x2b50, 0xffeb);
write_cmos_sensor(0x2b52, 0x7808);
write_cmos_sensor(0x2b54, 0x40b2);
write_cmos_sensor(0x2b56, 0xffd7);
write_cmos_sensor(0x2b58, 0x7806);
write_cmos_sensor(0x2b5a, 0x40b2);
write_cmos_sensor(0x2b5c, 0x003c);
write_cmos_sensor(0x2b5e, 0x7804);
write_cmos_sensor(0x2b60, 0x40b2);
write_cmos_sensor(0x2b62, 0x0030);
write_cmos_sensor(0x2b64, 0x7802);
write_cmos_sensor(0x2b66, 0xb2e2);
write_cmos_sensor(0x2b68, 0x0110);
write_cmos_sensor(0x2b6a, 0x241f);
write_cmos_sensor(0x2b6c, 0x42d2);
write_cmos_sensor(0x2b6e, 0x0111);
write_cmos_sensor(0x2b70, 0x82e0);
write_cmos_sensor(0x2b72, 0xb3e2);
write_cmos_sensor(0x2b74, 0x0110);
write_cmos_sensor(0x2b76, 0x2407);
write_cmos_sensor(0x2b78, 0x421f);
write_cmos_sensor(0x2b7a, 0x0084);
write_cmos_sensor(0x2b7c, 0x9f82);
write_cmos_sensor(0x2b7e, 0x0112);
write_cmos_sensor(0x2b80, 0x2c11);
write_cmos_sensor(0x2b82, 0x43d2);
write_cmos_sensor(0x2b84, 0x82e0);
write_cmos_sensor(0x2b86, 0xb3d2);
write_cmos_sensor(0x2b88, 0x0110);
write_cmos_sensor(0x2b8a, 0x2408);
write_cmos_sensor(0x2b8c, 0x434f);
write_cmos_sensor(0x2b8e, 0x93c2);
write_cmos_sensor(0x2b90, 0x82e0);
write_cmos_sensor(0x2b92, 0x2001);
write_cmos_sensor(0x2b94, 0x435f);
write_cmos_sensor(0x2b96, 0x4fc2);
write_cmos_sensor(0x2b98, 0x00ca);
write_cmos_sensor(0x2b9a, 0x3c39);
write_cmos_sensor(0x2b9c, 0x42d2);
write_cmos_sensor(0x2b9e, 0x82e0);
write_cmos_sensor(0x2ba0, 0x00ca);
write_cmos_sensor(0x2ba2, 0x3c35);
write_cmos_sensor(0x2ba4, 0x43c2);
write_cmos_sensor(0x2ba6, 0x82e0);
write_cmos_sensor(0x2ba8, 0x3fee);
write_cmos_sensor(0x2baa, 0x42d2);
write_cmos_sensor(0x2bac, 0x00ca);
write_cmos_sensor(0x2bae, 0x82e0);
write_cmos_sensor(0x2bb0, 0x3fe0);
write_cmos_sensor(0x2bb2, 0x435d);
write_cmos_sensor(0x2bb4, 0x403e);
write_cmos_sensor(0x2bb6, 0x808b);
write_cmos_sensor(0x2bb8, 0x403f);
write_cmos_sensor(0x2bba, 0x80a0);
write_cmos_sensor(0x2bbc, 0x12b0);
write_cmos_sensor(0x2bbe, 0xfcf0);
write_cmos_sensor(0x2bc0, 0x3fb6);
write_cmos_sensor(0x2bc2, 0x425f);
write_cmos_sensor(0x2bc4, 0x00d9);
write_cmos_sensor(0x2bc6, 0xf37f);
write_cmos_sensor(0x2bc8, 0x403d);
write_cmos_sensor(0x2bca, 0x80a0);
write_cmos_sensor(0x2bcc, 0x421e);
write_cmos_sensor(0x2bce, 0x00d4);
write_cmos_sensor(0x2bd0, 0x12b0);
write_cmos_sensor(0x2bd2, 0xfc74);
write_cmos_sensor(0x2bd4, 0x3fa2);
write_cmos_sensor(0x2bd6, 0x40b2);
write_cmos_sensor(0x2bd8, 0x09fa);
write_cmos_sensor(0x2bda, 0x7530);
write_cmos_sensor(0x2bdc, 0x40b2);
write_cmos_sensor(0x2bde, 0x09fc);
write_cmos_sensor(0x2be0, 0x7532);
write_cmos_sensor(0x2be2, 0x3f6f);
write_cmos_sensor(0x2be4, 0x43c2);
write_cmos_sensor(0x2be6, 0x00b5);
write_cmos_sensor(0x2be8, 0x3f43);
write_cmos_sensor(0x2bea, 0x4292);
write_cmos_sensor(0x2bec, 0x00a2);
write_cmos_sensor(0x2bee, 0x807c);
write_cmos_sensor(0x2bf0, 0x421f);
write_cmos_sensor(0x2bf2, 0x00a8);
write_cmos_sensor(0x2bf4, 0x531f);
write_cmos_sensor(0x2bf6, 0x4f82);
write_cmos_sensor(0x2bf8, 0x8054);
write_cmos_sensor(0x2bfa, 0x4292);
write_cmos_sensor(0x2bfc, 0x00a6);
write_cmos_sensor(0x2bfe, 0x82aa);
write_cmos_sensor(0x2c00, 0x421f);
write_cmos_sensor(0x2c02, 0x00ac);
write_cmos_sensor(0x2c04, 0x531f);
write_cmos_sensor(0x2c06, 0x3efa);
write_cmos_sensor(0x2c08, 0x4382);
write_cmos_sensor(0x2c0a, 0x82de);
write_cmos_sensor(0x2c0c, 0x3ee5);
write_cmos_sensor(0x2c0e, 0xd392);
write_cmos_sensor(0x2c10, 0x7102);
write_cmos_sensor(0x2c12, 0x4130);
write_cmos_sensor(0x2c14, 0x0260);
write_cmos_sensor(0x2c16, 0x0000);
write_cmos_sensor(0x2c18, 0x0c51);
write_cmos_sensor(0x2c1a, 0x0240);
write_cmos_sensor(0x2c1c, 0x0000);
write_cmos_sensor(0x2c1e, 0x0260);
write_cmos_sensor(0x2c20, 0x0000);
write_cmos_sensor(0x2c22, 0x0c19);
write_cmos_sensor(0x2c24, 0x4130);
write_cmos_sensor(0x2c26, 0x4e82);
write_cmos_sensor(0x2c28, 0x7602);
write_cmos_sensor(0x2c2a, 0x4f82);
write_cmos_sensor(0x2c2c, 0x7600);
write_cmos_sensor(0x2c2e, 0x0270);
write_cmos_sensor(0x2c30, 0x0000);
write_cmos_sensor(0x2c32, 0x0c18);
write_cmos_sensor(0x2c34, 0x0270);
write_cmos_sensor(0x2c36, 0x0001);
write_cmos_sensor(0x2c38, 0x421f);
write_cmos_sensor(0x2c3a, 0x7606);
write_cmos_sensor(0x2c3c, 0x4130);
write_cmos_sensor(0x2c3e, 0x4f0e);
write_cmos_sensor(0x2c40, 0xc312);
write_cmos_sensor(0x2c42, 0x100f);
write_cmos_sensor(0x2c44, 0x110f);
write_cmos_sensor(0x2c46, 0xc312);
write_cmos_sensor(0x2c48, 0x100f);
write_cmos_sensor(0x2c4a, 0x4f82);
write_cmos_sensor(0x2c4c, 0x7600);
write_cmos_sensor(0x2c4e, 0xf03e);
write_cmos_sensor(0x2c50, 0x0007);
write_cmos_sensor(0x2c52, 0x4e82);
write_cmos_sensor(0x2c54, 0x7602);
write_cmos_sensor(0x2c56, 0x0262);
write_cmos_sensor(0x2c58, 0x0000);
write_cmos_sensor(0x2c5a, 0x0222);
write_cmos_sensor(0x2c5c, 0x0000);
write_cmos_sensor(0x2c5e, 0x0262);
write_cmos_sensor(0x2c60, 0x0000);
write_cmos_sensor(0x2c62, 0x0260);
write_cmos_sensor(0x2c64, 0x0000);
write_cmos_sensor(0x2c66, 0x4130);
write_cmos_sensor(0x2c68, 0x4f0d);
write_cmos_sensor(0x2c6a, 0x430c);
write_cmos_sensor(0x2c6c, 0x430f);
write_cmos_sensor(0x2c6e, 0xdc0e);
write_cmos_sensor(0x2c70, 0xdd0f);
write_cmos_sensor(0x2c72, 0x4130);
write_cmos_sensor(0x2c74, 0x120b);
write_cmos_sensor(0x2c76, 0x120a);
write_cmos_sensor(0x2c78, 0x1209);
write_cmos_sensor(0x2c7a, 0x1208);
write_cmos_sensor(0x2c7c, 0x4d0b);
write_cmos_sensor(0x2c7e, 0x12b0);
write_cmos_sensor(0x2c80, 0xfc68);
write_cmos_sensor(0x2c82, 0x4e8b);
write_cmos_sensor(0x2c84, 0x0000);
write_cmos_sensor(0x2c86, 0x4f8b);
write_cmos_sensor(0x2c88, 0x0002);
write_cmos_sensor(0x2c8a, 0x803e);
write_cmos_sensor(0x2c8c, 0x0006);
write_cmos_sensor(0x2c8e, 0x730f);
write_cmos_sensor(0x2c90, 0x2c05);
write_cmos_sensor(0x2c92, 0x40bb);
write_cmos_sensor(0x2c94, 0x0006);
write_cmos_sensor(0x2c96, 0x0000);
write_cmos_sensor(0x2c98, 0x438b);
write_cmos_sensor(0x2c9a, 0x0002);
write_cmos_sensor(0x2c9c, 0x4308);
write_cmos_sensor(0x2c9e, 0x93c2);
write_cmos_sensor(0x2ca0, 0x00bc);
write_cmos_sensor(0x2ca2, 0x2001);
write_cmos_sensor(0x2ca4, 0x4318);
write_cmos_sensor(0x2ca6, 0x4309);
write_cmos_sensor(0x2ca8, 0x421e);
write_cmos_sensor(0x2caa, 0x82ac);
write_cmos_sensor(0x2cac, 0x421f);
write_cmos_sensor(0x2cae, 0x82ae);
write_cmos_sensor(0x2cb0, 0x503e);
write_cmos_sensor(0x2cb2, 0xfffa);
write_cmos_sensor(0x2cb4, 0x633f);
write_cmos_sensor(0x2cb6, 0x4b3a);
write_cmos_sensor(0x2cb8, 0x4b2b);
write_cmos_sensor(0x2cba, 0x4a0c);
write_cmos_sensor(0x2cbc, 0x4b0d);
write_cmos_sensor(0x2cbe, 0x8e0c);
write_cmos_sensor(0x2cc0, 0x7f0d);
write_cmos_sensor(0x2cc2, 0x2801);
write_cmos_sensor(0x2cc4, 0x4319);
write_cmos_sensor(0x2cc6, 0x480f);
write_cmos_sensor(0x2cc8, 0xf90f);
write_cmos_sensor(0x2cca, 0x2407);
write_cmos_sensor(0x2ccc, 0x503a);
write_cmos_sensor(0x2cce, 0x0006);
write_cmos_sensor(0x2cd0, 0x630b);
write_cmos_sensor(0x2cd2, 0x4a82);
write_cmos_sensor(0x2cd4, 0x82ac);
write_cmos_sensor(0x2cd6, 0x4b82);
write_cmos_sensor(0x2cd8, 0x82ae);
write_cmos_sensor(0x2cda, 0x4292);
write_cmos_sensor(0x2cdc, 0x82ac);
write_cmos_sensor(0x2cde, 0x7314);
write_cmos_sensor(0x2ce0, 0x4292);
write_cmos_sensor(0x2ce2, 0x82ae);
write_cmos_sensor(0x2ce4, 0x7336);
write_cmos_sensor(0x2ce6, 0x4138);
write_cmos_sensor(0x2ce8, 0x4139);
write_cmos_sensor(0x2cea, 0x413a);
write_cmos_sensor(0x2cec, 0x413b);
write_cmos_sensor(0x2cee, 0x4130);
write_cmos_sensor(0x2cf0, 0x120b);
write_cmos_sensor(0x2cf2, 0x120a);
write_cmos_sensor(0x2cf4, 0x1209);
write_cmos_sensor(0x2cf6, 0x1208);
write_cmos_sensor(0x2cf8, 0x1207);
write_cmos_sensor(0x2cfa, 0x1206);
write_cmos_sensor(0x2cfc, 0x4f0c);
write_cmos_sensor(0x2cfe, 0x4e07);
write_cmos_sensor(0x2d00, 0x4d46);
write_cmos_sensor(0x2d02, 0x4218);
write_cmos_sensor(0x2d04, 0x82ac);
write_cmos_sensor(0x2d06, 0x4219);
write_cmos_sensor(0x2d08, 0x82ae);
write_cmos_sensor(0x2d0a, 0x480e);
write_cmos_sensor(0x2d0c, 0x490f);
write_cmos_sensor(0x2d0e, 0x503e);
write_cmos_sensor(0x2d10, 0xfffa);
write_cmos_sensor(0x2d12, 0x633f);
write_cmos_sensor(0x2d14, 0x4c3a);
write_cmos_sensor(0x2d16, 0x4c3b);
write_cmos_sensor(0x2d18, 0x4a0c);
write_cmos_sensor(0x2d1a, 0x4b0d);
write_cmos_sensor(0x2d1c, 0x8e0c);
write_cmos_sensor(0x2d1e, 0x7f0d);
write_cmos_sensor(0x2d20, 0x283d);
write_cmos_sensor(0x2d22, 0x403a);
write_cmos_sensor(0x2d24, 0x0006);
write_cmos_sensor(0x2d26, 0x430b);
write_cmos_sensor(0x2d28, 0x4349);
write_cmos_sensor(0x2d2a, 0x421e);
write_cmos_sensor(0x2d2c, 0x7560);
write_cmos_sensor(0x2d2e, 0x421f);
write_cmos_sensor(0x2d30, 0x7578);
write_cmos_sensor(0x2d32, 0x12b0);
write_cmos_sensor(0x2d34, 0xfc68);
write_cmos_sensor(0x2d36, 0x4a0c);
write_cmos_sensor(0x2d38, 0x4b0d);
write_cmos_sensor(0x2d3a, 0x8e0c);
write_cmos_sensor(0x2d3c, 0x7f0d);
write_cmos_sensor(0x2d3e, 0x2c01);
write_cmos_sensor(0x2d40, 0x4359);
write_cmos_sensor(0x2d42, 0x434f);
write_cmos_sensor(0x2d44, 0x9382);
write_cmos_sensor(0x2d46, 0x807e);
write_cmos_sensor(0x2d48, 0x2001);
write_cmos_sensor(0x2d4a, 0x435f);
write_cmos_sensor(0x2d4c, 0xf94f);
write_cmos_sensor(0x2d4e, 0x434e);
write_cmos_sensor(0x2d50, 0x93c7);
write_cmos_sensor(0x2d52, 0x0000);
write_cmos_sensor(0x2d54, 0x2001);
write_cmos_sensor(0x2d56, 0x435e);
write_cmos_sensor(0x2d58, 0xfe4f);
write_cmos_sensor(0x2d5a, 0x4fc7);
write_cmos_sensor(0x2d5c, 0x0000);
write_cmos_sensor(0x2d5e, 0x9346);
write_cmos_sensor(0x2d60, 0x200d);
write_cmos_sensor(0x2d62, 0x4a82);
write_cmos_sensor(0x2d64, 0x7540);
write_cmos_sensor(0x2d66, 0x4b82);
write_cmos_sensor(0x2d68, 0x7574);
write_cmos_sensor(0x2d6a, 0x934f);
write_cmos_sensor(0x2d6c, 0x241c);
write_cmos_sensor(0x2d6e, 0x421f);
write_cmos_sensor(0x2d70, 0x8076);
write_cmos_sensor(0x2d72, 0x5f82);
write_cmos_sensor(0x2d74, 0x7542);
write_cmos_sensor(0x2d76, 0x5f82);
write_cmos_sensor(0x2d78, 0x7544);
write_cmos_sensor(0x2d7a, 0x3c15);
write_cmos_sensor(0x2d7c, 0x4a82);
write_cmos_sensor(0x2d7e, 0x754a);
write_cmos_sensor(0x2d80, 0x4b82);
write_cmos_sensor(0x2d82, 0x7576);
write_cmos_sensor(0x2d84, 0x934f);
write_cmos_sensor(0x2d86, 0x240f);
write_cmos_sensor(0x2d88, 0x93c2);
write_cmos_sensor(0x2d8a, 0x00cd);
write_cmos_sensor(0x2d8c, 0x240c);
write_cmos_sensor(0x2d8e, 0x421f);
write_cmos_sensor(0x2d90, 0x8076);
write_cmos_sensor(0x2d92, 0x5f82);
write_cmos_sensor(0x2d94, 0x754c);
write_cmos_sensor(0x2d96, 0x5f82);
write_cmos_sensor(0x2d98, 0x754e);
write_cmos_sensor(0x2d9a, 0x3c05);
write_cmos_sensor(0x2d9c, 0x8a08);
write_cmos_sensor(0x2d9e, 0x7b09);
write_cmos_sensor(0x2da0, 0x480a);
write_cmos_sensor(0x2da2, 0x490b);
write_cmos_sensor(0x2da4, 0x3fc1);
write_cmos_sensor(0x2da6, 0x4136);
write_cmos_sensor(0x2da8, 0x4137);
write_cmos_sensor(0x2daa, 0x4138);
write_cmos_sensor(0x2dac, 0x4139);
write_cmos_sensor(0x2dae, 0x413a);
write_cmos_sensor(0x2db0, 0x413b);
write_cmos_sensor(0x2db2, 0x4130);
write_cmos_sensor(0x2db4, 0x120b);
write_cmos_sensor(0x2db6, 0x120a);
write_cmos_sensor(0x2db8, 0x1209);
write_cmos_sensor(0x2dba, 0x1208);
write_cmos_sensor(0x2dbc, 0x4f0a);
write_cmos_sensor(0x2dbe, 0x4e09);
write_cmos_sensor(0x2dc0, 0x4382);
write_cmos_sensor(0x2dc2, 0x8084);
write_cmos_sensor(0x2dc4, 0x4308);
write_cmos_sensor(0x2dc6, 0x421e);
write_cmos_sensor(0x2dc8, 0x82de);
write_cmos_sensor(0x2dca, 0x930e);
write_cmos_sensor(0x2dcc, 0x24c2);
write_cmos_sensor(0x2dce, 0x421f);
write_cmos_sensor(0x2dd0, 0x00ac);
write_cmos_sensor(0x2dd2, 0x503f);
write_cmos_sensor(0x2dd4, 0xffc0);
write_cmos_sensor(0x2dd6, 0x4f82);
write_cmos_sensor(0x2dd8, 0x8098);
write_cmos_sensor(0x2dda, 0x930e);
write_cmos_sensor(0x2ddc, 0x24b7);
write_cmos_sensor(0x2dde, 0x421f);
write_cmos_sensor(0x2de0, 0x00a6);
write_cmos_sensor(0x2de2, 0x503f);
write_cmos_sensor(0x2de4, 0xffc0);
write_cmos_sensor(0x2de6, 0x4f82);
write_cmos_sensor(0x2de8, 0x808e);
write_cmos_sensor(0x2dea, 0x4382);
write_cmos_sensor(0x2dec, 0x8082);
write_cmos_sensor(0x2dee, 0x930a);
write_cmos_sensor(0x2df0, 0x24b3);
write_cmos_sensor(0x2df2, 0x9382);
write_cmos_sensor(0x2df4, 0x82de);
write_cmos_sensor(0x2df6, 0x24a6);
write_cmos_sensor(0x2df8, 0x490f);
write_cmos_sensor(0x2dfa, 0x5a0f);
write_cmos_sensor(0x2dfc, 0x821f);
write_cmos_sensor(0x2dfe, 0x8082);
write_cmos_sensor(0x2e00, 0x503f);
write_cmos_sensor(0x2e02, 0xfffd);
write_cmos_sensor(0x2e04, 0x4f82);
write_cmos_sensor(0x2e06, 0x809c);
write_cmos_sensor(0x2e08, 0x430e);
write_cmos_sensor(0x2e0a, 0x12b0);
write_cmos_sensor(0x2e0c, 0xfc26);
write_cmos_sensor(0x2e0e, 0x4f82);
write_cmos_sensor(0x2e10, 0x805e);
write_cmos_sensor(0x2e12, 0x421f);
write_cmos_sensor(0x2e14, 0x809c);
write_cmos_sensor(0x2e16, 0x531f);
write_cmos_sensor(0x2e18, 0x430e);
write_cmos_sensor(0x2e1a, 0x12b0);
write_cmos_sensor(0x2e1c, 0xfc26);
write_cmos_sensor(0x2e1e, 0x4f82);
write_cmos_sensor(0x2e20, 0x808c);
write_cmos_sensor(0x2e22, 0x421f);
write_cmos_sensor(0x2e24, 0x809c);
write_cmos_sensor(0x2e26, 0x532f);
write_cmos_sensor(0x2e28, 0x430e);
write_cmos_sensor(0x2e2a, 0x12b0);
write_cmos_sensor(0x2e2c, 0xfc26);
write_cmos_sensor(0x2e2e, 0x4f82);
write_cmos_sensor(0x2e30, 0x82e2);
write_cmos_sensor(0x2e32, 0x421d);
write_cmos_sensor(0x2e34, 0x808c);
write_cmos_sensor(0x2e36, 0x4d0e);
write_cmos_sensor(0x2e38, 0xf37e);
write_cmos_sensor(0x2e3a, 0x108e);
write_cmos_sensor(0x2e3c, 0xdf0e);
write_cmos_sensor(0x2e3e, 0xf03e);
write_cmos_sensor(0x2e40, 0x0fff);
write_cmos_sensor(0x2e42, 0x4e82);
write_cmos_sensor(0x2e44, 0x809a);
write_cmos_sensor(0x2e46, 0x421c);
write_cmos_sensor(0x2e48, 0x805e);
write_cmos_sensor(0x2e4a, 0x5c0c);
write_cmos_sensor(0x2e4c, 0x5c0c);
write_cmos_sensor(0x2e4e, 0x5c0c);
write_cmos_sensor(0x2e50, 0x5c0c);
write_cmos_sensor(0x2e52, 0x4d0f);
write_cmos_sensor(0x2e54, 0xc312);
write_cmos_sensor(0x2e56, 0x100f);
write_cmos_sensor(0x2e58, 0x110f);
write_cmos_sensor(0x2e5a, 0x110f);
write_cmos_sensor(0x2e5c, 0x110f);
write_cmos_sensor(0x2e5e, 0xdf0c);
write_cmos_sensor(0x2e60, 0xf03c);
write_cmos_sensor(0x2e62, 0x0fff);
write_cmos_sensor(0x2e64, 0x4c82);
write_cmos_sensor(0x2e66, 0x8094);
write_cmos_sensor(0x2e68, 0x434b);
write_cmos_sensor(0x2e6a, 0x930e);
write_cmos_sensor(0x2e6c, 0x2002);
write_cmos_sensor(0x2e6e, 0x930c);
write_cmos_sensor(0x2e70, 0x2401);
write_cmos_sensor(0x2e72, 0x435b);
write_cmos_sensor(0x2e74, 0x90b2);
write_cmos_sensor(0x2e76, 0x0033);
write_cmos_sensor(0x2e78, 0x809c);
write_cmos_sensor(0x2e7a, 0x2c62);
write_cmos_sensor(0x2e7c, 0x934b);
write_cmos_sensor(0x2e7e, 0x2060);
write_cmos_sensor(0x2e80, 0x435f);
write_cmos_sensor(0x2e82, 0xdf4b);
write_cmos_sensor(0x2e84, 0x93c2);
write_cmos_sensor(0x2e86, 0x00cd);
write_cmos_sensor(0x2e88, 0x2459);
write_cmos_sensor(0x2e8a, 0x432e);
write_cmos_sensor(0x2e8c, 0x421d);
write_cmos_sensor(0x2e8e, 0x82de);
write_cmos_sensor(0x2e90, 0x930d);
write_cmos_sensor(0x2e92, 0x244f);
write_cmos_sensor(0x2e94, 0x421f);
write_cmos_sensor(0x2e96, 0x8098);
write_cmos_sensor(0x2e98, 0x821f);
write_cmos_sensor(0x2e9a, 0x8094);
write_cmos_sensor(0x2e9c, 0x5e0f);
write_cmos_sensor(0x2e9e, 0x4f82);
write_cmos_sensor(0x2ea0, 0x8050);
write_cmos_sensor(0x2ea2, 0x430e);
write_cmos_sensor(0x2ea4, 0x421f);
write_cmos_sensor(0x2ea6, 0x8050);
write_cmos_sensor(0x2ea8, 0xf21f);
write_cmos_sensor(0x2eaa, 0x8090);
write_cmos_sensor(0x2eac, 0xc312);
write_cmos_sensor(0x2eae, 0x100f);
write_cmos_sensor(0x2eb0, 0x930f);
write_cmos_sensor(0x2eb2, 0x2001);
write_cmos_sensor(0x2eb4, 0x431e);
write_cmos_sensor(0x2eb6, 0x4e82);
write_cmos_sensor(0x2eb8, 0x8080);
write_cmos_sensor(0x2eba, 0x930d);
write_cmos_sensor(0x2ebc, 0x2435);
write_cmos_sensor(0x2ebe, 0x9292);
write_cmos_sensor(0x2ec0, 0x8094);
write_cmos_sensor(0x2ec2, 0x8098);
write_cmos_sensor(0x2ec4, 0x2c2e);
write_cmos_sensor(0x2ec6, 0x430f);
write_cmos_sensor(0x2ec8, 0x4f82);
write_cmos_sensor(0x2eca, 0x8058);
write_cmos_sensor(0x2ecc, 0x930f);
write_cmos_sensor(0x2ece, 0x241e);
write_cmos_sensor(0x2ed0, 0x9382);
write_cmos_sensor(0x2ed2, 0x8080);
write_cmos_sensor(0x2ed4, 0x241b);
write_cmos_sensor(0x2ed6, 0x421e);
write_cmos_sensor(0x2ed8, 0x8084);
write_cmos_sensor(0x2eda, 0x903e);
write_cmos_sensor(0x2edc, 0x0081);
write_cmos_sensor(0x2ede, 0x2c0d);
write_cmos_sensor(0x2ee0, 0x4e0f);
write_cmos_sensor(0x2ee2, 0x5f0f);
write_cmos_sensor(0x2ee4, 0x5f0f);
write_cmos_sensor(0x2ee6, 0x429f);
write_cmos_sensor(0x2ee8, 0x8094);
write_cmos_sensor(0x2eea, 0x80a6);
write_cmos_sensor(0x2eec, 0x429f);
write_cmos_sensor(0x2eee, 0x809a);
write_cmos_sensor(0x2ef0, 0x80a8);
write_cmos_sensor(0x2ef2, 0x531e);
write_cmos_sensor(0x2ef4, 0x4e82);
write_cmos_sensor(0x2ef6, 0x8084);
write_cmos_sensor(0x2ef8, 0x4308);
write_cmos_sensor(0x2efa, 0x421f);
write_cmos_sensor(0x2efc, 0x8082);
write_cmos_sensor(0x2efe, 0x503f);
write_cmos_sensor(0x2f00, 0x0003);
write_cmos_sensor(0x2f02, 0x4f82);
write_cmos_sensor(0x2f04, 0x8082);
write_cmos_sensor(0x2f06, 0x9a0f);
write_cmos_sensor(0x2f08, 0x2b74);
write_cmos_sensor(0x2f0a, 0x3c26);
write_cmos_sensor(0x2f0c, 0x421f);
write_cmos_sensor(0x2f0e, 0x8084);
write_cmos_sensor(0x2f10, 0x580f);
write_cmos_sensor(0x2f12, 0x5f0f);
write_cmos_sensor(0x2f14, 0x5f0f);
write_cmos_sensor(0x2f16, 0x438f);
write_cmos_sensor(0x2f18, 0x80a6);
write_cmos_sensor(0x2f1a, 0x438f);
write_cmos_sensor(0x2f1c, 0x80a8);
write_cmos_sensor(0x2f1e, 0x5318);
write_cmos_sensor(0x2f20, 0x3fec);
write_cmos_sensor(0x2f22, 0x4b0f);
write_cmos_sensor(0x2f24, 0xf31f);
write_cmos_sensor(0x2f26, 0x3fd0);
write_cmos_sensor(0x2f28, 0x9292);
write_cmos_sensor(0x2f2a, 0x8098);
write_cmos_sensor(0x2f2c, 0x8094);
write_cmos_sensor(0x2f2e, 0x2bcb);
write_cmos_sensor(0x2f30, 0x3ff8);
write_cmos_sensor(0x2f32, 0x421f);
write_cmos_sensor(0x2f34, 0x8094);
write_cmos_sensor(0x2f36, 0x821f);
write_cmos_sensor(0x2f38, 0x8098);
write_cmos_sensor(0x2f3a, 0x3fb0);
write_cmos_sensor(0x2f3c, 0x430e);
write_cmos_sensor(0x2f3e, 0x3fa6);
write_cmos_sensor(0x2f40, 0x434f);
write_cmos_sensor(0x2f42, 0x3f9f);
write_cmos_sensor(0x2f44, 0x490f);
write_cmos_sensor(0x2f46, 0x521f);
write_cmos_sensor(0x2f48, 0x8082);
write_cmos_sensor(0x2f4a, 0x3f5c);
write_cmos_sensor(0x2f4c, 0x421f);
write_cmos_sensor(0x2f4e, 0x00ac);
write_cmos_sensor(0x2f50, 0x3f48);
write_cmos_sensor(0x2f52, 0x421f);
write_cmos_sensor(0x2f54, 0x00a6);
write_cmos_sensor(0x2f56, 0x3f3d);
write_cmos_sensor(0x2f58, 0x4138);
write_cmos_sensor(0x2f5a, 0x4139);
write_cmos_sensor(0x2f5c, 0x413a);
write_cmos_sensor(0x2f5e, 0x413b);
write_cmos_sensor(0x2f60, 0x4130);
write_cmos_sensor(0x2f62, 0xdf02);
write_cmos_sensor(0x2f64, 0x3ffe);
write_cmos_sensor(0x2f66, 0x430e);
write_cmos_sensor(0x2f68, 0x430f);
write_cmos_sensor(0x2f6a, 0x3c08);
write_cmos_sensor(0x2f6c, 0xc312);
write_cmos_sensor(0x2f6e, 0x100d);
write_cmos_sensor(0x2f70, 0x100c);
write_cmos_sensor(0x2f72, 0x2802);
write_cmos_sensor(0x2f74, 0x5a0e);
write_cmos_sensor(0x2f76, 0x6b0f);
write_cmos_sensor(0x2f78, 0x5a0a);
write_cmos_sensor(0x2f7a, 0x6b0b);
write_cmos_sensor(0x2f7c, 0x930c);
write_cmos_sensor(0x2f7e, 0x23f6);
write_cmos_sensor(0x2f80, 0x930d);
write_cmos_sensor(0x2f82, 0x23f4);
write_cmos_sensor(0x2f84, 0x4130);
write_cmos_sensor(0x2f86, 0x4030);
write_cmos_sensor(0x2f88, 0xff66);
write_cmos_sensor(0x2f8a, 0xee0e);
write_cmos_sensor(0x2f8c, 0x403b);
write_cmos_sensor(0x2f8e, 0x0011);
write_cmos_sensor(0x2f90, 0x3c05);
write_cmos_sensor(0x2f92, 0x100d);
write_cmos_sensor(0x2f94, 0x6e0e);
write_cmos_sensor(0x2f96, 0x9a0e);
write_cmos_sensor(0x2f98, 0x2801);
write_cmos_sensor(0x2f9a, 0x8a0e);
write_cmos_sensor(0x2f9c, 0x6c0c);
write_cmos_sensor(0x2f9e, 0x6d0d);
write_cmos_sensor(0x2fa0, 0x831b);
write_cmos_sensor(0x2fa2, 0x23f7);
write_cmos_sensor(0x2fa4, 0x4130);
write_cmos_sensor(0x2fa6, 0xef0f);
write_cmos_sensor(0x2fa8, 0xee0e);
write_cmos_sensor(0x2faa, 0x4039);
write_cmos_sensor(0x2fac, 0x0021);
write_cmos_sensor(0x2fae, 0x3c0a);
write_cmos_sensor(0x2fb0, 0x1008);
write_cmos_sensor(0x2fb2, 0x6e0e);
write_cmos_sensor(0x2fb4, 0x6f0f);
write_cmos_sensor(0x2fb6, 0x9b0f);
write_cmos_sensor(0x2fb8, 0x2805);
write_cmos_sensor(0x2fba, 0x2002);
write_cmos_sensor(0x2fbc, 0x9a0e);
write_cmos_sensor(0x2fbe, 0x2802);
write_cmos_sensor(0x2fc0, 0x8a0e);
write_cmos_sensor(0x2fc2, 0x7b0f);
write_cmos_sensor(0x2fc4, 0x6c0c);
write_cmos_sensor(0x2fc6, 0x6d0d);
write_cmos_sensor(0x2fc8, 0x6808);
write_cmos_sensor(0x2fca, 0x8319);
write_cmos_sensor(0x2fcc, 0x23f1);
write_cmos_sensor(0x2fce, 0x4130);
write_cmos_sensor(0x2fd0, 0x0000);
write_cmos_sensor(0x2ffe, 0xf06e);
write_cmos_sensor(0x3000, 0x33af);
write_cmos_sensor(0x3002, 0xb3af);
write_cmos_sensor(0x3004, 0x0abf);
write_cmos_sensor(0x3006, 0x0003);
write_cmos_sensor(0x4000, 0x0484);
write_cmos_sensor(0x4002, 0x0481);
write_cmos_sensor(0x4004, 0xaf44);
write_cmos_sensor(0x4006, 0xa004);

//    EOFIRM
//end of software code

//=============================================//
//           Mode Select
//=============================================//
//write_cmos_sensor(0x0a00, 0x0000); //stream off

//=============================================//
//           Sreg Set file
//=============================================//
write_cmos_sensor(0x0b00, 0x001a);            
write_cmos_sensor(0x0b02, 0x9885);            
write_cmos_sensor(0x0b04, 0xc540);            
write_cmos_sensor(0x0b06, 0xb540);            
write_cmos_sensor(0x0b08, 0xc085);            
write_cmos_sensor(0x0b0a, 0xd701);            
write_cmos_sensor(0x0b0c, 0x0420);            
write_cmos_sensor(0x0b0e, 0xc600);             
write_cmos_sensor(0x0b10, 0x4d28);             
write_cmos_sensor(0x0b12, 0x0000);             
write_cmos_sensor(0x0b14, 0x0000);             
write_cmos_sensor(0x0b16, 0x2d0b);             
write_cmos_sensor(0x0b18, 0xc009);             
write_cmos_sensor(0x0b1a, 0x0000);             
write_cmos_sensor(0x0b1c, 0x0000);             
write_cmos_sensor(0x0b1e, 0x0081);             
write_cmos_sensor(0x0b20, 0x0000);             
write_cmos_sensor(0x0b22, 0x4c80);             
write_cmos_sensor(0x0b24, 0x0000);             
write_cmos_sensor(0x0b26, 0x0001); 
write_cmos_sensor(0x0b28, 0x4a8f);
write_cmos_sensor(0x0b2a, 0xc808); 
//=============================================//           
//      BLC Set file                             write_cmos_sensor();             
//=============================================//write_cmos_sensor();             
write_cmos_sensor(0x0c00, 0x1190);             
write_cmos_sensor(0x0c02, 0x0011);             
write_cmos_sensor(0x0c04, 0x0000);             
write_cmos_sensor(0x0c06, 0x0200);             
write_cmos_sensor(0x0c10, 0x0040);//act_rr_offset             
write_cmos_sensor(0x0c12, 0x0040);//act_gr_offset             
write_cmos_sensor(0x0c14, 0x0040);//act_gb_offset
write_cmos_sensor(0x0c16, 0x0040);//act_bb_offset
write_cmos_sensor(0x0c18, 0x2020);
//=============================================//
//      TG set file & ISP set file               
//(FMT, Frame/Line length, AG/Int.time,Pixel Arrawrite_cmos_sensor();
//=============================================//write_cmos_sensor();
write_cmos_sensor(0x0000, 0x0100);
write_cmos_sensor(0x004c, 0x0100);            
write_cmos_sensor(0x000c, 0x0022);            
write_cmos_sensor(0x0012, 0x000c);            
write_cmos_sensor(0x0018, 0x0cd3);            
write_cmos_sensor(0x001e, 0x1111);            
write_cmos_sensor(0x0008, 0x0ed8);             
write_cmos_sensor(0x0034, 0x0700);             
write_cmos_sensor(0x0022, 0x0008);             
write_cmos_sensor(0x0028, 0x0017);             
write_cmos_sensor(0x0024, 0x003a);             
write_cmos_sensor(0x002a, 0x0045);             
write_cmos_sensor(0x0026, 0x0048);             
write_cmos_sensor(0x002c, 0x09d7);             
write_cmos_sensor(0x005c, 0x0202);             
write_cmos_sensor(0x002e, 0x1111);             
write_cmos_sensor(0x0030, 0x1111);             
write_cmos_sensor(0x0032, 0x1111);             
write_cmos_sensor(0x0006, 0x09cc);
write_cmos_sensor(0x0a22, 0x0000);             
write_cmos_sensor(0x0a12, 0x0cc0);             
write_cmos_sensor(0x0a14, 0x0990);             
write_cmos_sensor(0x003c, 0x0006);             
write_cmos_sensor(0x003e, 0x0000);             
write_cmos_sensor(0x0004, 0x09C5);             
write_cmos_sensor(0x0054, 0x00f9);             
write_cmos_sensor(0x0002, 0x0000);             
write_cmos_sensor(0x0a02, 0x0100);
write_cmos_sensor(0x0a04, 0x014A);
#ifdef HI842_OTP_Enable
sensor_wb_gain_setting();
#endif
//write_cmos_sensor(0x0508, 0x0200);
//write_cmos_sensor(0x050A, 0x0200);
//write_cmos_sensor(0x050C, 0x0200);
//write_cmos_sensor(0x050E, 0x0200);
write_cmos_sensor(0x0A1A, 0x0800);
write_cmos_sensor(0x0046, 0x0000);
write_cmos_sensor(0x003a, 0x0000);
write_cmos_sensor(0x0050, 0x0400);
write_cmos_sensor(0x0036, 0x007f);
write_cmos_sensor(0x0038, 0x7f00);
write_cmos_sensor(0x004e, 0x7f7f);
write_cmos_sensor(0x0122, 0x0301);
write_cmos_sensor(0x0120, 0x0001);
write_cmos_sensor(0x0156, 0x003f);
write_cmos_sensor(0x0158, 0x3f3f);
write_cmos_sensor(0x0136, 0x0fff);
write_cmos_sensor(0x0138, 0x0fff);
write_cmos_sensor(0x013a, 0x0fff);
write_cmos_sensor(0x013c, 0x0fff);
write_cmos_sensor(0x013e, 0x0fff);
write_cmos_sensor(0x0140, 0x0fff); 
write_cmos_sensor(0x0142, 0x0fff); 
write_cmos_sensor(0x0144, 0x0fff); 
write_cmos_sensor(0x0146, 0x0fff); 
write_cmos_sensor(0x0148, 0x0fff); 
write_cmos_sensor(0x014a, 0x0fff); 
write_cmos_sensor(0x014c, 0x0fff); 
write_cmos_sensor(0x014e, 0x0fff); 
write_cmos_sensor(0x0150, 0x0fff); 
write_cmos_sensor(0x0152, 0x0fff); 
write_cmos_sensor(0x0154, 0x0fff); 
write_cmos_sensor(0x0804, 0x0004); 
write_cmos_sensor(0x0900, 0x0300); 
write_cmos_sensor(0x0902, 0xc319); 


write_cmos_sensor(0x0914, 0xc109);
write_cmos_sensor(0x0916, 0x061a);
write_cmos_sensor(0x0918, 0x0307);
write_cmos_sensor(0x091a, 0x0a0a);
write_cmos_sensor(0x091c, 0x0f07);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x0d7c);
write_cmos_sensor(0x090e, 0x0077);
write_cmos_sensor(0x0a00, 0x0100);


	


}                 
static void preview_setting(void)
{
// Preview Mode Setting                                                                                                                                                            
   LOG_INF("E preview\n");                                                                                           
write_cmos_sensor(0x0a00, 0x0000); //sleep on  
//=============================================//
//           Sreg Set file                       
//=============================================//                     
write_cmos_sensor(0x0b16, 0x2d0b);                         
write_cmos_sensor(0x0b18, 0xc049);   
//=============================================//
//      TG set file & ISP set file               
//(FMT, Frame/Line length, AG/Int.time,Pixel Arra
//=============================================//                      
write_cmos_sensor(0x0012, 0x0008); 
write_cmos_sensor(0x0018, 0x0cd7); 
write_cmos_sensor(0x0024, 0x0038); 
write_cmos_sensor(0x002a, 0x0043);                      
write_cmos_sensor(0x0026, 0x0048);                         
write_cmos_sensor(0x002c, 0x09d7);
write_cmos_sensor(0x005c, 0x0204); 
write_cmos_sensor(0x0032, 0x3311);
write_cmos_sensor(0x0006, 0x09c8);
write_cmos_sensor(0x0a22, 0x0100); 
write_cmos_sensor(0x0a12, 0x0660);
write_cmos_sensor(0x0a14, 0x04c8);                                         
write_cmos_sensor(0x0a04, 0x016a); //0x0162
//========================================================
//            MIPI Speed 360Mbps sub2                     
//========================================================                                                    
write_cmos_sensor(0x0914, 0xc105); 
write_cmos_sensor(0x0916, 0x030c); 
write_cmos_sensor(0x0918, 0x0203); 
write_cmos_sensor(0x091a, 0x0607);
write_cmos_sensor(0x091c, 0x0b04); 
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x06b6);
write_cmos_sensor(0x090e, 0x0034);
//sleep off                                       
write_cmos_sensor(0x0a00, 0x0100);      

}
static void capture_setting(kal_uint16 currefps)
{
// Capture Mode Setting                                                                                       
  LOG_INF("E capture, currefps = %d\n",currefps);  
  if(currefps==300)
  	{                                                                         
write_cmos_sensor(0x0a00, 0x0000);  
                  
write_cmos_sensor(0x0b16, 0x2d0b);                    
write_cmos_sensor(0x0b18, 0xc009);  
//=============================================//
//      TG set file & ISP set file               
//(FMT, Frame/Line length, AG/Int.time,Pixel Arra
//=============================================//                    
write_cmos_sensor(0x0012, 0x000c);
write_cmos_sensor(0x0018, 0x0cd3);
write_cmos_sensor(0x0024, 0x003a);
write_cmos_sensor(0x002a, 0x0045);
write_cmos_sensor(0x0026, 0x0048);
write_cmos_sensor(0x002c, 0x09d7);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x09c8);
write_cmos_sensor(0x0a22, 0x0000); 
write_cmos_sensor(0x0a12, 0x0cc0);
write_cmos_sensor(0x0a14, 0x0990);
write_cmos_sensor(0x0a04, 0x014a);    //0x0142  
//========================================================
//            MIPI Speed 720Mbps Normal
//========================================================              
write_cmos_sensor(0x0914, 0xc109);
write_cmos_sensor(0x0916, 0x061a);
write_cmos_sensor(0x0918, 0x0307);
write_cmos_sensor(0x091a, 0x0a0a);
write_cmos_sensor(0x091c, 0x0f07);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x0d7c);
write_cmos_sensor(0x090e, 0x0077);              

write_cmos_sensor(0x0a00, 0x0100); 
}
else
	{
write_cmos_sensor(0x0a00, 0x0000);  
                  
write_cmos_sensor(0x0b16, 0x2d0b);                    
write_cmos_sensor(0x0b18, 0xc009);  
//=============================================//
//      TG set file & ISP set file
//(FMT, Frame/Line length, AG/Int.time,Pixel Array)
//=============================================//
write_cmos_sensor(0x0012, 0x000c);
write_cmos_sensor(0x0018, 0x0cd3);
write_cmos_sensor(0x0024, 0x003a);
write_cmos_sensor(0x002a, 0x0045);
write_cmos_sensor(0x0026, 0x0048);
write_cmos_sensor(0x002c, 0x09d7);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x13b8);
write_cmos_sensor(0x0a22, 0x0000); 
write_cmos_sensor(0x0a12, 0x0cc0);
write_cmos_sensor(0x0a14, 0x0990);
write_cmos_sensor(0x0a04, 0x014a);//0x0142
//========================================================
//            MIPI Speed 720Mbps Normal
//========================================================
write_cmos_sensor(0x0914, 0xc109);
write_cmos_sensor(0x0916, 0x061a);
write_cmos_sensor(0x0918, 0x0307);
write_cmos_sensor(0x091a, 0x0a0a);
write_cmos_sensor(0x091c, 0x0f07);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x0d7c);
write_cmos_sensor(0x090e, 0x0077);            

write_cmos_sensor(0x0a00, 0x0100); 
		
		}
}
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	
		// Video Mode Setting 
write_cmos_sensor(0x0a00, 0x0000);  
                  
write_cmos_sensor(0x0b16, 0x2d0b);
write_cmos_sensor(0x0b18, 0xc009);
//=============================================//
//      TG set file & ISP set file
//(FMT, Frame/Line length, AG/Int.time,Pixel Array)
//=============================================//
write_cmos_sensor(0x0012, 0x000c);
write_cmos_sensor(0x0018, 0x0cd3);
write_cmos_sensor(0x0024, 0x003a);
write_cmos_sensor(0x002a, 0x0045);
write_cmos_sensor(0x0026, 0x0048);
write_cmos_sensor(0x002c, 0x09d7);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x09c8);
write_cmos_sensor(0x0a22, 0x0000); 
write_cmos_sensor(0x0a12, 0x0cc0);
write_cmos_sensor(0x0a14, 0x0990);
write_cmos_sensor(0x0a04, 0x014a);//0x0142
//========================================================
//            MIPI Speed 720Mbps Normal
//========================================================
write_cmos_sensor(0x0914, 0xc109);
write_cmos_sensor(0x0916, 0x061a);
write_cmos_sensor(0x0918, 0x0307);
write_cmos_sensor(0x091a, 0x0a0a);
write_cmos_sensor(0x091c, 0x0f07);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x0d7c);
write_cmos_sensor(0x090e, 0x0077);                            

write_cmos_sensor(0x0a00, 0x0100);  // sleep on        
}
static void hs_video_setting(void)
{
    LOG_INF("E\n");
//Preview Mode VGA Setting                                                       
// MIPI Setting  
if(SCALE == 1)
	{  
		LOG_INF("Enter scale hs\n");                                                            
write_cmos_sensor(0x0a00, 0x0000);  

write_cmos_sensor(0x0b16, 0x2d0b);
write_cmos_sensor(0x0b18, 0xc089); 
//=============================================//
//      TG set file & ISP set file
//(FMT, Frame/Line length, AG/Int.time,Pixel Array)
//=============================================//
write_cmos_sensor(0x0012, 0x0160);
write_cmos_sensor(0x0018, 0x0b7f);
write_cmos_sensor(0x0024, 0x013c);
write_cmos_sensor(0x002a, 0x0147);
write_cmos_sensor(0x0026, 0x0150);
write_cmos_sensor(0x002c, 0x08cf);
write_cmos_sensor(0x005c, 0x0208);
write_cmos_sensor(0x0032, 0x7711);
write_cmos_sensor(0x0006, 0x0276);
write_cmos_sensor(0x0a22, 0x0200); 
write_cmos_sensor(0x0a12, 0x0280);
write_cmos_sensor(0x0a14, 0x01e0);
write_cmos_sensor(0x0a04, 0x016a);//0x0162
//========================================================
//            MIPI Speed 180Mbps VGA
//========================================================
write_cmos_sensor(0x0914, 0xc104); 
write_cmos_sensor(0x0916, 0x0108); 
write_cmos_sensor(0x0918, 0x0202); 
write_cmos_sensor(0x091a, 0x0306); 
write_cmos_sensor(0x091c, 0x0902); 
write_cmos_sensor(0x091e, 0x0300); 
write_cmos_sensor(0x090c, 0x034b); 
write_cmos_sensor(0x090e, 0x0045);                                                           
	                                                                           
write_cmos_sensor(0x0a00, 0x0100);   // sleep on     
}
else
	{
    LOG_INF("Enter crop hs\n"); 
write_cmos_sensor(0x0a00, 0x0000);  


                                                   
write_cmos_sensor(0x0b16, 0x2d0b);                                                     
write_cmos_sensor(0x0b18, 0xc009);

                                                
write_cmos_sensor(0x0012, 0x052c);                                                
write_cmos_sensor(0x0018, 0x07b3);                                                
write_cmos_sensor(0x0024, 0x0412);                                                
write_cmos_sensor(0x002a, 0x041d);                                                
write_cmos_sensor(0x0026, 0x0420);                                                                                                                     
write_cmos_sensor(0x002c, 0x05ff);                  
write_cmos_sensor(0x005c, 0x0202);                  
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x0276);
write_cmos_sensor(0x0a22, 0x0000);                  
write_cmos_sensor(0x0a12, 0x0280);
write_cmos_sensor(0x0a14, 0x01e0);
write_cmos_sensor(0x0a04, 0x014a);//0x0142
//========================================================
//            MIPI Speed 720Mbps VGA
//========================================================
write_cmos_sensor(0x0914, 0xc109);
write_cmos_sensor(0x0916, 0x061a);
write_cmos_sensor(0x0918, 0x0307);
write_cmos_sensor(0x091a, 0x0a0a);
write_cmos_sensor(0x091c, 0x0f07);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x0d6f);                  
write_cmos_sensor(0x090e, 0x039e);                                                             
	                                                                           
write_cmos_sensor(0x0a00, 0x0100);	
		
	}           
}

static void slim_video_setting(void)
{
    LOG_INF("E\n");
    //@@video_720p_30fps_800Mbps
    // Preview Mode VGA Setting      
write_cmos_sensor(0x0a00, 0x0000); 

write_cmos_sensor(0x0b16, 0x2d0b); 
write_cmos_sensor(0x0b18, 0xc049); 
//=============================================//
//      TG set file & ISP set file
//(FMT, Frame/Line length, AG/Int.time,Pixel Array)
//=============================================//
write_cmos_sensor(0x0012, 0x0168);
write_cmos_sensor(0x0018, 0x0b77);
write_cmos_sensor(0x0024, 0x0230);
write_cmos_sensor(0x002a, 0x023b);
write_cmos_sensor(0x0026, 0x0240);
write_cmos_sensor(0x002c, 0x07df);
write_cmos_sensor(0x005c, 0x0204);
write_cmos_sensor(0x0032, 0x3311);
write_cmos_sensor(0x0006, 0x09c8);
write_cmos_sensor(0x0a22, 0x0100); 
write_cmos_sensor(0x0a12, 0x0500);
write_cmos_sensor(0x0a14, 0x02d0);
write_cmos_sensor(0x0a04, 0x016a);//0x0162
//========================================================
//            MIPI Speed 360Mbps HD
//========================================================
write_cmos_sensor(0x0914, 0xc105);
write_cmos_sensor(0x0916, 0x030c);
write_cmos_sensor(0x0918, 0x0203);
write_cmos_sensor(0x091a, 0x0607);
write_cmos_sensor(0x091c, 0x0b04);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x090c, 0x06b6);
write_cmos_sensor(0x090e, 0x00a2);       

write_cmos_sensor(0x0a00, 0x0100);  // sleep on     
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{

LOG_INF("[get_imgsensor_id] ");
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
				*sensor_id =return_sensor_id();
				if (*sensor_id == imgsensor_info.sensor_id) {				
					LOG_INF("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  

			// If read Hi-842 Chip ID Pass, Read OTP Data. 
				#ifdef HI842_OTP_Enable
				read_Hi842_otp();
				#endif
				
					return ERROR_NONE;
				}	
				LOG_INF("get_imgsensor_id Read sensor id fail, i2c write id: 0x%x,sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				retry--;
			} while(retry > 0);
			i++;
			retry = 2;
}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	LOG_INF("[open] ");
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_INF("[open]: PLATFORM:MT6735,MIPI 4LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");

	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("SP\n");
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("open:Read sensor id fail open i2c write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
  imgsensor.test_pattern = KAL_FALSE;
  
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("close E");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_INF("Warning: current_fps  fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);   
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	capture_setting(imgsensor.current_fps); 
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */






static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("[contrlo]scenario_id = %d", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		
			LOG_INF("preview\n");
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	
	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{

	
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
         //   set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d", enable);

	if (enable) { 
		  LOG_INF("enter color bar");            
		// 0x5E00[8]: 1 enable,  0 disable               
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0a04, 0x0149);    //0x0141           
		write_cmos_sensor(0x020a, 0x0200);	             
	} else {               
		// 0x5E00[8]: 1 enable,  0 disable               
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0a04, 0x014a); // 0x0142          
		write_cmos_sensor(0x020a, 0x0000);               
	}	 
	spin_lock(&imgsensor_drv_lock);
  imgsensor.test_pattern = enable;
  spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
       /* case SENSOR_FEATURE_GET_FRAMELENGTH: //for PIXEL MODE, add by lpf
            *feature_return_para_32 = imgsensor.frame_length;
            *feature_para_len=4;
            break;*/
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI842_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	HI842_MIPI_RAW_SensorInit	*/
