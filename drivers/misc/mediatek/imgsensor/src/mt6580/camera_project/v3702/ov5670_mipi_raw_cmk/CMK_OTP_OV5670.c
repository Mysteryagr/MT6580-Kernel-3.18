/*
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function
bool otp_update_wb(unsigned short golden_rg, unsigned short golden_bg),
then the calibration of AWB will be applied. 
After finishing the OTP written, we will provide you the golden_rg and golden_bg settings.
*/
/*
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
	
#include "ov5670_Sensor.h"
#include "ov5670_Camera_Sensor_para.h"
#include "ov5670_CameraCustomized.h"
*/


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

//#include "ov5670mipi_Sensor_cmk.h"
//#include "ov5670mipi_Camera_Sensor_para_cmk.h"
//#include "ov5670mipi_CameraCustomized_cmk.h"
#include "ov5670mipi_Sensor_cmk.h"
//#include "ov5670mipiraw_Camera_Sensor_para.h"
//#include "ov5670mipiraw_CameraCustomized.h"

//#undef printk
//#define printk(fmt, args...) printk(KERN_INFO "ov5670_OTP.c: " fmt, ## args)
//#define printk(fmt, arg...) printk("[ov5670MIPIRaw_OTP_CMK]%s: " fmt "\n", __FUNCTION__ ,##arg)//LINE <> <DATE20130923> <ov5670 OTP log> wupingzhou

extern kal_uint16 ov5670MIPI_read_cmos_sensor_cmk(kal_uint32 addr);
extern void ov5670MIPI_write_cmos_sensor_cmk(kal_uint32 addr, kal_uint32 para);

//#define ov5670_write_cmos_sensor(kal_uint32 addr, kal_uint32 para) ov5670MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
//#define ov5670_read_cmos_sensor(kal_uint32 addr) ov5670MIPI_read_cmos_sensor(kal_uint32 addr)

#define ov5670_write_cmos_sensor(addr,para) ov5670MIPI_write_cmos_sensor_cmk(addr,para)
#define ov5670_read_cmos_sensor(addr) ov5670MIPI_read_cmos_sensor_cmk(addr)

//#define SUPPORT_FLOATING

#define OTP_DATA_ADDR         0x3D00
#define OTP_LOAD_ADDR         0x3D81

#define OTP_WB_GROUP_ADDR     0x3D05
#define OTP_WB_GROUP_SIZE     9
#define OTP_BANK_ADDR         0x3D84
#define OTP_BANK              0x3D85
#define OTP_END_ADDR          0x3D86


#define GAIN_DEFAULT_VALUE    0x0400 // 1x gain

#define RG_Ratio_Typical 	1  
#define BG_Ratio_Typical 	1  

#define OTP_MID               0x08
//#define LENS_ID              0x01


// R/G and B/G of current camera module
static unsigned char RG_MSB = 0;
static unsigned char BG_MSB = 0;
static unsigned char AWB_LSB = 0;
signed char check_cmk_MID_ov5670(bool bOnlyCheck);


/*******************************************************************************
* Function    :  otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool otp_update_wb_cmk_ov5670(unsigned short golden_rg, unsigned short golden_bg) 
{
	printk("start wb update\n");
	int R_gain, G_gain, B_gain, Base_gain,G_gain_R,G_gain_B;
//	unsigned short rg_ratio ;
//	unsigned short bg_ratio ;
     int rg_ratio;
     int bg_ratio;
     
	check_cmk_MID_ov5670(false);
	
	if((RG_MSB == 0)&&(BG_MSB == 0))
	{
		 return 0;
	    
	}
	else
	{
		//	rg_ratio = RG_MSB<<2 + ((AWB_LSB>>6)&0x03);
			  rg_ratio =( RG_MSB<<2) + ((AWB_LSB & 0xc0) >> 6); 
			   bg_ratio = (BG_MSB<<2) +((AWB_LSB & 0x30) >> 4);
		  
		/*  	
		  printk("RG_MSB << 2 = 0x%x \n",RG_MSB);
			printk("BG_MSB <<2 = 0x%x \n",BG_MSB);
			printk("AWB_LSB = 0x%x \n",AWB_LSB);
			printk("(AWB_LSB& 0xc0) = 0x%x \n",(AWB_LSB& 0xc0) );
			printk("(AWB_LSB>>4)= 0x%x \n",(AWB_LSB>>4) );
			printk("((AWB_LSB>>4)& 0x03)= 0x%x \n",((AWB_LSB>>4)& 0x03) );
			printk("rg_ratio = 0x%x \n",rg_ratio);
			printk("bg_ratio  = 0x%x \n",bg_ratio);
			
	*/
		  
		 
			  
			
			if(bg_ratio < golden_bg)
				{
					if (rg_ratio < golden_rg)
						{
							  G_gain = 0x400;
							  B_gain = 0x400 * golden_bg / bg_ratio;
							  R_gain = 0x400 * golden_rg / rg_ratio;
						}
						else
							{
								
								R_gain = 0x400;
								G_gain = 0x400 * rg_ratio / golden_rg;
								B_gain = G_gain * golden_bg / bg_ratio;
							}
				}
				else
					{
						if (rg_ratio < golden_rg)
							{
								B_gain = 0x400;
								G_gain = 0x400 * bg_ratio / golden_bg;
								R_gain = 0x400 * golden_rg / rg_ratio;
							
							}
							else
								{
									G_gain_B = 0x400 * bg_ratio / golden_bg;
									G_gain_R = 0x400 * rg_ratio /golden_rg;
									
									if(G_gain_B > G_gain_R)
										{
											B_gain = 0x400;
											G_gain = G_gain_B;
											R_gain = G_gain * golden_rg / rg_ratio;
										}
										else
											{
												R_gain = 0x400;
												G_gain = G_gain_R;
												B_gain = G_gain * golden_bg / bg_ratio;
											}
								}
					}
				
				
				
		// update sensor WB gain
		if (R_gain>0x400) {
		ov5670_write_cmos_sensor(0x5032, R_gain>>8);
		ov5670_write_cmos_sensor(0x5033, R_gain & 0x00ff);
		}
		if (G_gain>0x400) {
		ov5670_write_cmos_sensor(0x5034, G_gain>>8);
		ov5670_write_cmos_sensor(0x5035, G_gain & 0x00ff);
		}
		if (B_gain>0x400) {
		ov5670_write_cmos_sensor(0x5036, B_gain>>8);
		ov5670_write_cmos_sensor(0x5037, B_gain & 0x00ff);
		}
		
		  printk("[ov5670 ]rg_raio = 0x%x \n",rg_ratio);
			printk("[ov5670 ]bg_ratio = 0x%x \n",bg_ratio);
			printk("[ov5670 ]golden_rg = 0x%x \n",golden_rg);
			printk("[ov5670 ]golden_bg = 0x%x \n",golden_bg);
			printk("[ov5670 ]R_gain = 0x%x \n",R_gain);
			printk("[ov5670 ]B_gain = 0x%x \n",B_gain);
			printk("[ov5670 ]G_gain = 0x%x \n",G_gain);
	   printk("wb update failed\n");
		return 1;

	}
	   
	return 0;
}


signed char check_cmk_MID_ov5670(bool bOnlyCheck)
{

		int otp_flag, addr, temp, i,MID_ID;
		int temp1;

		
		ov5670_write_cmos_sensor(0x100, 0x01);
		
		temp1 = ov5670_read_cmos_sensor(0x5002);
		
		printk("temp1 == %x \n",temp1);
		ov5670_write_cmos_sensor(0x5002, (0x00 & 0x08) | (temp1 & (~0x08)));
		// read OTP into buffer
		ov5670_write_cmos_sensor(0x3d84, 0xC0);
		ov5670_write_cmos_sensor(0x3d88, 0x70); // OTP start address
		ov5670_write_cmos_sensor(0x3d89, 0x10);
		ov5670_write_cmos_sensor(0x3d8A, 0x70); // OTP end address
		ov5670_write_cmos_sensor(0x3d8B, 0x29);
		ov5670_write_cmos_sensor(0x3d81, 0x01); // load otp into buffer

		mdelay(10);

		otp_flag = ov5670_read_cmos_sensor(0x7010);
		addr = 0;
		
		printk("otp_flag == %x \n",otp_flag);
		if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7011; // base address of info group 1
		}
		else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7016; // base address of info group 2
		}
		else if((otp_flag & 0x0c) == 0x04) {
		addr = 0x701b; // base address of info group 3
		}
		if(addr == 0)
		{
		
		MID_ID = ov5670_read_cmos_sensor(0x7011);
		printk("MID_ID1 == %x \n",MID_ID);
		MID_ID = ov5670_read_cmos_sensor(0x7016);
		printk("MID_ID2 == %x \n",MID_ID);
		MID_ID = ov5670_read_cmos_sensor(0x701b);
		printk("MID_ID3 == %x \n",MID_ID);
			return 0;
		}
		else
		{
			MID_ID = ov5670_read_cmos_sensor(addr);
		}
		
		printk("MID_ID == %x \n",MID_ID);
		if(MID_ID ==  0x08)
		{
		if(bOnlyCheck)
			return 1;
			otp_flag = ov5670_read_cmos_sensor(0x7020);
			addr = 0;
			if((otp_flag & 0xc0) == 0x40) {
			addr = 0x7021; // base address of WB Calibration group 1
			}
			else if((otp_flag & 0x30) == 0x10) {
			addr = 0x7024; // base address of WB Calibration group 2
			}
			else if((otp_flag & 0x0c) == 0x04) {
			addr = 0x7027; // base address of WB Calibration group 3
			}

			if(addr != 0) {
			AWB_LSB = ov5670_read_cmos_sensor( addr + 2);
			RG_MSB = ov5670_read_cmos_sensor(addr);
			BG_MSB = ov5670_read_cmos_sensor( addr + 1);
			
			
			}
			else {
			AWB_LSB =0;
			RG_MSB = 0;
			BG_MSB = 0;
			}
			
		
			printk("AWB_LSB = 0x%x \n",AWB_LSB);
			printk("RG_MSB = 0x%x \n",RG_MSB);
			printk("BG_MSB = 0x%x \n",BG_MSB);
			for(i=0x7010;i<=0x7029;i++) {
			ov5670_write_cmos_sensor(i,0); // clear OTP buffer, recommended use continuous write to accelarate
			}
			//set 0x5002[3] to â€?â€?
			temp1 = ov5670_read_cmos_sensor(0x5002);
			ov5670_write_cmos_sensor(0x5002, (0x02 & 0x08) | (temp1 & (~0x08)));
			return 1;
		}
		else
			return 0;
		
		 

		
	}


