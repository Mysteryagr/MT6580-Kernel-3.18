/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sp0a20_yuv_Sensor.h
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver declare and macro define in the header file.
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2011/10/25 Firsty Released By Mormo;
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
 
#ifndef __SP0A20_SENSOR_H
#define __SP0A20_SENSOR_H

//#include "image_sensor.h"//get IMAGE_SENSOR_DRVNAME
//#include "kd_imgsensor.h"

    //------------------------Engineer mode---------------------------------
#define FACTORY_START_ADDR 	0
#define ENGINEER_START_ADDR	10

    typedef enum group_enum {
       PRE_GAIN=0,
	   CMMCLK_CURRENT,
	   FRAME_RATE_LIMITATION,
	   REGISTER_EDITOR,
	   GROUP_TOTAL_NUMS
    } FACTORY_REGISTER_INDEX;

    typedef enum register_index {
        SENSOR_BASEGAIN=FACTORY_START_ADDR,
	      PRE_GAIN_R_INDEX,
	      PRE_GAIN_Gr_INDEX,
	      PRE_GAIN_Gb_INDEX,
	      PRE_GAIN_B_INDEX,
	      FACTORY_END_ADDR
    } CCT_REGISTER_INDEX;
    
typedef enum engineer_index
{   
	CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
	ENGINEER_END
} FACTORY_ENGINEER_INDEX; 

//------------------------Engineer mode---------------------------------
typedef struct {
    SENSOR_REG_STRUCT Reg[ENGINEER_END];
    SENSOR_REG_STRUCT CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT,*PSENSOR_DATA_STRUCT;



#define VGA_PERIOD_PIXEL_NUMS						784
#define VGA_PERIOD_LINE_NUMS						510

#define IMAGE_SENSOR_VGA_GRAB_PIXELS			0
#define IMAGE_SENSOR_VGA_GRAB_LINES			1

#define IMAGE_SENSOR_VGA_WIDTH					(640)
#define IMAGE_SENSOR_VGA_HEIGHT					(480)

#define IMAGE_SENSOR_PV_WIDTH					(IMAGE_SENSOR_VGA_WIDTH - 8)
#define IMAGE_SENSOR_PV_HEIGHT					(IMAGE_SENSOR_VGA_HEIGHT - 6)

#define IMAGE_SENSOR_FULL_WIDTH					(IMAGE_SENSOR_VGA_WIDTH - 8)
#define IMAGE_SENSOR_FULL_HEIGHT					(IMAGE_SENSOR_VGA_HEIGHT - 6)

#define SP0A20_WRITE_ID							    0x42
#define SP0A20_READ_ID								0x43

// SP0A20 SENSOR Chip ID: 0xd0

struct SP0A20_Sensor_Struct
{
	struct i2c_client *i2c_clit;
	MSDK_SENSOR_CONFIG_STRUCT cfg_data;
	SENSOR_DATA_STRUCT eng; /* engineer mode */
	MSDK_SENSOR_ENG_INFO_STRUCT eng_info;

	
	kal_bool sensor_night_mode;
	kal_bool MPEG4_encode_mode;

	kal_uint16 dummy_pixels;
	kal_uint16 dummy_lines;
	kal_uint16 extra_exposure_lines;
	kal_uint16 exposure_lines;

	kal_bool MODE_CAPTURE;
	kal_uint16 iBackupExtraExp;


	
	kal_uint32 fPV_PCLK; //26000000;
	kal_uint16 iPV_Pixels_Per_Line;

	kal_bool  bNight_mode; // to distinguish night mode or auto mode, default: auto mode setting
	kal_bool  bBanding_value; // to distinguish between 50HZ and 60HZ.
	kal_uint8 u8Wb_value;
	kal_uint8 u8Effect_value;
	kal_uint8 u8Ev_value;
};
UINT32 SP0A20Open(void);
UINT32 SP0A20Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 SP0A20FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 SP0A20GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 SP0A20GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 SP0A20Close(void);

#endif /* __SENSOR_H */

