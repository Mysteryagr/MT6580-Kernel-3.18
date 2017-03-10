#ifndef TPD_CUSTOM_EKTF2K_H__
#define TPD_CUSTOM_EKTF2K_H__

#define ELAN_X_MAX      760 
#define ELAN_Y_MAX      1280

#define LCM_X_MAX      480 //simple_strtoul(LCM_WIDTH, NULL, 0)
#define LCM_Y_MAX      854	//simple_strtoul(LCM_HEIGHT, NULL, 0)
#define KEYCODE_APP_SWITCH KEY_F17
//Elan Key's define
#define ELAN_KEY_BACK		0x10
#define ELAN_KEY_HOME		0x08
#define ELAN_KEY_MENU		0x04
#define ELAN_KEY_SEARCH 	0x11

/////////////////////////////////////////////////////////
#define I2C_NUM 1
#define TPD_POWER_SOURCE_CUSTOM	MT6323_POWER_LDO_VGP2

//#define SOFTKEY_AXIS_VER
//#define ELAN_TEN_FINGERS
//#define _DMA_MODE_

#ifdef CONFIG_PROJECT_V2800_BLU_US
#else
#define ELAN_BUTTON
#define __ELAN_TRUE_BUTTON__
#endif

//#define LCT_VIRTUAL_KEY
//#define TPD_HAVE_BUTTON
//#define ELAN_3K_IC_SOLUTION

//#define NON_MTK_MODE	//I2C Support > 8bits Transfer

#define MTK_ELAN_DEBUG

#ifdef ELAN_TEN_FINGERS
#define PACKET_SIZE             44            /* support 10 fingers packet */
#else
#define PACKET_SIZE             8            	/* support 2 fingers packet  */
//#define PACKET_SIZE            	18            /* support 5 fingers packet  */
#endif

#define ELAN_DEBUG

#if defined(CUSTOM_KERNEL_PS)
#define TP_PROXIMITY_SENSOR_NEW //tp proximity
#endif
#define ELAN_2527_IC
#define IAP_PORTION                     //upgrade  FW
#define FACTORY_UPDATE_FIRMWARE                    //upgrade  FW
#define ESD_CHECK
#endif /* TOUCHPANEL_H__ */
