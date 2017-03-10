#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
    #include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    
#endif

// ---------------------------------------------------------------------------
//RGK add
// ---------------------------------------------------------------------------
//#include <cust_adc.h>        // zhoulidong  add for lcm detect
#define AUXADC_LCM_VOLTAGE_CHANNEL     12
#define AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL     1

#define MIN_VOLTAGE (1300)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (1500)     // zhoulidong  add for lcm detect

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                          (720)
#define FRAME_HEIGHT                                         (1280)

#define LCM_ID_HX8394                                    0x8394


#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
    LCM_PRINT ("[LCM-hx8394f-txd] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                             (lcm_util.udelay(n))
#define MDELAY(n)                                             (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)            lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                            lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE                            0

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static LCM_setting_table_V3 lcm_initialization_setting[] = {

		{0x39,0xB9,3,{0xFF,0x83,0x94}},
		{0x39,0xBA,6,{0x62,0x03,0x68,0x5F,0xB2,0xC0}},    //0x7B-->0x5F  
		{0x39,0xB1,10,{0x50,0x12,0x72,0x09,0x33,0x54,0x91,0x31,0x6B,0x2F}},
		{0x39,0xB2,6,{0x00,0x80,0x64,0x0E,0x0D,0x2F}},
		{0x39,0xB4,21,{0x73,0x74,0x73,0x74,0x73,0x74,0x01,0x15,0x84,0x35,0x40,0x1F,0x73,0x74,0x73,0x74,0x73,0x74,0x01,0x0C,0x86}},
		{0x39,0xB6,2,{0x73,0x73}},
		{0x39,0xD3,33,{0x00,0x00,0x07,0x07,0x40,0x07,0x10,0x00,0x08,0x10,0x08,0x00,0x08,0x54,0x15,0x0E,0x05,0x0E,0x02,0x15,0x06,0x05,0x06,0x47,0x44,0x0A,0x0A,0x4B,0x10,0x07,0x07,0x0E,0x40}},
		           
		{0x39,0xD5,44,{0x1A,0x1A,0x1B,0x1B,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x24,0x25,0x18,0x18,0x26,0x27,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x20,0x21,0x18,0x18,0x18,0x18}},
		           
		{0x39,0xD6,44,{0x1A,0x1A,0x1B,0x1B,0x0B,0x0A,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x21,0x20,0x18,0x18,0x27,0x26,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x25,0x24,0x18,0x18,0x18,0x18}},
		{0x39,0xE0,58,{0x00,0x0C,0x19,0x20,0x23,0x26,0x29,0x28,0x51,0x61,0x70,0x6F,0x76,0x86,0x89,0x8D,0x99,0x9A,0x95,0xA1,0xB0,0x57,0x55,0x58,0x5C,0x5E,0x64,0x6B,0x7F,0x00,0x0C,0x18,0x20,0x23,0x26,0x29,0x28,0x51,0x61,0x70,0x6F,0x76,0x86,0x89,0x8D,0x99,0x9A,0x95,0xA1,0xB0,0x57,0x55,0x58,0x5C,0x5E,0x64,0x6B,0x7F}},
		{0x39,0xC0,3,{0x1F,0x31,0x34}},
		{0x15,0xCC,1,{0X03}},
		{0x15,0xD4,1,{0X02}},
		{0x15,0xBD,1,{0X02}},
		{0x39,0xD8,12,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
		           
		{0x15,0xBD,1,{0X00}},
		{0x15,0xBD,1,{0X01}},
		{0x15,0xB1,1,{0X60}},
		{0x15,0xBD,1,{0X00}},
		{0x39,0xBF,7,{0x40,0x81,0x50,0x00,0x1A,0xFC,0x0C}},// 7TH 01-->0C
		{0x15,0xCC,1,{0X09}},
		{0x15,0x21,1,{0X00}},	
		
		{0x15,0x35,1,{0X00}}, // 35 off te
	    {0x05, 0x11,1,{0x00}},
	    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},

	    {0x05, 0x29,1,{0x00}},
	    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 20, {}},
};

static LCM_setting_table_V3  lcm_deep_sleep_mode_in_setting[] = {

	
    {0x05, 0x28, 0, {}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

    // Sleep Mode On
    {0x05, 0x10, 0, {}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

        memset(params, 0, sizeof(LCM_PARAMS));

        params->type   = LCM_TYPE_DSI;

        params->width  = FRAME_WIDTH;
        params->height = FRAME_HEIGHT;

        params->physical_width  = 62.10;
        params->physical_height = 110.40;

        #if (LCM_DSI_CMD_MODE)
        params->dsi.mode   = CMD_MODE;
        #else
        params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
        #endif

        // DSI
        /* Command mode setting */
        //1 Three lane or Four lane
        params->dsi.LANE_NUM                = LCM_THREE_LANE;
        //The following defined the fomat for data coming from LCD engine.
        params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

        // Video mode setting
        params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                =4;//6;
    params->dsi.vertical_backporch                   =12; //6;
    params->dsi.vertical_frontporch                   =15;//6;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;


    params->dsi.horizontal_sync_active                = 10;//16;///////////////20 20 4  20  14  6
    params->dsi.horizontal_backporch                = 60;//15;
    params->dsi.horizontal_frontporch                = 60;//6;
    params->dsi.horizontal_active_pixel                = FRAME_WIDTH;

    params->dsi.PLL_CLOCK=260; //234;
    params->dsi.ssc_disable=1; // 1

    params->dsi.cont_clock = 0;
    params->dsi.clk_lp_per_line_enable = 1; // per line to lp11 for esd

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;//1;

    #ifdef BUILD_LK
    #else
    #if defined(BUILD_UBOOT)
    #else
    params->dsi.esd_check_for8394f_enable =1;      // FOR 94F   by peter wang 
    #endif
    #endif

    params->dsi.lcm_esd_check_table[0].cmd	   = 0x45;//	 45	 0ff0  051d
    params->dsi.lcm_esd_check_table[0].count	   = 2;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x05;// 	00
    params->dsi.lcm_esd_check_table[0].para_list[1] = 0x1d;// 	0c
    
    params->dsi.lcm_esd_check_table[1].cmd          = 0x09;//09                   
    params->dsi.lcm_esd_check_table[1].count        = 2;
    params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;// two  
    params->dsi.lcm_esd_check_table[1].para_list[1] = 0x73;// two  

    params->dsi.lcm_esd_check_table[2].cmd          = 0xd9;//09
    params->dsi.lcm_esd_check_table[2].count        = 2;
    params->dsi.lcm_esd_check_table[2].para_list[0] = 0x80;// two
    params->dsi.lcm_esd_check_table[2].para_list[1] = 0x01;// two

		
}


static void lcm_init(void)
{
	LCM_DBG();
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

}

static void lcm_suspend(void)
{

    dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20); // 1ms

    SET_RESET_PIN(1);
    MDELAY(120);
}

static void lcm_resume(void)
{
    LCM_DBG();
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;
	
	int array[4];
	char buffer[4]={0,0,0,0};
	int id=0,id_high =0,id_low=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0]=0x00043902;
	array[1]=0x9483ffB9;
	dsi_set_cmdq(array, 2, 1);

	MDELAY(2);
	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x04, buffer, 2);
	
	id_high = buffer[0];
	id_low = buffer[1];

	id = (id_high<<8) | id_low;
	
	LCM_DBG("read id=0x%x, id1=0x%x, id2=0x%x",id, id_high,id_low);

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
    LCM_DBG("read adc error");
    return 0;

    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

   	LCM_DBG("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
   
    if ((lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE )&&(LCM_ID_HX8394==id))
    {
    	return 1;
    }
    return 0;

}// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
	LCM_DBG();
}

static unsigned int lcm_esd_recover(void)
{
	LCM_DBG();
    lcm_init();
    return TRUE;
}
// zhoulidong add for eds(end)
LCM_DRIVER hx8394f_hd4200_dsi_vdo_3lanes_txd_lcm_drv =
{
        .name            = "hx8394f_hd4200_dsi_vdo_3lanes_txd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
};


