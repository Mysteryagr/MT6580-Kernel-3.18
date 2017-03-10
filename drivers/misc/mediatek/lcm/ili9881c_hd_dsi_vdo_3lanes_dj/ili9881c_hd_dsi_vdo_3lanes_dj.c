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
   //#include <mach/mt_gpio.h>
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

#define LCM_ID_ILI9881                                    0x9881


#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif

 unsigned static int lcm_esd_test = FALSE;      ///only for ESD test

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
extern int auxadc_test(void ) ;

static LCM_setting_table_V3 lcm_initialization_setting[] = {
		{0x39,0xFF,3,{0x98,0x81,0x03}},
		{0x15,0x01,1,{0X00}},
		{0x15,0x02,1,{0X00}},
		{0x15,0x03,1,{0X73}},
		{0x15,0x04,1,{0X00}},
		{0x15,0x05,1,{0X00}},
		{0x15,0x06,1,{0X0C}},
		{0x15,0x07,1,{0X00}},
		{0x15,0x08,1,{0X00}},
		{0x15,0x09,1,{0X01}},
		{0x15,0x0a,1,{0X01}},
		{0x15,0x0b,1,{0X01}},
		{0x15,0x0c,1,{0X01}},
		{0x15,0x0d,1,{0X01}},
		{0x15,0x0e,1,{0X01}},
		{0x15,0x0f,1,{0X20}},
		{0x15,0x10,1,{0X20}},
		{0x15,0x11,1,{0X00}},
		{0x15,0x12,1,{0X00}},
		{0x15,0x13,1,{0X02}},
		{0x15,0x14,1,{0X00}},
		{0x15,0x15,1,{0X05}},
		{0x15,0x16,1,{0X00}},
		{0x15,0x17,1,{0X00}},
		{0x15,0x18,1,{0X00}},
		{0x15,0x19,1,{0X00}},
		{0x15,0x1a,1,{0X00}},
		{0x15,0x1b,1,{0X00}},
		{0x15,0x1c,1,{0X00}},
		{0x15,0x1d,1,{0X00}},
		{0x15,0x1e,1,{0X44}},
		{0x15,0x1f,1,{0XC1}},
		{0x15,0x20,1,{0X0A}},
		{0x15,0x21,1,{0X03}},
		{0x15,0x22,1,{0X0A}},
		{0x15,0x23,1,{0X03}},
		{0x15,0x24,1,{0X8C}},
		{0x15,0x25,1,{0X8C}},
		{0x15,0x26,1,{0X00}},
		{0x15,0x27,1,{0X00}},
		{0x15,0x28,1,{0X3B}},
		{0x15,0x29,1,{0X03}},
		{0x15,0x2a,1,{0X00}},
		{0x15,0x2b,1,{0X00}},
		{0x15,0x2c,1,{0X00}},
		{0x15,0x2d,1,{0X00}},
		{0x15,0x2e,1,{0X00}},
		{0x15,0x2f,1,{0X00}},
		{0x15,0x30,1,{0X00}},
		{0x15,0x31,1,{0X00}},
		{0x15,0x32,1,{0X00}},
		{0x15,0x33,1,{0X00}},
		{0x15,0x34,1,{0X43}},

		{0x15,0x36,1,{0X03}},
		{0x15,0x37,1,{0X00}},
		{0x15,0x38,1,{0X00}},
		{0x15,0x39,1,{0X00}},
		{0x15,0x3a,1,{0X00}},
		{0x15,0x3b,1,{0X00}},
		{0x15,0x3c,1,{0X00}},
		{0x15,0x3d,1,{0X00}},
		{0x15,0x3e,1,{0X00}},
		{0x15,0x3f,1,{0X00}},
		{0x15,0x40,1,{0X00}},
		{0x15,0x41,1,{0X00}},
		{0x15,0x42,1,{0X00}},
		{0x15,0x43,1,{0X00}},
		{0x15,0x44,1,{0X00}},
		{0x15,0x50,1,{0X01}},
		{0x15,0x51,1,{0X23}},
		{0x15,0x52,1,{0X45}},
		{0x15,0x53,1,{0X67}},
		{0x15,0x54,1,{0X89}},
		{0x15,0x55,1,{0Xab}},
		{0x15,0x56,1,{0X01}},
		{0x15,0x57,1,{0X23}},
		{0x15,0x58,1,{0X45}},
		{0x15,0x59,1,{0X67}},
		{0x15,0x5a,1,{0X89}},
		{0x15,0x5b,1,{0Xab}},
		{0x15,0x5c,1,{0Xcd}},
		{0x15,0x5d,1,{0Xef}},
		{0x15,0x5e,1,{0X11}},
		{0x15,0x5f,1,{0X02}},
		{0x15,0x60,1,{0X00}},
		{0x15,0x61,1,{0X0C}},
		{0x15,0x62,1,{0X0D}},
		{0x15,0x63,1,{0X0E}},
		{0x15,0x64,1,{0X0F}},
		{0x15,0x65,1,{0X02}},
		{0x15,0x66,1,{0X02}},
		{0x15,0x67,1,{0X02}},
		{0x15,0x68,1,{0X02}},
		{0x15,0x69,1,{0X02}},
		{0x15,0x6a,1,{0X02}},
		{0x15,0x6b,1,{0X02}},
		{0x15,0x6c,1,{0X02}},
		{0x15,0x6d,1,{0X02}},
		{0x15,0x6e,1,{0X05}},
		{0x15,0x6f,1,{0X05}},
		{0x15,0x70,1,{0X05}},
		{0x15,0x71,1,{0X05}},
		{0x15,0x72,1,{0X01}},
		{0x15,0x73,1,{0X06}},
		{0x15,0x74,1,{0X07}},
		{0x15,0x75,1,{0X12}},
		{0x15,0x76,1,{0X00}},
		{0x15,0x77,1,{0X0C}},
		{0x15,0x78,1,{0X0D}},
		{0x15,0x79,1,{0X0E}},
		{0x15,0x7a,1,{0X0F}},
		{0x15,0x7b,1,{0X02}},
		{0x15,0x7c,1,{0X02}},
		{0x15,0x7d,1,{0X02}},
		{0x15,0x7e,1,{0X02}},
		{0x15,0x7f,1,{0X02}},
		{0x15,0x80,1,{0X02}},
		{0x15,0x81,1,{0X02}},
		{0x15,0x82,1,{0X02}},
		{0x15,0x83,1,{0X02}},
		{0x15,0x84,1,{0X05}},
		{0x15,0x85,1,{0X05}},
		{0x15,0x86,1,{0X05}},
		{0x15,0x87,1,{0X05}},
		{0x15,0x88,1,{0X01}},
		{0x15,0x89,1,{0X06}},
		{0x15,0x8A,1,{0X07}},
		{0x39,0xFF,3,{0x98,0x81,0x04}},
		{0x15,0x00,1,{0X00}},
		{0x15,0x6C,1,{0X15}},
		{0x15,0x6E,1,{0X1A}},
		{0x15,0x6F,1,{0X25}},
		{0x15,0x3A,1,{0XA4}},
		{0x15,0x8D,1,{0X20}},
		{0x15,0x87,1,{0XBA}},
		{0x15,0x26,1,{0X76}},
		{0x15,0xB2,1,{0XD1}},
		{0x39,0xFF,3,{0x98,0x81,0x01}},
		{0x15,0x22,1,{0X0A}},
		//{0x15,0x53,1,{0X72}},
		//{0x15,0x55,1,{0X77}},
		{0x15,0x50,1,{0XA6}},
		{0x15,0x51,1,{0XA6}},
		{0x15,0x31,1,{0X00}},
		{0x15,0x60,1,{0X20}},
		{0x15,0xA0,1,{0X08}},
		{0x15,0xA1,1,{0X10}},
		{0x15,0xA2,1,{0X1F}},
		{0x15,0xA3,1,{0X13}},
		{0x15,0xA4,1,{0X12}},
		{0x15,0xA5,1,{0X25}},
		{0x15,0xA6,1,{0X18}},
		{0x15,0xA7,1,{0X1B}},
		{0x15,0xA8,1,{0X67}},
		{0x15,0xA9,1,{0X1D}},
		{0x15,0xAA,1,{0X2A}},
		{0x15,0xAB,1,{0X54}},
		{0x15,0xAC,1,{0X13}},
		{0x15,0xAD,1,{0X0F}},
		{0x15,0xAE,1,{0X49}},
		{0x15,0xAF,1,{0X1D}},
		{0x15,0xB0,1,{0X25}},
		{0x15,0xB1,1,{0X3E}},
		{0x15,0xB2,1,{0X54}},
		{0x15,0xB3,1,{0X39}},
		{0x15,0xC0,1,{0X08}},
		{0x15,0xC1,1,{0X1A}},
		{0x15,0xC2,1,{0X22}},
		{0x15,0xC3,1,{0X0F}},
		{0x15,0xC4,1,{0X11}},
		{0x15,0xC5,1,{0X22}},
		{0x15,0xC6,1,{0X17}},
		{0x15,0xC7,1,{0X1B}},
		{0x15,0xC8,1,{0X66}},
		{0x15,0xC9,1,{0X1D}},
		{0x15,0xCA,1,{0X2A}},
		{0x15,0xCB,1,{0X54}},
		{0x15,0xCC,1,{0X13}},
		{0x15,0xCD,1,{0X0F}},
		{0x15,0xCE,1,{0X4A}},
		{0x15,0xCF,1,{0X1D}},
		{0x15,0xD0,1,{0X2B}},
		{0x15,0xD1,1,{0X3E}},
		{0x15,0xD2,1,{0X52}},
		{0x15,0xD3,1,{0X39}},
		{0x39,0xFF,3,{0x98,0x81,0x00}},
		{0x15,0x20,1,{0X00}},
		{0x15,0x35,1,{0X00}},
	    {0x05, 0x11,1,{0x00}},
	    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},

	    {0x05, 0x29,1,{0x00}},
	    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 20, {}},
	   
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
        params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
        #endif

        // DSI
        /* Command mode setting */
        //1 Three lane or Four lane
        params->dsi.LANE_NUM                = LCM_THREE_LANE;
        //The following defined the fomat for data coming from LCD engine.
        params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

        // Video mode setting
        params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                =8;//6;
    params->dsi.vertical_backporch                   =6; //6;
    params->dsi.vertical_frontporch                   =4;//6;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;


    params->dsi.horizontal_sync_active                = 16;//16;///////////////20 20 4  20  14  6
    params->dsi.horizontal_backporch                = 24;//15;
    params->dsi.horizontal_frontporch                = 28;//6;
    params->dsi.horizontal_active_pixel                = FRAME_WIDTH;

        //params->dsi.LPX=8;

        // Bit rate calculation
        //1 Every lane speed
        //params->dsi.pll_select=1;
        //params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
        params->dsi.PLL_CLOCK=215; //234;
        
        params->dsi.pll_div1=0;        // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
        params->dsi.pll_div2=0;        // div2=0,1,2,3;div1_real=1,2,4,4
#if (LCM_DSI_CMD_MODE)
        params->dsi.fbk_div =7;
#else
        params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#endif
        //params->dsi.compatibility_for_nvk = 1;        // this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
 		params->dsi.esd_check_enable = 1; 
		params->dsi.customization_esd_check_enable = 1;//1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C; 
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

        dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

}


static LCM_setting_table_V3  lcm_deep_sleep_mode_in_setting[] = {

	//{0x39,0xFF, 3,{0x98,0x81,0x00}}, 
    // Display off sequence
    {0x05, 0x28, 0, {}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

    // Sleep Mode On
    {0x05, 0x10, 0, {}},
    {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};
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
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
	    int lcm_vol = 0;
    lcm_init();

}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);

}
#endif





static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[4]={0,0,0,0};
    char id_high=0;
    char id_low=0;
    int id=0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(200);

    array[0]=0x00043902;
    array[1]=0x0698ffff;
    dsi_set_cmdq(array, 2, 1);

    MDELAY(10);
    array[0] = 0x00083700;
    dsi_set_cmdq(array, 1, 1);

    MDELAY(10);
    read_reg_v2(0xD3, buffer, 4);//    NC 0x00  0x98 0x16

    id_high = buffer[1];
    id_low = buffer[2];
    id = (id_high<<8) | id_low;

    #ifdef BUILD_LK

        printf("ILI9881 uboot %s \n", __func__);
        printf("%s id = 0x%08x \n", __func__, id);

    #else
        printk("ILI9881 kernel %s \n", __func__);
        printk("%s id = 0x%08x \n", __func__, id);

    #endif


    return (LCM_ID_ILI9881 == id)?1:0;

}
static unsigned int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;
//	auxadc_test();
#if 1
#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
    #ifdef BUILD_LK
    printf("[adc_uboot dijing]: get data error\n");
   #else
	 printk("[adc_uboot dijing]: get data error\n");
  #endif
    return 0;

    }
#endif
    lcm_vol = data[0]*1000+data[1]*10;


    #ifdef BUILD_LK
     printf("[adc_uboot dijing]: lcm_vol= %d\n",lcm_vol);
   #else
     printk("[adc_uboot dijing]: lcm_vol= %d\n",lcm_vol);
  #endif
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE )
    {
    return 1;
    }
#endif
    return 0;

}// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
    #ifdef BUILD_LK
        //printf("lcm_esd_check()\n");
    #else
        //printk("lcm_esd_check()\n");
    #endif
 #if 0//ndef BUILD_LK
    char  buffer[3];
    int   array[4];

    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0x0a, buffer, 1);
    if(buffer[0]==0x9c)
    {
        //#ifdef BUILD_LK
        //printf("%s %d\n FALSE", __func__, __LINE__);
        //#else
        //printk("%s %d\n FALSE", __func__, __LINE__);
        //#endif
        return FALSE;
    }
    else
    {
        //#ifdef BUILD_LK
        //printf("%s %d\n FALSE", __func__, __LINE__);
        //#else
        //printk("%s %d\n FALSE", __func__, __LINE__);
        //#endif
        return TRUE;
    }
 #endif

}

static unsigned int lcm_esd_recover(void)
{

    #ifdef BUILD_LK
        printf("lcm_esd_recover()\n");
    #else
        printk("lcm_esd_recover()\n");
    #endif

    lcm_init();

    return TRUE;
}
// zhoulidong add for eds(end)
LCM_DRIVER ili9881c_hd_dsi_vdo_3lanes_dj_lcm_drv =
{
        .name            = "ili9881c_hd_dsi_vdo_3lanes_dj",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = rgk_lcm_compare_id,
//    .esd_check = lcm_esd_check,
//    .esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};


