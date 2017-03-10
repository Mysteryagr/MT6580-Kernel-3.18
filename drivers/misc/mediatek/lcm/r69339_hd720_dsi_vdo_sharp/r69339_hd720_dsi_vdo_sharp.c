#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
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

#define LCM_ID_ILI9881                                    0x9881


#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;
#if 0
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#else
extern int DispTEpin_Enable(void);
extern int DispTEpin_Disable(void);
#define SET_RESET_PIN(v)    \
    if(v)                                           \
        DispTEpin_Enable(); \
    else                                           \
        DispTEpin_Disable();
#endif
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

        params->physical_width  = 68.04; //62.10;
        params->physical_height = 120.96; //110.40;

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

        params->dsi.vertical_sync_active				= 1;
        params->dsi.vertical_backporch					= 11;
        params->dsi.vertical_frontporch					= 13;
        params->dsi.vertical_active_line				= FRAME_HEIGHT; 

        params->dsi.horizontal_sync_active				= 10;
        params->dsi.horizontal_backporch				= 40;
        params->dsi.horizontal_frontporch				= 100;
        params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

        // Bit rate calculation
        //1 Every lane speed
        //params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
        //params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
        //params->dsi.fbk_div =0x12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
        params->dsi.PLL_CLOCK = 208;

        params->dsi.pll_div1=0;        // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
        params->dsi.pll_div2=0;        // div2=0,1,2,3;div1_real=1,2,4,4
#if (LCM_DSI_CMD_MODE)
        params->dsi.fbk_div =7;
#else
        params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#endif
        //params->dsi.compatibility_for_nvk = 1;        // this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
 		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;//1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
}

//extern void hwPowerOnVGP2(void);
static void init_lcm_registers(void)
{
	unsigned int data_array[16];


        printk("%s, KEZX\n", __func__);
        //hwPowerOnVGP2();
        printk("%s, KEZX,line=%d\n", __func__,__LINE__);
        data_array[0] = 0x04B02300; 						 
        dsi_set_cmdq(data_array, 1, 1); 
        //data_array[0] = 0x01D62300; 						 
        //dsi_set_cmdq(data_array, 1, 1); 
        

        data_array[0] = 0x00082902; 
        data_array[1] = 0x001500C2;
        data_array[2] = 0x00000D0C;
        dsi_set_cmdq(data_array, 3, 1); 

        data_array[0] = 0x01D62300; 						 
        dsi_set_cmdq(data_array, 1, 1); 

        data_array[0] = 0x08B42300; 						 
        dsi_set_cmdq(data_array, 1, 1);
        
        data_array[0] = 0x00350500;
        dsi_set_cmdq(data_array, 1, 1);

        data_array[0] = 0x00290500;                          
        dsi_set_cmdq(data_array, 1, 1);

        data_array[0] = 0x00110500;                          
        dsi_set_cmdq(data_array, 1, 1);
}

extern int LcmPowerOnPMIC(void);
static void lcm_init(void)
{
    //SET_RESET_PIN(1);
    //SET_RESET_PIN(0);
    //MDELAY(10);
    //SET_RESET_PIN(1);
    //MDELAY(120);

    //    dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
        //power_on_vgp1();
        //MDELAY(5);
        //power_on_vgp2();
        //MDELAY(10);
        //SET_RESET_PIN(0);
        //MDELAY(20);
        //SET_RESET_PIN(1);
        //MDELAY(50);
        LcmPowerOnPMIC();
        MDELAY(10);
        SET_RESET_PIN(1);
        MDELAY(20);
        SET_RESET_PIN(0);
        MDELAY(50);   //50
        SET_RESET_PIN(1);
        MDELAY(100);

        init_lcm_registers();

}
extern int LcmPowerOffPMIC_V18(void);
extern int LcmPowerOffPMIC_V28(void);
static void lcm_suspend(void)
{
	unsigned int data_array[16];

#ifdef BUILD_LK
	dprintf(INFO, "%s, LK\n", __func__);
#else
	printk("%s, kernel", __func__);
#endif

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(20);   //0729
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(80);    //0729

	data_array[0] = 0x00B02300; 						 
	dsi_set_cmdq(data_array, 1, 1); 

	data_array[0] = 0x01B12300; //0729			 
	dsi_set_cmdq(data_array, 1, 1); 
	LcmPowerOffPMIC_V28();
	MDELAY(10);  //0729
	SET_RESET_PIN(0);
             MDELAY(10);  //0729
             LcmPowerOffPMIC_V18();
}

static void lcm_resume(void)
{
#ifdef BUILD_LK
	dprintf(INFO, "%s, LK\n", __func__);
#else
	printk("%s, kernel zx\n", __func__);
#endif
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




/*
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
*/
static unsigned int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#if 1
#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)

    {
    #ifdef BUILD_LK
    printf("[adc_uboot]: get data error\n");
    #endif
    return 0;

    }
#endif
    lcm_vol = data[0]*1000+data[1]*10;


    #ifdef BUILD_LK
    printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
    #endif

    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE )
    {
    return 1;
    }
#endif
    return 0;

}// zhoulidong add for eds(start)


// zhoulidong add for eds(end)
LCM_DRIVER r69339_hd720_dsi_vdo_sharp_lcm_drv =
{
        .name            = "r69339_hd720_dsi_vdo_sharp",
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


