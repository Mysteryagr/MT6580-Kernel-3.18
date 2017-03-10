#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
//	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
//	#include <asm/arch/mt_gpio.h>
#else
//	#include <mach/mt_gpio.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

//#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0xFF,4,{0xAA,0x55,0xA5,0x80}},
	{0x6F,1,{0X0E}},
	{0xF4,1,{0X3A}},
	{0xFF,4,{0xAA,0x55,0xA5,0x00}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
	{0xB0,3,{0x0A,0x0A,0x0A}},
	{0xB6,3,{0x44,0x44,0x44}},
	{0xB1,3,{0x0A,0x0A,0x0A}},
	{0xB7,3,{0x34,0x34,0x34}},
	{0xB2,3,{0x00,0x00,0x00}},
	{0xB8,3,{0x24,0x24,0x24}},
	{0xBF,1,{0X01}},
	{0xB3,3,{0x08,0x08,0x08}},
	{0xB9,3,{0x34,0x34,0x34}},
	{0xB5,3,{0x06,0x06,0x06}},
	{0xC3,1,{0X05}},
	{0xBA,3,{0x14,0x14,0x14}},
	{0xBC,3,{0x00,0x90,0x00}},
	{0xBD,3,{0x00,0x90,0x00}},
	{0xBE,2,{0x00,0x7F}},
	{0xC2,1,{0X03}},
	{0xD1,52,{0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{0xD2,52,{0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{0xD3,52,{0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{0xD4,52,{0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{0xD5,52,{0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{0xD6,52,{0x00,0x00,0x00,0x01,0x00,0x0E,0x00,0x28,0x00,0x4A,0x00,0x91,0x00,0xCF,0x01,0x20,0x01,0x54,0x01,0x99,0x01,0xC4,0x02,0x04,0x02,0x31,0x02,0x32,0x02,0x59,0x02,0x80,0x02,0x97,0x02,0xB1,0x02,0xC1,0x02,0xD4,0x02,0xE1,0x02,0xF2,0x02,0xFF,0x03,0x12,0x03,0x37,0x03,0xFF}},
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB1,2,{0xFC,0x00}},
	{0xB6,1,{0X05}},
	{0xB7,2,{0x70,0x70}},
	{0xB8,4,{0x01,0x05,0x05,0x05}},
	{0xBC,3,{0x02,0x02,0x02}},
	{0xBD,5,{0x01,0x90,0x1C,0x1C,0x00}},
	{0xCB,10,{0x02,0x0B,0xDC,0x01,0x12,0x33,0x33,0x11,0x11,0x0C}},
	{0x35,1,{0X00}},
	{0x11,1,{0X00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0X00}},
	{REGFLAG_DELAY, 10, {}},
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};





static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//MDELAY(10);//soso add or it will fail to send register
       	}
    }
	
}


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


		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 9;//50
		params->dsi.vertical_frontporch					= 6;//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;
		params->dsi.horizontal_backporch				= 32;
		params->dsi.horizontal_frontporch				= 32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


		// Bit rate calculation
	//	params->dsi.pll_div1=30;//32		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	//	params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

	    	params->dsi.PLL_CLOCK				= 160;
		
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		/*
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames		
		*/
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 120ms

     push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
  
	//IsFirstBoot = KAL_TRUE;
}


static void lcm_suspend(void)
{

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(150);//Must > 120ms

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	//lcm_compare_id();

	lcm_init();
	
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x04, buffer, 3);
	id = buffer[1]; //we only need ID
#if defined(BUILD_UBOOT)
	/*The Default Value should be 0x00,0x80,0x00*/
	//printf("\n\n\n\n[soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif
 	printk(" nt35512 lcm_compare_id=%x\n",id);
    return (id == 0x80)?1:0;
   // return 1;
}


LCM_DRIVER nt35512_wvga_dsi_vdo_ctc_lcm_drv = 
{
    .name			= "nt35512_wvga_dsi_vdo_ctc_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
};

