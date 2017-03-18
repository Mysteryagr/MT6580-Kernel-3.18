
/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#else
#include <linux/string.h>
#endif

#include "lcm_drv.h"
//yufeng
#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
        #include <mt-plat/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                          (720)
#define FRAME_HEIGHT                                         (1280)

#define REGFLAG_DELAY                                         0XFE
#define REGFLAG_END_OF_TABLE                                  0xFFF   // END OF REGISTERS MARKER

#define LCM_ID                                         (0x55)
#define LCM_DSI_CMD_MODE                                    0

#define FALSE 0
#define TRUE 1
//HQ_fujin 131104
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                    lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
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

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;


    params->dsi.vertical_sync_active                = 8;
    params->dsi.vertical_backporch                  = 14;
    params->dsi.vertical_frontporch                 = 20;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 12;
    params->dsi.horizontal_backporch                = 110;
    params->dsi.horizontal_frontporch               = 110;
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

	    //params->dsi.LPX=8;

		// Bit rate calculation
		//1 Every lane speed
        	//params->dsi.pll_select=1;
	    params->dsi.PLL_CLOCK = 299; //this value must be in MTK suggested table
    params->dsi.compatibility_for_nvk = 1;
}

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
        }
    }

}

static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xFF,4,{0xAA,0x55,0x25,0x01}},
    {0x6F,1,{0x16}},
    {0xF7,1,{0x10}},
    {0xFF,4,{0xAA,0x55,0x25,0x00}},
    {0xFE,5,{0x00,0x00,0x00,0x00,0x00}},
    {0xFF,4,{0xAA,0x55,0xA5,0x80}},
    {0x6F,2,{0x11,0x00}},
    {0xF7,2,{0x20,0x00}},
    {0x6F,1,{0x06}},
    {0xF7,1,{0xA0}},
    {0x6F,1,{0x19}},
    {0xF7,1,{0x12}},
    {0x6F,1,{0x10}},
    {0xF5,1,{0x70}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xC8,1,{0x80}},
    {0xB1,2,{0x60,0x21}},
    {0xB6,1,{0x0F}},
    {0xBC,2,{0x00,0x00}},
    {0xBD,5,{0x02,0xB0,0x1E,0x1E,0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
    {0xBC,2,{0x78,0x00}},
    {0xBD,2,{0x78,0x00}},
    {0xCA,1,{0x01}},
    {0xC0,1,{0x0C}},
    {0xBE,1,{0x56}},
    {0xB3,2,{0x38,0x38}},
    {0xB4,2,{0x11,0x11}},
    {0xB6,2,{0x05,0x05}},
    {0xB9,2,{0x35,0x35}},
    {0xBA,2,{0x14,0x14}},
    {0xC4,2,{0x11,0x11}},
    {0xCE,1,{0x66}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
    {0xEE,1,{0x01}},
    {0xB0,16,{0x00,0x00,0x00,0x11,0x00,0x32,0x00,0x52,0x00,0x68,0x00,0x8D,0x00,0xAB,0x00,0xD9}},
    {0xB1,16,{0x00,0xFF,0x01,0x3B,0x01,0x69,0x01,0xB4,0x01,0xF1,0x01,0xF3,0x02,0x2B,0x02,0x6C}},
    {0xB2,16,{0x02,0x90,0x02,0xC7,0x02,0xE9,0x03,0x1E,0x03,0x3C,0x03,0x69,0x03,0x7E,0x03,0x9E}},
    {0xB3,4,{0x03,0xD9,0x03,0xFF}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},
    {0xB0,2,{0x29,0x2A}},
    {0xB1,2,{0x10,0x12}},
    {0xB2,2,{0x14,0x16}},
    {0xB3,2,{0x18,0x1A}},
    {0xB4,2,{0x02,0x04}},
    {0xB5,2,{0x34,0x34}},
    {0xB6,2,{0x34,0x2E}},
    {0xB7,2,{0x2E,0x2E}},
    {0xB8,2,{0x34,0x00}},
    {0xB9,2,{0x34,0x34}},
    {0xBA,2,{0x34,0x34}},
    {0xBB,2,{0x01,0x34}},
    {0xBC,2,{0x2E,0x2E}},
    {0xBD,2,{0x2E,0x34}},
    {0xBE,2,{0x34,0x34}},
    {0xBF,2,{0x05,0x03}},
    {0xC0,2,{0x1B,0x19}},
    {0xC1,2,{0x17,0x15}},
    {0xC2,2,{0x13,0x11}},
    {0xC3,2,{0x2A,0x29}},
    {0xE5,2,{0x2E,0x2E}},
    {0xC4,2,{0x29,0x2A}},
    {0xC5,2,{0x1B,0x19}},
    {0xC6,2,{0x17,0x15}},
    {0xC7,2,{0x13,0x11}},
    {0xC8,2,{0x01,0x05}},
    {0xC9,2,{0x34,0x34}},
    {0xCA,2,{0x34,0x2E}},
    {0xCB,2,{0x2E,0x2E}},
    {0xCC,2,{0x34,0x03}},
    {0xCD,2,{0x34,0x34}},
    {0xCE,2,{0x34,0x34}},
    {0xCF,2,{0x02,0x34}},
    {0xD0,2,{0x2E,0x2E}},
    {0xD1,2,{0x2E,0x34}},
    {0xD2,2,{0x34,0x34}},
    {0xD3,2,{0x04,0x00}},
    {0xD4,2,{0x10,0x12}},
    {0xD5,2,{0x14,0x16}},
    {0xD6,2,{0x18,0x1A}},
    {0xD7,2,{0x2A,0x29}},
    {0xE6,2,{0x2E,0x2E}},
    {0xD8,5,{0x00,0x00,0x00,0x54,0x00}},
    {0xD9,5,{0x00,0x15,0x00,0x00,0x00}},
    {0xE7,1,{0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
    {0xB1,2,{0x00,0x00}},
    {0xB0,2,{0x00,0x00}},
    {0xB2,5,{0x05,0x00,0x00,0x00,0x00}},
    {0xB3,5,{0x05,0x00,0x00,0x00,0x00}},
    {0xB4,5,{0x05,0x00,0x00,0x00,0x00}},
    {0xB5,5,{0x05,0x00,0x17,0x00,0x00}},
    {0xB6,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xB7,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xB8,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xB9,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xBA,5,{0x57,0x00,0x00,0x00,0x00}},
    {0xBB,5,{0x57,0x00,0x00,0x00,0x00}},
    {0xBC,5,{0x75,0x00,0x1A,0x00,0x00}},
    {0xBD,5,{0x53,0x00,0x1A,0x00,0x00}},
    {0xC0,4,{0x00,0x34,0x00,0x00}},
    {0xC1,4,{0x00,0x34,0x00,0x00}},
    {0xC2,4,{0x00,0x34,0x00,0x00}},
    {0xC3,4,{0x00,0x34,0x00,0x00}},
    {0xC4,1,{0x20}},
    {0xC5,1,{0x00}},
    {0xC6,1,{0x00}},
    {0xC7,1,{0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
    {0xED,1,{0x30}},
    {0xB0,2,{0x17,0x06}},
    {0xB8,1,{0x08}},
    {0xBD,5,{0x03,0x07,0x00,0x03,0x00}},
    {0xB1,2,{0x17,0x06}},
    {0xB9,1,{0x00}},
    {0xB2,2,{0x00,0x00}},
    {0xBA,1,{0x00}},
    {0xB3,2,{0x17,0x06}},
    {0xBB,1,{0x0A}},
    {0xB4,2,{0x17,0x06}},
    {0xB5,2,{0x17,0x06}},
    {0xB6,2,{0x14,0x03}},
    {0xB7,2,{0x00,0x00}},
    {0xBC,1,{0x02}},
    {0xE5,1,{0x06}},
    {0xE6,1,{0x06}},
    {0xE7,1,{0x00}},
    {0xE8,1,{0x06}},
    {0xE9,1,{0x06}},
    {0xEA,1,{0x06}},
    {0xEB,1,{0x00}},
    {0xEC,1,{0x00}},
    {0xC0,1,{0x07}},
    {0xC1,1,{0x80}},
    {0xC2,1,{0xA4}},
    {0xC3,1,{0x05}},
    {0xC4,1,{0x00}},
    {0xC5,1,{0x02}},
    {0xC6,1,{0x22}},
    {0xC7,1,{0x03}},
    {0xC8,2,{0x05,0x30}},
    {0xC9,2,{0x01,0x31}},
    {0xCA,2,{0x03,0x21}},
    {0xCB,2,{0x01,0x20}},
    {0xD1,5,{0x00,0x05,0x09,0x07,0x10}},
    {0xD2,5,{0x10,0x05,0x0E,0x03,0x10}},
    {0xD3,5,{0x20,0x00,0x48,0x07,0x10}},
    {0xD4,5,{0x30,0x00,0x43,0x07,0x10}},
    {0xD0,1,{0x00}},
    {0xCC,3,{0x00,0x00,0x3E}},
    {0xCD,3,{0x00,0x00,0x3E}},
    {0xCE,3,{0x00,0x00,0x02}},
    {0xCF,3,{0x00,0x00,0x02}},
    {0x6F,1,{0x11}},
    {0xF3,1,{0x01}},
    {0x35,1,{0x01}},
    {0x11,1,{0x00}},
    {REGFLAG_DELAY,120,{}},
    {0x29,1,{0x00}},
    // Not sure why there's no end of table flag found in LK table
    // Better compare with table in decompiled kernel
    // {REGFLAG_END_OF_TABLE,0x00,{}}
};

static void lcm_init(void)
{
    unsigned int data_array[16];   
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
   // lcm_initialization();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

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
    //MDELAY(1);
   
    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    //MDELAY(1);
   
    data_array[0]= 0x00290508;
    dsi_set_cmdq(data_array, 1, 1);
    //MDELAY(1);
   
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
    //MDELAY(1);

}

static void lcm_suspend(void)
{
    unsigned int data_array[16];
    data_array[0]=0x00280500;
    dsi_set_cmdq(data_array,1,1);
    MDELAY(10);
    data_array[0]=0x00100500;
    dsi_set_cmdq(data_array,1,1);
    MDELAY(100);
}


static void lcm_resume(void)
{
    unsigned int data_array[16];
    data_array[0]=0x00110500;
    dsi_set_cmdq(data_array,1,1);
    MDELAY(100);
    data_array[0]=0x00290500;
    dsi_set_cmdq(data_array,1,1);
    MDELAY(10);
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[3];
    unsigned int array[16]; 
    unsigned int data_array[16];

    SET_RESET_PIN(1);
        MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
   
    SET_RESET_PIN(1);
    MDELAY(120);

    data_array[0] = 0x00063902;
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000108;               
    dsi_set_cmdq(data_array, 3, 1);

    array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
   
    read_reg_v2(0xC5, buffer, 3);
    id = buffer[1]; //we only need ID
#ifdef BUILD_LK
    printf("nt35521 lk %s,buffer=%x,%x,%x,%x\n", __func__,buffer[0],buffer[1],buffer[2],id);
    printf("%s id = 0x%08x \n", __func__, id);
#else
    printk("nt35521 kernel %s \n", __func__);
    printk("%s id = 0x%08x \n", __func__, id);
#endif

   // if(id == LCM_ID_NT35521)
    if(buffer[0]==0x55 && buffer[1]==0x21)
        return 1;
    else
        return 0;

}

static int err_count = 0;

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    unsigned char buffer[8] = {0};
    unsigned int array[4];
    int i =0;

    array[0] = 0x00013700;   
    dsi_set_cmdq(array, 1,1);
    read_reg_v2(0x0A, buffer,8);

    printk( "nt35521_JDI lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/
    {
        printk( "nt35521_JDI lcm_esd_check buffer[0] = %d\n",buffer[0]);
        return TRUE;
    }
    else
    {
        if(buffer[3] != 0x02) //error data type is 0x02
        {
             //return FALSE;
        err_count = 0;
        }
        else
        {
             //if(((buffer[4] != 0) && (buffer[4] != 0x40)) ||  (buffer[5] != 0x80))
        if( (buffer[4] == 0x40) || (buffer[5] == 0x80))
             {
                  err_count = 0;
             }
             else
             {
                  err_count++;
             }            
             if(err_count >=2 )
             {
                 err_count = 0;
                 printk( "nt35521_JDI lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);
                 return TRUE;
             }
        }
        return FALSE;
    }
#endif
   
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();
    return TRUE;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER nt35521_dsi_vdo_yushun_cmi_hd720_lcm_drv =
{
    .name           = "nt35521_dsi_vdo_yushun_cmi_hd720",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
#if 0//defined(LCM_DSI_CMD_MODE)
//    .set_backlight	= lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .update         = lcm_update
#endif
};

