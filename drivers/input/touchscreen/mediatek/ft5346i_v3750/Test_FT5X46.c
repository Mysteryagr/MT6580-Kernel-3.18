/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: Test_FT5X46.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test item for FT5X46\FT5X46i\FT5526\FT3X17\FT5436\FT3X27\FT5526i\FT5416\FT5426\FT5435
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "Test_FT5X46.h"
#include <linux/i2c.h>
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/device.h>

struct kobject *ft_autotest_kobj;
#define FAIL    0
#define SUCCESS 1
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/////////////////////////////////////////////////Reg 
#define DEVIDE_MODE_ADDR	0x00
#define REG_LINE_NUM	0x01
#define REG_TX_NUM	0x02
#define REG_RX_NUM	0x03
#define REG_NORMALIZE_TYPE      0x16
#define REG_RawBuf0 0x36
/*******************************************************************************
* Static variables
*******************************************************************************/

static int m_RawData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_s5250Data[12][36] = 
{
5125,9519,5328,9896,5388,10006,5443,10109,5473,10163,5513,10239,5578,10360,5736,10652,5039,9357,4975,9239,5601,10401,5365,9963,5298,9838,5749,10677,5691,10569,5814,10798,6144,11410,6897,12809, 
7731,14357,6930,12870,6814,12654,6814,12654,6794,12617,6830,12684,6873,12763,7056,13104,6228,11566,6101,11330,6819,12665,6553,12169,6476,12026,6357,11805,6231,11573,6092,11314,5971,11089,5097,9465, 
6653,12355,6829,12682,6829,12683,6877,12771,6900,12814,6938,12886,6974,12952,7151,13280,6326,11748,6204,11522,6952,12912,6662,12372,6584,12227,6448,11976,6326,11748,6180,11478,6061,11255,5150,9564, 
7243,13451,6962,12929,6897,12809,6933,12875,6921,12853,6962,12930,7008,13016,7178,13330,6346,11786,6199,11513,6965,12935,6684,12412,6604,12264,6459,11995,6338,11770,6202,11518,6073,11279,5172,9604, 
6625,12303,6887,12791,6890,12796,6945,12899,6948,12904,7006,13012,7067,13125,7223,13415,6436,11952,6272,11648,7044,13082,6749,12533,6682,12409,6541,12147,6402,11890,6269,11642,6131,11387,5225,9703, 
6940,12888,7027,13051,6992,12984,7018,13034,6997,12994,7039,13073,7100,13186,7251,13467,6475,12025,6304,11707,7067,13124,6785,12601,6698,12440,6572,12206,6422,11926,6290,11681,6156,11432,5259,9767, 
6864,12747,7004,13008,6976,12956,7001,13003,7004,13008,7052,13096,7116,13215,7270,13501,6502,12076,6308,11714,7088,13163,6803,12633,6717,12475,6581,12223,6436,11952,6293,11687,6162,11444,5263,9775, 
7251,13467,7064,13118,7001,13001,7011,13020,7014,13026,7066,13122,7119,13221,7285,13529,6509,12087,6327,11751,7092,13172,6822,12669,6732,12502,6593,12245,6446,11972,6304,11707,6164,11447,5250,9750, 
7437,13811,7101,13187,6993,12987,6998,12996,6996,12992,7027,13051,7071,13133,7227,13421,6415,11913,6252,11612,7027,13051,6756,12546,6676,12398,6545,12155,6396,11878,6250,11608,6109,11345,5189,9637, 
6669,12385,6880,12776,6875,12767,6911,12835,6914,12840,6968,12940,7013,13023,7173,13321,6371,11833,6223,11557,6990,12981,6716,12472,6659,12367,6502,12076,6357,11805,6212,11536,6061,11257,5155,9573, 
7203,13377,6815,12657,6713,12467,6716,12472,6710,12462,6742,12522,6804,12636,6931,12873,6174,11466,6108,11343,6841,12705,6648,12346,6762,12558,6274,11652,6119,11365,5978,11102,5836,10838,4962,9214, 
5148,9560,5346,9928,5399,10027,5463,10145,5486,10188,5537,10283,5581,10365,5726,10634,5055,9389,5618,10434,6500,12071,6842,12706,8511,15805,5159,9581,5029,9339,4891,9083,4751,8823,3980,7391, 
};

#define MAX_LIMIT_VALUE 30
#define MIN_LIMIT_VALUE 30
static int m_iTempRawData[TX_NUM_MAX * RX_NUM_MAX] = {0};
static unsigned char m_ucTempData[TX_NUM_MAX * RX_NUM_MAX*2] = {0};

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/


/*******************************************************************************
* Static function prototypes
*******************************************************************************/
//////////////////////////////////////////////Communication function
static int StartScan(void);
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer);
static unsigned char GetPanelRows(unsigned char *pPanelRows);
static unsigned char GetPanelCols(unsigned char *pPanelCols);
//////////////////////////////////////////////Common function
static unsigned char GetRawData(void);
static unsigned char GetChannelNum(void);
//////////////////////////////////////////////about Test
//////////////////////////////////////////////Others 
static void ShowRawData(void);
static bool ft_test_sysfs = false;
/************************************************************************
* Name: FT5X46_TestItem_EnterFactoryMode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
//#include "test_lib.h"

#define DEVIDE_MODE_ADDR	0x00
extern struct i2c_client *i2c_ftclient_point ;
struct StruScreenSeting g_ScreenSetParam; //屏幕设置参数


void set_max_channel_num(void)
{

	g_ScreenSetParam.iUsedMaxTxNum = 30;
	g_ScreenSetParam.iUsedMaxRxNum = 30;
}


static DEFINE_MUTEX(i2c_rw_access);

//static tinno_ts_data *g_pts = NULL;
/************************************************************************
* Name: ReadReg(Same function name as FT_MultipleTest)
* Brief:  Read Register
* Input: RegAddr
* Output: RegData
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/

int WriteReg(u8 addr, u8 para)
{
	int rc;
	char buf[3];

#if 1
//     rc =i2c_smbus_read_i2c_block_data(i2c_ftclient_point, addr, 1, pdata);

	buf[0] = addr;
	buf[1] = para;
	mutex_lock(&i2c_rw_access);
	rc = i2c_master_send(i2c_ftclient_point, buf, 2);
	mutex_unlock(&i2c_rw_access);
#endif	
	if(rc >= 0)
		return (ERROR_CODE_OK);
	else
		return (ERROR_CODE_COMM_ERROR);
}

int ReadReg(u8 addr, unsigned char *pdata)
{
	int rc;
	unsigned char buf[2];

	buf[0] = addr;               //register address

	mutex_lock(&i2c_rw_access);
	i2c_master_send(i2c_ftclient_point, &buf[0], 1);
	rc = i2c_master_recv(i2c_ftclient_point, &buf[0], 1);
	mutex_unlock(&i2c_rw_access);

	if (rc < 0)
		printk("msg %s i2c read error: %d\n", __func__, rc);
	*pdata = buf[0];

	if(rc >= 0)
		return (ERROR_CODE_OK);
	else
		return (ERROR_CODE_COMM_ERROR);

}
/************************************************************************
* Name: Comm_Base_IIC_IO(Same function name as FT_MultipleTest)
* Brief:  Write/Read Data by IIC
* Input: pWriteBuffer, iBytesToWrite, iBytesToRead
* Output: pReadBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char Comm_Base_IIC_IO(unsigned char *pWriteBuffer, int  iBytesToWrite, unsigned char *pReadBuffer, int iBytesToRead)
{
	int iRet;	
	
	iRet = fts_i2c_global_read(pWriteBuffer, iBytesToWrite, pReadBuffer, iBytesToRead);

	if(iRet >= 0)
		return (ERROR_CODE_OK);
	else
		return (ERROR_CODE_COMM_ERROR);
}
/************************************************************************
* Name: EnterWork(Same function name as FT_MultipleTest)
* Brief:  Enter Work Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char EnterWork(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if(ReCode == ERROR_CODE_OK)
	{
		if(((RunState>>4)&0x07) == 0x00)	//work
		{
			ReCode = ERROR_CODE_OK;
		}
		else
		{
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0);
			if(ReCode == ERROR_CODE_OK)
			{
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if(ReCode == ERROR_CODE_OK)
				{	
					if(((RunState>>4)&0x07) == 0x00)	ReCode = ERROR_CODE_OK;
					else	ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;
	
}
/************************************************************************
* Name: EnterFactory
* Brief:  enter Fcatory Mode
* Input: null
* Output: null
* Return: Comm Code. Code = 0 is OK, else fail.
***********************************************************************/
unsigned char EnterFactory(void)
{
	unsigned char RunState = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
	if(ReCode == ERROR_CODE_OK)
	{
		if(((RunState>>4)&0x07) == 0x04)	//factory
		{
			ReCode = ERROR_CODE_OK;
		}
		else
		{
			ReCode = WriteReg(DEVIDE_MODE_ADDR, 0x40);
			if(ReCode == ERROR_CODE_OK)
			{
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RunState);
				if(ReCode == ERROR_CODE_OK)
				{	
					if(((RunState>>4)&0x07) == 0x04)	ReCode = ERROR_CODE_OK;
					else	ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;
}


unsigned char FT5X46_TestItem_EnterFactoryMode(void)
{	
	unsigned char ReCode = ERROR_CODE_INVALID_PARAM;
	int iRedo = 5;	//如果不成功，重复进入5次
	int i ;
	unsigned char chPattern=0;
    printk("\n\n[Test_FT5X46.c]FT5X46_TestItem_EnterFactoryMode start\n\n");
	msleep(150);
	for(i = 1; i <= iRedo; i++)
	{
		ReCode = EnterFactory();
		if(ERROR_CODE_OK != ReCode)
		{
			if(i < iRedo)
			{
				msleep(50);
				continue;
			}
		}
		else
		{
			break;
		}

	}
	msleep(300);


	if(ReCode != ERROR_CODE_OK)	
	{	
		return ReCode;
	}

	//进工厂模式成功后，就读出通道数
	ReCode = GetChannelNum();

    printk("\n\n[Test_FT5X46.c]FT5X46_TestItem_EnterFactoryMode end\n\n");
	return ReCode;
}
/************************************************************************
* Name: FT5X46_TestItem_RawDataTest
* Brief:  TestItem: RawDataTest. Check if MCAP RawData is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5X46_TestItem_RawDataTest(bool * bTestResult)
{
	unsigned char ReCode = 0;
	bool btmpresult = true;
	int RawDataMin;
	int RawDataMax;
	unsigned char ucFre;
	unsigned char strSwitch = 0;
	unsigned char OriginValue = 0xff;
	int index = 0;
	int iRow, iCol;
	int iValue = 0;
    printk("[Test_FT5X46.c]FT5X46_TestItem_RawDataTest start\n");
	printk("\n\n[Test_FT5X46.c]==============================Test Item: -------- Raw Data  Test \n\n");
	FT5X46_TestItem_EnterFactoryMode();
	ReCode = ReadReg( REG_NORMALIZE_TYPE, &OriginValue );//读取原始值	
	if( ReCode != ERROR_CODE_OK )goto TEST_ERR;	
	{	
		if(OriginValue != 0)//原始值与需要改变的值不同，则写寄存器为需要的值
		{
			ReCode = WriteReg( REG_NORMALIZE_TYPE, 0x00 );	
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
		}

		ReCode =  ReadReg( 0x0A, &ucFre );
		if( ReCode != ERROR_CODE_OK )goto TEST_ERR;

		{
			printk("[Test_FT5X46.c]=========FIR State: OFF\n" );
			ReCode = WriteReg(0xFB, 0);
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
			msleep(100);
			//先前改变了寄存器 需丢三帧数据
			for (index = 0; index < 3; ++index )
			{
				ReCode = GetRawData();
			}

			if( ReCode != ERROR_CODE_OK )  
			{
				printk("[Test_FT5X46.c]Get Rawdata failed, Error Code: 0x%x",  ReCode);
				goto TEST_ERR;
			}
			ShowRawData();
			////////////////////////////////To Determine RawData if in Range or not
			for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
			{
				for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
				{

					RawDataMin = m_s5250Data[iRow][iCol*2];
					RawDataMax = m_s5250Data[iRow][iCol*2+1];
					iValue = m_RawData[iRow][iCol];
					if(iValue < RawDataMin || iValue > RawDataMax)
					{
						btmpresult = false;
						printk("[Test_FT5X46.c]FT5X46_TestItem_RawDataTest  fall\n");
                        printk("[Test_FT5X46.c]m_s5250Data[iRow][iCol*2] is %d,m_s5250Data[iRow][iCol*2+1] is %d\n",m_s5250Data[iRow][iCol*2],m_s5250Data[iRow][iCol*2+1]);
						printk("RawDataMin[Test_FT5X46.c] is %d,RawDataMax is %d\n",RawDataMin,RawDataMax);

						printk("[Test_FT5X46.c]iRow = %d  iCol=%d iValue=%d\n",iRow,iCol,iValue);				

						goto TEST_ERR;

					} else {
						printk("[Test_FT5X46.c]iRow = %d  iCol=%d iValue=%d\n",iRow,iCol,iValue);
                        printk("[Test_FT5X46.c]m_s5250Data[iRow][iCol*2] is %d,m_s5250Data[iRow][iCol*2+1] is %d\n",m_s5250Data[iRow][iCol*2],m_s5250Data[iRow][iCol*2+1]);
						printk("RawDataMin[Test_FT5X46.c] is %d,RawDataMax is %d\n",RawDataMin,RawDataMax);
					}

				}
			}
		}

	}
	ReCode = WriteReg( REG_NORMALIZE_TYPE, OriginValue );//恢复原来寄存器值
	if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
	
	ReCode = WriteReg(DEVIDE_MODE_ADDR, 0);
	if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
	msleep(150);
	if( btmpresult )
	{
		*bTestResult = true;
		printk("[Test_FT5X46.c]//RawData Test is OK!\n");
	}
	else
	{
		* bTestResult = false;
		printk("[Test_FT5X46.c]//RawData Test is NG!\n");
	}
    printk("[Test_FT5X46.c]FT5X46_TestItem_RawDataTest end\n");
	return ReCode;

TEST_ERR:
	WriteReg(DEVIDE_MODE_ADDR, 0);
	* bTestResult = false;
	printk("[Test_FT5X46.c]//RawData Test is NG!\n");
    printk("[Test_FT5X46.c]FT5X46_TestItem_RawDataTest end\n");
	return ReCode;

}


/************************************************************************
* Name: GetPanelRows(Same function name as FT_MultipleTest)
* Brief:  Get row of TP
* Input: none
* Output: pPanelRows
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelRows(unsigned char *pPanelRows)
{
	return ReadReg(REG_TX_NUM, pPanelRows);
}

/************************************************************************
* Name: GetPanelCols(Same function name as FT_MultipleTest)
* Brief:  get column of TP
* Input: none
* Output: pPanelCols
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelCols(unsigned char *pPanelCols)
{
	return ReadReg(REG_RX_NUM, pPanelCols);
}
/************************************************************************
* Name: StartScan(Same function name as FT_MultipleTest)
* Brief:  Scan TP, do it before read Raw Data
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int StartScan(void)
{
	unsigned char RegVal = 0;
	unsigned char times = 0;
	const unsigned char MaxTimes = 20;	//最长等待160ms
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
	if(ReCode == ERROR_CODE_OK)
	{
		RegVal |= 0x80;		//最高位置1，启动扫描
		ReCode = WriteReg(DEVIDE_MODE_ADDR, RegVal);
		if(ReCode == ERROR_CODE_OK)
		{
			while(times++ < MaxTimes)		//等待扫描完成
			{
				msleep(8);	//8ms
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
				if(ReCode == ERROR_CODE_OK)
				{
					if((RegVal>>7) == 0)	break;
				}
				else
				{
					break;
				}
			}
			if(times < MaxTimes)	ReCode = ERROR_CODE_OK;
			else ReCode = ERROR_CODE_COMM_ERROR;
		}
	}
	return ReCode;

}	
/************************************************************************
* Name: ReadRawData(Same function name as FT_MultipleTest)
* Brief:  read Raw Data
* Input: Freq(No longer used, reserved), LineNum, ByteNum
* Output: pRevBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode=ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3];
	int i, iReadNum;
	unsigned short BytesNumInTestMode1=0;
	iReadNum=ByteNum/342;

	if(0 != (ByteNum%342)) iReadNum++;

	if(ByteNum <= 342)
	{
		BytesNumInTestMode1 = ByteNum;		
	}
	else
	{
		BytesNumInTestMode1 = 342;
	}
	ReCode = WriteReg(REG_LINE_NUM, LineNum);//Set row addr;

	//***********************************************************Read raw data		
	I2C_wBuffer[0] = REG_RawBuf0;	//set begin address
	if(ReCode == ERROR_CODE_OK)
	{
		msleep(10);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, m_ucTempData, BytesNumInTestMode1);
	}

	for(i=1; i<iReadNum; i++)
	{
		if(ReCode != ERROR_CODE_OK) break;

		if(i==iReadNum-1)//last packet
		{
			msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, m_ucTempData+342*i, ByteNum-342*i);
		}
		else
		{
			msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, m_ucTempData+342*i, 342);	
		}
	}

	if(ReCode == ERROR_CODE_OK)
	{
		for(i=0; i<(ByteNum>>1); i++)
		{
			pRevBuffer[i] = (m_ucTempData[i<<1]<<8)+m_ucTempData[(i<<1)+1];
		}
	}

	return ReCode;

}

/************************************************************************
* Name: GetChannelNum
* Brief:  Get Channel Num(Tx and Rx)
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNum(void)
{
	unsigned char ReCode;
	unsigned char rBuffer[1]; //= new unsigned char;
    printk("[Test_FT5X46.c]GetChannelNum start\n\n");
	set_max_channel_num();//set used TxRx
	ReCode = GetPanelRows(rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iTxNum = rBuffer[0];
        printk("[Test_FT5X46.c]g_ScreenSetParam.iTxNum is %d\n",g_ScreenSetParam.iTxNum);
		if(g_ScreenSetParam.iTxNum > g_ScreenSetParam.iUsedMaxTxNum)
		{
			printk("[Test_FT5X46.c]Failed to get Tx number, Get num = %d, UsedMaxNum = %d\n",
				g_ScreenSetParam.iTxNum, g_ScreenSetParam.iUsedMaxTxNum);
			return ERROR_CODE_INVALID_PARAM;
		}
	}
	else
	{
		printk("[Test_FT5X46.c]Failed to get Tx number\n");
	}

	///////////////m_strCurrentTestMsg = "Get Rx Num...";

	ReCode = GetPanelCols(rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iRxNum = rBuffer[0];
        printk("[Test_FT5X46.c]g_ScreenSetParam.iRxNum is %d\n",g_ScreenSetParam.iRxNum);
		if(g_ScreenSetParam.iRxNum > g_ScreenSetParam.iUsedMaxRxNum)
		{
			printk("[Test_FT5X46.c]Failed to get Rx number, Get num = %d, UsedMaxNum = %d\n",
				g_ScreenSetParam.iRxNum, g_ScreenSetParam.iUsedMaxRxNum);
			return ERROR_CODE_INVALID_PARAM;
		}		
	}
	else
	{
		printk("[Test_FT5X46.c]Failed to get Rx number\n");
	}

    printk("[Test_FT5X46.c]GetChannelNum end\n\n");
	return ReCode;

}
/************************************************************************
* Name: GetRawData
* Brief:  Get Raw Data of MCAP
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetRawData(void)
{
	unsigned char ReCode = ERROR_CODE_OK;
	int iRow = 0;
	int iCol = 0;		

	//--------------------------------------------Enter Factory Mode
	ReCode = EnterFactory();	
	if( ERROR_CODE_OK != ReCode ) 
	{
		printk("Failed to Enter Factory Mode...\n");
		return ReCode;
	}
	//--------------------------------------------Check Num of Channel 
	if(0 == (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)) 
	{
		ReCode = GetChannelNum();
		if( ERROR_CODE_OK != ReCode ) 
		{
			printk("Error Channel Num...\n");
			return ERROR_CODE_INVALID_PARAM;
		}
	}
	ReCode = StartScan();
	if(ERROR_CODE_OK != ReCode) 
	{
		printk("Failed to Scan ...\n");
		return ReCode;
	}  
	//--------------------------------------------Read RawData, Only MCAP
	memset(m_RawData, 0, sizeof(m_RawData));
	ReCode = ReadRawData( 1, 0xAA, ( g_ScreenSetParam.iTxNum * g_ScreenSetParam.iRxNum )*2, m_iTempRawData );
	for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; iRow++)
	{
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			m_RawData[iRow][iCol] = m_iTempRawData[iRow*g_ScreenSetParam.iRxNum + iCol];
		}
	}
	return ReCode;
}
/************************************************************************
* Name: ShowRawData
* Brief:  Show RawData
* Input: none
* Output: none
* Return: none.
***********************************************************************/
static void ShowRawData(void)
{
	int iRow, iCol;
	//----------------------------------------------------------Show RawData
	for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; iRow++)
	{
		printk("\nTx iRow %2d:  ", iRow+1);
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			printk("%d, ", m_RawData[iRow][iCol]);
		}
	}
}

static ssize_t ft_sysfs_opentest_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t ft_sysfs_opentest_show(struct device *dev,struct device_attribute *attr, char *buf)
{
        //changed by caozhengguang
        s32 index;
	s32 ret=FAIL;
	ssize_t len = 0;
   	 bool bTempResult = 1;
	 
	ret =  FT5X46_TestItem_RawDataTest(&bTempResult);

	if(ERROR_CODE_OK != ret || (!bTempResult))
	{
		len = sprintf(buf, "FOCALTECH FAIL \n");
	}
	else
		len = sprintf(buf, "FOCALTECH PASS \n");

 	return len;
}

static DEVICE_ATTR(opentest, S_IRUGO|S_IWUSR, ft_sysfs_opentest_show, ft_sysfs_opentest_store);


//static DEVICE_ATTR(shorttest, S_IRUGO|S_IWUSR, gtp_sysfs_shorttest_show, gtp_sysfs_shorttest_store);
/*******************************************************
Description:
    Goodix debug sysfs init function.

Parameter:
    none.
    
return:
    Executive outcomes. 0---succeed.
*******************************************************/
s32 ft_test_sysfs_init(void)
{
    s32 ret ;
    
    ft_autotest_kobj = kobject_create_and_add("gtp_test", NULL) ;
    if (ft_autotest_kobj == NULL)
    {
        return -ENOMEM;
    }
    ret = sysfs_create_file(ft_autotest_kobj, &dev_attr_opentest.attr);
    if (ret)
    {
        return ret;
    }
    ft_test_sysfs=true;
    return 0 ;
}

void ft_test_sysfs_deinit(void)
{
    if(ft_test_sysfs)
    {
 	sysfs_remove_file(ft_autotest_kobj, &dev_attr_opentest.attr);
 	kobject_del(ft_autotest_kobj);
    }
}


