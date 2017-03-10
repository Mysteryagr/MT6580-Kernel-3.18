/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: Test_FT5X46.h
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test item for FT5X46\FT5X46i\FT5526\FT3X17\FT5436\FT3X27\FT5526i\FT5416\FT5426\FT5435
*
************************************************************************/
#ifndef _TEST_FT5X46_H
#include <linux/kernel.h>
//#include "DetailThreshold.h"
//#include "Test_FT5X46.h"
/*-----------------------------------------------------------
Error Code for Comm
-----------------------------------------------------------*/
#define ERROR_CODE_OK								0x00
#define ERROR_CODE_CHECKSUM_ERROR				0x01
#define ERROR_CODE_INVALID_COMMAND				0x02
#define ERROR_CODE_INVALID_PARAM					0x03
#define ERROR_CODE_IIC_WRITE_ERROR				0x04
#define ERROR_CODE_IIC_READ_ERROR					0x05
#define ERROR_CODE_WRITE_USB_ERROR				0x06
#define ERROR_CODE_WAIT_RESPONSE_TIMEOUT		0x07
#define ERROR_CODE_PACKET_RE_ERROR				0x08
#define ERROR_CODE_NO_DEVICE						0x09
#define ERROR_CODE_WAIT_WRITE_TIMEOUT			0x0a
#define ERROR_CODE_READ_USB_ERROR				0x0b
#define ERROR_CODE_COMM_ERROR					0x0c
#define ERROR_CODE_ALLOCATE_BUFFER_ERROR		0x0d
#define ERROR_CODE_DEVICE_OPENED					0x0e
#define ERROR_CODE_DEVICE_CLOSED					0x0f

/*-----------------------------------------------------------
Test Status
-----------------------------------------------------------*/
#define		RESULT_NULL			0
#define		RESULT_PASS			1
#define		RESULT_NG		    		2
#define		RESULT_TESTING		3
#define		RESULT_TBD				4
#define		RESULT_REPLACE		5
#define		RESULT_CONNECTING		6

#define MAX_IC_TYPE		32

#define _TEST_FT5X46_H
#define TX_NUM_MAX			50
#define RX_NUM_MAX			50
#define MAX_PATH			256
//#define BUFFER_LENGTH		80*80*8
#define BUFFER_LENGTH		512
#define MAX_TEST_ITEM		100

struct StruScreenSeting 
{
	int iSelectedIC;//当前选择的IC
	int iTxNum;
	int iRxNum;
	int    isNormalize;
	int iUsedMaxTxNum;//iTxNum <= iUsedMaxTxNum
	int iUsedMaxRxNum;//iRxNum <= iUsedMaxRxNum
};

struct stTestItem
{
	unsigned char ItemType;//对测试项进行分类	CfgItem, DataTestItem, GraphTestItem,
	unsigned char TestNum;//测试时的序号
	unsigned char TestResult;
	unsigned char ItemCode;
};

void focal_msleep(int ms);
void SysDelay(int ms);


extern int fts_i2c_global_read(char *writebuf,int writelen, char *readbuf, int readlen);
int ReadReg(unsigned char RegAddr, unsigned char *RegData);
int WriteReg(unsigned char RegAddr, unsigned char RegData);
unsigned char Comm_Base_IIC_IO(unsigned char *pWriteBuffer, int  iBytesToWrite, unsigned char *pReadBuffer, int iBytesToRead);

unsigned char EnterWork(void);
unsigned char EnterFactory(void);


unsigned char FT5X46_TestItem_EnterFactoryMode(void);
unsigned char FT5X46_TestItem_RawDataTest(bool * bTestResult);

#endif
