/*
 * i2c_sh367309.c
 *
 *  Created on: 2017-09-12
 *      Author: AID
 */

/* Includes ------------------------------------------------------------------------------------------------*/
#include "i2c_SH367309.h"
#include "io_iic.h"
#include "main.h"
#include <string.h>

#define TRY_TIMES   10
uint8_t  MTPBuffer[26];			//for storage MTP register values

uint16_t afe_com_count = 0;
uint16_t afe_com_first = 0;
uint16_t afe_com_second = 0;
uint16_t afe_com_faild = 0;

u32 REF_RES_VAL;// =	(680 + 5*MTPBuffer[25]);				////resistance = (680 + 5*TR[6~0])*10,  The unit is 10 ohms
//#define REF_RES_VAL	1000				////resistance 10K=1000*10,  The unit is 10 ohms
#define NTC_T_NB  166
const u32 NTC103AT[NTC_T_NB]=		   //this table  the temp corresponding to the (resistance value/10) of 103AT3435
{
	//89710, 85233, 81008, 77019, 73252, //-25~-24
	//69693, 66329, 63148, 60142, 57293, //-20~-16
	//54599, 52049, 49633, 47343, 45174, //-15~-11
	//43117, 41166, 39370, 37558, 35891, //-10~-6
	//34307, 32802, 37370, 30014, 28772, //-5~-1
	//27493, 36324, 25211, 24152, 23144, //0~4
	//22183, 21268, 20395, 19564, 18771, //5~9
	//18015, 17293, 16604, 15947, 15319, //10~14
	//14720, 14147, 13600, 13077, 12577, //15~19
	//12098, 11641, 11203, 10784, 10383, //20~24
	//10000, 9632, 9280, 8942, 8619, //25~29
	//8309, 8012, 7727, 7454, 7191, //30~34
	//6940, 6698, 6466, 6244, 6030, //35~39
	//5825, 5627, 5400, 5255, 5080, //40~44
	//4911, 4749, 4593, 4443, 4299, //45~49
	//4160, 4026, 3898, 3773, 3654, //50~54
	//3539, 3428, 3321, 3218, 3119, //55~59
	//3023, 2931, 2841, 2755, 2672, //60~64
	//2592, 2515, 2441, 2369, 2299, //65~69
	//2232, 2167, 2104, 2044, 1985, //70~74
	//1928, 1874, 1821, 1769, 1720, //75~79
	//1672, 1626, 1581, 1537, 1495, //80~84
	//1455, 1415, 1377, 1340, 1305, //85~89
	//1270, 1236, 1204, 1172, 1142, //90~94
	//1112, 1083, 1056, 1029, 1002, //95~99
	//977, 952, 928, 905, 883, //100~104
	
	208921,197535,186799,176682,167154,158442,149929,141929,134406,127331,
	120672,114420,108531,102981,97748,92814,88173,83792,79654,75745,
	72051,69002,66080,63280,60596,58024,54920,52016,49297,46750,
	44362,42346,40432,38615,36888,35248,33584,32013,30531,29131,
	27808,26606,25464,24378,23345,22362,21451,20580,19748,18952,
	18190,17270,16426,15650,14938,14283,13851,13418,12983,12548,
	12115,11651,11209,10787,10384,10000,9633,9283,8948,8628,
	8322,8024,7738,7463,7199,6944,6704,6472,6251,6037,
	5833,5634,5443,5259,5082,4912,4750,4594,4443,4299,
	4159,4028,3901,3779,3662,3549,3438,3330,3227,3127,
	3031,2939,2850,2763,2680,2600,2523,2448,2376,2306,
	2238,2175,2113,2053,1996,1940,1884,1830,1778,1727,
	1678,1630,1583,1538,1494,1451,1414,1378,1343,1309,
	1276,1243,1210,1179,1148,1118,1090,1062,1035,1009,
	983,959,936,914,892,871,850,830,810,790,
	772,753,736,719,702,686,669,654,638,623,
	608,594,580,567,553,541,
	
};

u8 ucMTPBuffer[26] = 
{
	0x50,0x0C,0x72,0xE4,0x72,
	0xB2,0x6E,0x8C,0xAF,0x4B,
	0x4B,0xFA,0x02,0x20,0x25,
	0x01,0x00,
	70,//充电高温
	65,//充电高温恢复
	-30,//充电低温
	-25,//充电低温恢复
	65,//放电高温
	60,//放电高温恢复
	-25,//放电低温
	-20,//放电低温恢复
	0x46,	
};

SYSINFOR	Info;
AFE_REG  AFE;
u8 CellNum;
s32 sys_current =0;

bool EnCRC = TRUE;//如何设计EnCRC？

u16 ExtVPack;//外部设置的校准电压
u16 ExtCur;//外部设置的校准电流

s16 CadcOffset;
s16 CadcGain;
u16 VPackGain;	
u16 VCellGain[16] = {0};				

u8 cell_min_index;
u8 cell_max_index;

//u8 MTPConfVal = 0;			//for MTP CONF Register

s16 CurBuf[1];				//for storage CADC value, Is used to calculate the mean Within 1s
//u8  CadcTimeCnt;			//for storage CADC value, Is used to calculate the mean Within 1s

s16 Tempe1Offset;
s16 Tempe2Offset;
s16 Tempe3Offset;


//s16  DfilterCur;

const u8  CRC8Table[]={							//120424-1			CRC Table

	0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
	0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
	0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
	0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
	0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
	0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
	0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
	0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
	0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
	0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
	0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
	0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
	0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
	0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
	0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
	0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};
/****************************************************************************
FUNCTION		: InitAFE
DESCRIPTION		: AFE SH367309初始化
INPUT			: None
OUTPUT			: None
UPDATE			:
DATE			: 2019/05/07
*****************************************************************************/
u8 InitAFE(void)
{
	u8 result=1;
	u8 TempCnt=0;
	u8 MTPConfVal_BAK = 0;
	u8 TempVar;
	AFE.CONF.Byte = 0;
	AFE.BFLAG1.Byte = 0;
	AFE.BFLAG2.Byte = 0;
	IIC_Init();
	// AFE初始化
	
	if(MTPRead(MTP_BFLAG2, 1, &AFE.BFLAG2.Byte))							//read AFE reg 43H~45H
	{
		if (AFE.BFLAG2.Bit.RST_FLG == 1)
		{
			AFE.BFLAG2.Bit.RST_FLG = 0;
			MTPWrite(MTP_BFLAG2, 1, &AFE.BFLAG2.Byte);
		}
		else
		{
			MTPRead(MTP_CONF, 1, &AFE.CONF.Byte);
			if (AFE.CONF.Bit.DSGMOS == 1 || AFE.CONF.Bit.CHGMOS == 1)
			{
				sys_flags.flag.sys_id_connect_flag = 1;
			}
		}
	}
	
	while(1)
	{
        EnCRC = TRUE;
        TempVar = 0;
		AFE.CONF.Bit.CADCON = 1;
		MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
		MTPWrite(MTP_BFLAG1, 1, &AFE.BFLAG1.Byte);
		MTPWrite(MTP_BFLAG2, 1, &AFE.BFLAG2.Byte);
        if(MTPRead(MTP_BFLAG2, 1, &TempVar))          
        {
            if((TempVar&0x10) == 0x10) //
            {
                result = 1;
                break;
            }
        }
        else
        {
            EnCRC = FALSE;
            if(MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte))
            {
                if(MTPRead(MTP_CONF, 1, &MTPConfVal_BAK))
                {
                    if(MTPConfVal_BAK == AFE.CONF.Byte)
                    {
                        TempVar = 0;
                        MTPRead(MTP_BFLAG2, 1, &TempVar);        
                        if((TempVar&0x10) == 0x10)
                        {
                            result = 0;
                            break;
                        }
                    }
                }
            }
        }
        delay_us(20);
        if(++TempCnt >= 50)
        {
            result = 0;
            //AFE_ERR = 1;
            //Info.PackStatus = PackStatus;					//Updata AFE_ERR flag and display on the upper computer
            break;
        }
    }
    MTPRead(MTP_SCONF1, 1, &TempVar);// debug ok  13节读取
	CellNum = 	TempVar&0x0F;  //Get System Cell number
	//if(CellNum >= 5) //???????????
	{
		CellNum = 16;
        result = 0;
        // EEPROM ERR
	}
    
	if(!MTPRead(MTP_SCONF1, 26, MTPBuffer))						//Read MTP value  for the PC display
    {
        memset(MTPBuffer, 0, sizeof(MTPBuffer));    
    }
	MTPBuffer[25] &= 0x7F;								//TR[6~0]这里去掉最高位
    
	return result;
}


/****************************************************************************
FUNCTION		: PChg_Control
DESCRIPTION		: AFE SH367309 预充电控制
INPUT			: state 控制状态 1为打开 0为关闭
OUTPUT			: None
UPDATE			:
DATE			: 2019/05/07
*****************************************************************************/
void PChg_Control(u8 state)
{
	if(state)
	{
		AFE.CONF.Bit.PCHMOS = 1;
	}
	else
	{
		AFE.CONF.Bit.PCHMOS = 0;
	}
	MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
}
/****************************************************************************
FUNCTION		: AFERamCheck
DESCRIPTION		: AFE SH367309 定时检测RAM状态
INPUT			: None
OUTPUT			: None
UPDATE			: 如果异常,需要能恢复正常状态
DATE			: 2019/05/07
*****************************************************************************/
//void AFERamCheck(void)
//{
//  u8 result;
//	u8 buf[3];

//	result = MTPRead(MTP_CONF, 3, buf);
//	MTPRead(MTP_CONF, 3, buf);
//	if(result)
//	{
//		if(buf[0] != MTPConfVal)
//		{
//		 	MTPWrite(MTP_CONF, 1, &MTPConfVal);	
//		}	 	
//		if((((u16)buf[1]<<8)|buf[2]) != BalanceChannel)
//		{
//		 	MTPWrite(MTP_BALANCEH, 0x02, (u8 *)&BalanceChannel);
//		}	 	
//	}	
//}

/****************************************************************************
FUNCTION		: ShutDown
DESCRIPTION		: AFE SH367309
INPUT			: None
OUTPUT			: None
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
void ShutDown(void)
{
	SH_SHUTDOWN();// 关闭AFE，此时是彻底关机
}

/****************************************************************************
FUNCTION		: WorkMode
DESCRIPTION		: AFE SH367309
INPUT			: WORK_MODE
OUTPUT			: None
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
void WorkMode(WORK_MODE mode)
{
	//设置AFE工作模式
}


// 51小端16位数据转STM32ARM大端数据，高低字节对调
void UpdateDataFromC51tOStm32U16(uint16_t *u16data)
{
    uint16_t tmp = *u16data;
    uint16_t h1 = 0;
    uint16_t l1 = 0;
    
    l1 = (tmp>>8)&0x00ff;// 高8位转低8位
    h1 = (tmp<<8)&0xff00;// 低8位转高8位
    
    tmp = l1 + h1;
    
    *u16data = tmp;
}


void UpdateDataFromC51tOStm32(void)
{
    u16 i=0;
    UpdateDataFromC51tOStm32U16((uint16_t *)&AFE.Temp1);// 温度
    UpdateDataFromC51tOStm32U16((uint16_t *)&AFE.Temp2);
    UpdateDataFromC51tOStm32U16((uint16_t *)&AFE.Temp3);
    UpdateDataFromC51tOStm32U16((uint16_t *)&AFE.Cur1);// 电流 
    for(i=0;i<CELL_NUM;i++)// 本项目13节电池 1-16
    {
        UpdateDataFromC51tOStm32U16((uint16_t *)&AFE.Cell[i]);// 电压
    }
    UpdateDataFromC51tOStm32U16((uint16_t *)&AFE.Cadc);// 电流寄存器 含充放电标识 最高位1放电0充电
}

/****************************************************************************
FUNCTION		: UpdateDataFromSH367309
DESCRIPTION		: AFE SH367309 数据更新
INPUT			: None
OUTPUT			: None
UPDATE			: 包含寄存器状态，电压，电流，温度
DATE			: 2019/05/07
*****************************************************************************/
void UpdateDataFromSH367309(void)
{
	uint8_t buff[64] = {0} ;
	static uint8_t com_fail_count = 0;
	//获取最新的寄存器值
	//MTPRead(MTP_CONF, 1, &AFE.CONF.Byte);
	////AFERamCheck();//检测是否复位
	//UpdateStates();//更新三个状态寄存器
	//MTPRead(MTP_BALANCEH, sizeof(AFE.BALH), (u8 *)&AFE.BALH);//高8节均衡状态
	//MTPRead(MTP_BALANCEL, sizeof(AFE.BALL), (u8 *)&AFE.BALL);//低8节均衡状态
	//MTPRead(MTP_TEMP1, sizeof(AFE.Temp1), (u8 *)&AFE.Temp1);//更新温度  
	//MTPRead(MTP_TEMP2, sizeof(AFE.Temp2), (u8 *)&AFE.Temp2);
	//MTPRead(MTP_TEMP3, sizeof(AFE.Temp3), (u8 *)&AFE.Temp3);
	//MTPRead(MTP_CUR, sizeof(AFE.Cur1), (u8 *)&AFE.Cur1);//更新电流
	//MTPRead(MTP_CELL1,sizeof(AFE.Cell), (u8 *)&AFE.Cell);//更新电压 //
	//MTPRead(MTP_CADC, sizeof(AFE.Cadc), (u8 *)&AFE.Cadc);//更新电流，这个电流其实跟上一个电流是一个意思，只是用AFE的专用电流16位采集的值，而上面的是13位，得到值之后的电流计算公式也不同
		
	if(I2CRead(MTP_ID,MTP_CONF,51,buff))
	{
		//第一次成功
		AFE.CONF.Byte     = buff[MTP_CONF-MTP_CONF]; //控制
		AFE.BALH         = buff[MTP_BALANCEH-MTP_CONF];//高8节均衡状态
		AFE.BALL         = buff[MTP_BALANCEL-MTP_CONF];//低8节均衡状态
		AFE.BSTATUS1.Byte = buff[MTP_BSTATUS1-MTP_CONF];//更新三个状态寄存器
		AFE.BSTATUS2.Byte = buff[MTP_BSTATUS2-MTP_CONF];
		AFE.BSTATUS3.Byte = buff[MTP_BSTATUS3-MTP_CONF];
		AFE.BFLAG1.Byte = buff[MTP_BFLAG1-MTP_CONF];
		AFE.BFLAG2.Byte = buff[MTP_BFLAG2-MTP_CONF];
		memcpy((u8 *)&AFE.Temp1,buff+MTP_TEMP1-MTP_CONF,sizeof(AFE.Temp1)); //温度
		memcpy((u8 *)&AFE.Temp2,buff+MTP_TEMP2-MTP_CONF,sizeof(AFE.Temp2));
		memcpy((u8 *)&AFE.Temp3,buff+MTP_TEMP3-MTP_CONF,sizeof(AFE.Temp3));	
		memcpy((u8 *)&AFE.Cur1,buff+MTP_CUR-MTP_CONF,sizeof(AFE.Cur1)); //电流	
		memcpy((u8 *)&AFE.Cell,buff+MTP_CELL1-MTP_CONF,sizeof(AFE.Cell));//电压
		memcpy((u8 *)&AFE.Cadc,buff+MTP_CADC-MTP_CONF,sizeof(AFE.Cadc));//更新电流
		
		UpdateDataFromC51tOStm32();// 小端更新为大端数据 温度 电压 电流
		UpdateVoltage();
		UpdateTemperature();
		UpdateCurrent();
		com_fail_count = 0;
		sys_flags.flag.sys_sh367309_comm_err_flag = 0;
		

		
	}
	else if (I2CRead(MTP_ID,MTP_CONF,51,buff))
	{
		//第二次成功
		AFE.CONF.Byte     = buff[MTP_CONF-MTP_CONF]; //控制
		AFE.BALH         = buff[MTP_BALANCEH-MTP_CONF];//高8节均衡状态
		AFE.BALL         = buff[MTP_BALANCEL-MTP_CONF];//低8节均衡状态
		AFE.BSTATUS1.Byte = buff[MTP_BSTATUS1-MTP_CONF];//更新三个状态寄存器
		AFE.BSTATUS2.Byte = buff[MTP_BSTATUS2-MTP_CONF];
		AFE.BSTATUS3.Byte = buff[MTP_BSTATUS3-MTP_CONF];
		AFE.BFLAG2.Byte = buff[MTP_BFLAG2-MTP_CONF];
		memcpy((u8 *)&AFE.Temp1,buff+MTP_TEMP1-MTP_CONF,sizeof(AFE.Temp1)); //温度
		memcpy((u8 *)&AFE.Temp2,buff+MTP_TEMP2-MTP_CONF,sizeof(AFE.Temp2));
		memcpy((u8 *)&AFE.Temp3,buff+MTP_TEMP3-MTP_CONF,sizeof(AFE.Temp3));	
		memcpy((u8 *)&AFE.Cur1,buff+MTP_CUR-MTP_CONF,sizeof(&AFE.Cur1)); //电流	
		memcpy((u8 *)&AFE.Cell,buff+MTP_CELL1-MTP_CONF,sizeof(&AFE.Cell));//电压
		memcpy((u8 *)&AFE.Cadc,buff+MTP_CADC-MTP_CONF,sizeof(AFE.Cadc));//更新电流
		
		UpdateDataFromC51tOStm32();// 小端更新为大端数据 温度 电压 电流
		UpdateVoltage();
		UpdateTemperature();
		UpdateCurrent();
		com_fail_count = 0;
		sys_flags.flag.sys_sh367309_comm_err_flag = 0;
		

	} 
	else
	{
		if (com_fail_count > 25)
		{
			sys_flags.flag.sys_sh367309_comm_err_flag = 1;
		}
		else
		{
			com_fail_count++;
		}
		//失败

	}

}

/****************************************************************************
FUNCTION		: UpdateStates
DESCRIPTION		: AFE SH367309 状态更新
INPUT			: None
OUTPUT			: None
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
void UpdateStates(void)
{
	uint8_t TempBuf[3] = {0};
	if(MTPRead(MTP_BSTATUS1, 3, TempBuf))							//read AFE reg 43H~45H
	{
		AFE.BSTATUS1.Byte = TempBuf[0];
		AFE.BSTATUS2.Byte = TempBuf[1];
		AFE.BSTATUS3.Byte = TempBuf[2];
	}
}

/*******************************************************************************
Function: CaliPackVol()
Description:  Calibration of the total voltage, Update "VPackGain"
Input:
Output: 
Others: 电压校准
*******************************************************************************/
u8 CaliPackVol(void)
{
	u8 i;
	u32 VPackTemp = 0;
    u8 result;
	
	result = MTPRead(MTP_CELL1, CellNum*2, (u8 *)AFE.Cell);
    if(result)
    {
    	for(i=0; i<CellNum; i++)
    	{
    		VPackTemp += AFE.Cell[i];
    	}
    
    	VPackGain = ((u32)CALIPACKVOL*VPackTemp)/ExtVPack;
    	for(i=0; i<CellNum; i++)
    	{
    		VCellGain[i] = VPackGain;
    	}
    }	

    return result;
}

/*******************************************************************************
Function: CaliTemp()
Description:  Calibration temperature, update temperature offset
Input: Addr, ExtTemp1, TempeDiff
Output: 
Others: 温度校准
*******************************************************************************/
u8 CaliTemp(u8 Addr, s16 ExtTemp, s16 *TempeDiff)
{
    u8 result;
	s16 GetTemp;
	s16 Tempe;

	result = MTPRead(Addr, 2, (u8 *)&GetTemp);
    if(result)
    {
    	Tempe = ExtTemp-CalcuTemp(GetTemp);
    	
    	if(((Tempe-*TempeDiff)<150) && ((Tempe-*TempeDiff)>-150))
    	{
    	 	*TempeDiff = Tempe;
    	}
    }
	
    return result;	
}
/*******************************************************************************
Function: CaliCur()
Description:  Calibration current, update current gain"CadcGain"
Input:
Output: 
Others: 电流校准
*******************************************************************************/
u8 CaliCur(void)
{
    u8 result;

	result = MTPRead(MTP_CADC, 2, (u8 *)&AFE.Cadc);
    if(result)
    {
     	CadcGain = (s32)CALICUR*(AFE.Cadc-CadcOffset)/ExtCur;
    }

    return result;
}

/****************************************************************************
FUNCTION		: UpdateVoltage
DESCRIPTION		: AFE SH367309 电压更新
INPUT			: None
OUTPUT			: None
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
void UpdateVoltage(void)
{
	u8 i;
	s16 Vmax = 0x0000;					  
	s16 Vmin = 0x7fff;
	s32 TempPackVol=0, TempCellVol=0;
    
    VPackGain =1;//暂不考虑
	TempPackVol=0;
    TempCellVol=0;
	for(i=0; i<CellNum; i++)
	{
		TempCellVol = (s32)AFE.Cell[i];		
//		TempCellVol = (u32)AFE.Cell[i]*CALIVOL/VCellGain[i];			//Calculate a single battery voltage
		TempPackVol += TempCellVol;						//Calculate the total voltage
		Info.afe.VCell[i] = TempCellVol*5/32;// 转成实际电压
        if(Vmin > Info.afe.VCell[i])
        {
            Vmin = Info.afe.VCell[i];
			cell_min_index = i+1;
        }
        if(Vmax < Info.afe.VCell[i])
        {
            Vmax = Info.afe.VCell[i];
			cell_max_index = i+1;
        }
	}
	Info.vcell_min = Vmin;
	Info.vcell_max = Vmax;
	Info.afe.Voltage = TempPackVol*5/32;// 总压
}
/****************************************************************************
FUNCTION		: UpdateCurrent
DESCRIPTION		: AFE SH367309 电流更新
INPUT			: None
OUTPUT			: None
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
void UpdateCurrent(void)
{
	s16 AveTemp;
	static int16_t offset_value=0;
	static int16_t DSG_1A_value=0;
	static uint16_t k_value = 0;
	int16_t adc_value_temp = 0;
	
	
	CurBuf[0] = AFE.Cadc;
	AveTemp = (s32)CurBuf[0];//((s32)CurBuf[0]+CurBuf[1]+CurBuf[2]+CurBuf[3]) >> 2;			//Calculate the average current four consecutive times
	//Info.afe.CurCadc = ((double)AveTemp*200/26837)*1000/2;//CALICUR*(AveTemp-CadcOffset)/CadcGain;
	//k_value = 4566;
	//offset_value = -23;

	//校正K值
	if (sys_flags.flag.current_calibration == 1 || k_value == 0)
	{
		sys_flags.flag.current_calibration = 0;
		flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_0A_ADC_ADDR, (uint8_t *)&offset_value, 2);
		flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_1A_ADC_ADDR, (uint8_t *)&DSG_1A_value, 2);
		if (DSG_1A_value != (-1) || offset_value != (-1))
		{
			adc_value_temp = DSG_1A_value - offset_value;
			if (adc_value_temp < 0)
			{
				adc_value_temp = 0 - adc_value_temp;
			}
			
			if (adc_value_temp == 0)
			{
				k_value = 0;
			}
			else
			{
				k_value = adc_value_temp*1000000/53675;//53675;//    53675; //理论ADC = 21470*Ω*mA/200 放大1000倍
			}
	
		}
	}
	
	if (k_value == 0)
	{
		Info.afe.CurCadc = ((double)AveTemp*200/21470)*1000/2; //2毫欧的公式   未校准  电流缩小4倍
	}
	else
	{
		if ( 
			( AveTemp > offset_value && AveTemp - offset_value < 3 ) ||
			( offset_value > AveTemp && offset_value - AveTemp < 3 ) 
		)
		{
			Info.afe.CurCadc = 0;
		}
		else
		{
			Info.afe.CurCadc = ((double)(AveTemp - offset_value)*1000000/k_value)*200*2/21470;
		}
	}


	
}
/****************************************************************************
FUNCTION		: UpdateTemperature
DESCRIPTION		: AFE SH367309 温度更新
INPUT			: None
OUTPUT			: None
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
void UpdateTemperature()
{
	s16 TempeData;
	static u8 TempNum =3;// AFE计算的温度传感数量
	Tempe1Offset =0;//暂不使用
	if(TempNum == 1)					//Support one temperature detection
	{
		TempeData = CalcuTemp(AFE.Temp1)+Tempe1Offset;
		Info.afe.Temperature1 = TempeData/10;
		
		Info.min_temp = Info.afe.Temperature1;
		Info.max_temp = Info.afe.Temperature1;
	}
	else if(TempNum ==2)				//Support two temperature detection
	{
		TempeData = CalcuTemp(AFE.Temp1)+Tempe1Offset;
		Info.afe.Temperature1 = TempeData/10;
		
		TempeData = CalcuTemp(AFE.Temp2)+Tempe2Offset;
		Info.afe.Temperature2 = TempeData/10;
		
		if(Info.afe.Temperature1 < Info.afe.Temperature2)
		{
			Info.min_temp = Info.afe.Temperature1;
			Info.max_temp = Info.afe.Temperature2;
		}
		else
		{
			Info.min_temp = Info.afe.Temperature2;
			Info.max_temp = Info.afe.Temperature1;
		}
	}
	else if(TempNum ==3)				//Support three temperature detection
	{
		TempeData = CalcuTemp(AFE.Temp1)+Tempe1Offset;//16715
		Info.afe.Temperature1 = TempeData/10;
		TempeData = CalcuTemp(AFE.Temp2)+Tempe2Offset;
		Info.afe.Temperature2 = TempeData/10;
		TempeData = CalcuTemp(AFE.Temp3)+Tempe3Offset;
		Info.afe.Temperature3 = TempeData/10;
		if(Info.afe.Temperature1 < Info.afe.Temperature2)
		{
			Info.min_temp = Info.afe.Temperature1;
			Info.max_temp = Info.afe.Temperature2;
		}
		else
		{
			Info.min_temp = Info.afe.Temperature2;
			Info.max_temp = Info.afe.Temperature1;
		}
		if(Info.min_temp > Info.afe.Temperature3)
		{
			Info.min_temp = Info.afe.Temperature3;
		}
		if(Info.max_temp < Info.afe.Temperature3)
		{
			Info.max_temp = Info.afe.Temperature3;
		}
		
		if(Info.min_temp > Info.afe.Temperature4)
		{
			Info.min_temp = Info.afe.Temperature4;
		}
		if(Info.max_temp < Info.afe.Temperature4)
		{
			Info.max_temp = Info.afe.Temperature4;
		}
		
		
	}
	
	if(Info.min_temp < -5000 || Info.max_temp > 30000)//-50度，300度，要是超出这个范围，就认为NTC故障了
	{
		//Errs.Bit.bat_ntc_err_flag = 1;
	}
	else
	{
		//Errs.Bit.bat_ntc_err_flag = 0;
	}
}
/****************************************************************************
FUNCTION		: CalcuTemp
DESCRIPTION		: AFE SH367309 温度转换
INPUT			: getdata AD值
OUTPUT			: 温度值，开尔文单位
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
s16 CalcuTemp(s16 getdata)
{
	volatile double dgetdata = getdata;
	volatile    u32 temp_R = 0;
	
	REF_RES_VAL = (6800 + 50*MTPBuffer[25]);//单位欧
//	tempcalcu= lgetdata*REF_RES_VAL; 
	temp_R= (dgetdata/(32768-dgetdata))*REF_RES_VAL; //单位欧
	
	return Get_Ntc_T(temp_R);	
}



/*******************************************************************************
Function: I2CWrite(U8 SlaveID, U16 WrAddr, U8 Length, U8 xdata *WrBuf)
Description:    a whole I2C write communcation without CRC
Input:	NULL 	
Output: NULL
Others:      SH367309 IIC写函数
*******************************************************************************/
u8 I2CWrite(u8 SlaveID, u16 WrAddr, u8 Length, u8 *WrBuf)
{
	u8 i;
	u8 result=0;
    u8 TempBuf[4];
    
    
    TempBuf[0] = SlaveID;
    TempBuf[1] = (u8)WrAddr;
    TempBuf[2] = *WrBuf;
    TempBuf[3] = CRC8Calcu(TempBuf, 3);
	
	if(Length > 0)
	{
		IIC_Start();
		
		if(!I2CSendData(SlaveID, 1))			//Send Slave ID
		{
			goto WrErr;
		}
		
//		if(SlaveID == E2PROM_ID)
//		{
//			if(!I2CSendData(WrAddr>>8, 0))		//Send Write Address(High 8bit) For EEPROM
//			{
//				goto WrErr;
//			}
//		}
		if(I2CSendData(WrAddr, 0))					//Send Write Address(Low 8bit)
		{
			result = 1;
			for(i=0; i<Length; i++)
			{
				if(I2CSendData(*WrBuf, 0))			//Send Write Data
				{
					WrBuf++;
				}
				else
				{
					result = 0;
					break;
				}
			}

            if(EnCRC && (SlaveID==MTP_ID))
            {
                if(!I2CSendData(TempBuf[3], 0))            //write CRC
                {
                    result = 0;
                }
            }
		}
WrErr:
		IIC_Stop();
	}
	
	return result;
}

/*******************************************************************************
Function: I2CWrite()
Description:  write one byte
Input: SlaveID--Slave Address
          RegAddr--register addr
          RegData--register data 
Output:     CY:1--OK
                  0--Error
Others:     SH367309 CRC8
********************************************************************************/
u8 CRC8Calcu(u8 *p, u8 Length)    		   //look-up table calculte CRC 
{    
    u8 crc8 = 0;    
    
	for(; Length > 0; Length--)
	{    
        crc8 = CRC8Table[crc8^*p];    
	    p++;    
    } 
       
    return(crc8);       
}

/*******************************************************************************
Function: I2CRead(U8 SlaveID, U16 RdAddr, U8 Length, U8 xdata *RdBuf)
Description:    a whole I2C read communcation without CRC
Input:	NULL 	
Output: NULL
Others:      SH367309 IIC读函数
*******************************************************************************/
u8 I2CRead(u8 SlaveID, u16 RdAddr, u8 Length, u8 *RdBuf)
{
	u8 i;
	u8 result=0;
	u8 TempBuf[46];
	u8 RdCrc=0;

	TempBuf[0] = SlaveID;
	TempBuf[1] = (u8)RdAddr;
	TempBuf[2] = Length;
	TempBuf[3] = SlaveID | 0x01;
	
	if(Length > 0)
	{
		IIC_Start();
		
		if(!I2CSendData(SlaveID, 1))			//Send Slave ID
		{
			goto RdErr;
		}
		
//		if(SlaveID == E2PROM_ID)
//		{
//			if(!I2CSendData(RdAddr>>8, 0))		//Send Read Address(High 8bit) For EEPROM
//			{
//				goto RdErr;
//			}
//		}
		if(!I2CSendData(RdAddr, 0))				//Send Read Address(Low 8bit)
		{
			goto RdErr;
		}

        if(EnCRC && (SlaveID==MTP_ID))          //CRC
        {
            if(!I2CSendData(Length, 0))
            {
                goto RdErr;
            }
        }                              
		
		I2CReStart();
		
		if(I2CSendData(SlaveID|0x1, 0))			//Send Slave ID
		{
			result = 1;
            if(EnCRC && (SlaveID==MTP_ID))
            {
				for(i=0; i<Length+1; i++)
				{
					if(i == Length)
					{
						RdCrc = I2CGetData(0);      //Get Data
					}
					else
					{
						TempBuf[4+i] = I2CGetData(1);     //Get Data
					}
				}

                if(RdCrc != CRC8Calcu(TempBuf, 4+Length))
                {
                    result = 0;    
                }
                else
                {
                    for(i=0; i<Length; i++)
                    {
                        *RdBuf = TempBuf[4+i];
                        RdBuf++;   
                    }
                }
            }
            else
            {
                for(i=0; i<Length; i++)
                {
        			*RdBuf = I2CGetData(Length-i-1);//Get Data
                    RdBuf++;
                }
            }
		}
	
RdErr:
		IIC_Stop();
	}
	
	return result;
}

/****************************************************************************
FUNCTION		: MTPWrite
DESCRIPTION		: AFE SH367309 寄存器写函数
INPUT			: WrAddr 目标寄存器地址,Length数据长度 *WrBuf 写入的寄存器
OUTPUT			: 0失败,1成功
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
u8 MTPWrite(u8 WrAddr, u8 Length, u8 *WrBuf)
{
    u8 result=0;
    u8 i=0;
    for(i=0; i<Length; i++)
    {
        result = I2CWrite(MTP_ID, WrAddr, 1, WrBuf);
        if(!result)
        {
            result = I2CWrite(MTP_ID, WrAddr, 1, WrBuf);
            if(!result)
            {
                break;
            }
        }
        WrAddr++;
        WrBuf++;
        delay_us(1);    
    }
    
    return  result;
}

/****************************************************************************
FUNCTION		: MTPRead
DESCRIPTION		: AFE SH367309 寄存器读函数
INPUT			: RdAddr 目标寄存器地址,Length数据长度 *RdBuf存放位置
OUTPUT			: 0失败,1成功
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
u8 MTPRead(u8 RdAddr, u8 Length, u8 *RdBuf)
{
  u8 result=1;
	u8 i=0;
	u8 data = 0;
	
	for(i=0; i<Length; i++)
	{
		if(!I2CRead(MTP_ID, RdAddr, 1, &data))
		{
			if(!I2CRead(MTP_ID, RdAddr, 1, &data))
			{
				result = 0;
				//Errs.Bit.AFE_hardware_err_flag = 1;
				break;
			}
		}
		//Errs.Bit.AFE_hardware_err_flag = 0;
		RdAddr++;
		*RdBuf = data;
		RdBuf++;
	}

	return result;
}

/*******************************************************************************
Function:ResetAFE() 
Description:  Reset SH367309 IC, Send Data:0xEA, 0xC0, 0xA5
Input:	 NULL
Output: NULL
Others:  复位AFE,暂未确认函数
*******************************************************************************/
void ResetAFE(void)
{
	u8 WrBuf[2];

	WrBuf[0] = 0xC0;
	WrBuf[1] = 0xA5;
    
    if(EnCRC)   //SP CRC
    {
        if(!I2CWrite(MTP_ID, 0xEA, 1, WrBuf))               //0xEA, 0xC0?A CRC
        {
            I2CWrite(MTP_ID, 0xEA, 1, WrBuf);
        }
    }
    else
    {
        if(!I2CWrite(MTP_ID, 0xEA, 2, WrBuf))               //0xEA, 0xC0?A 0xA5
        {
            I2CWrite(MTP_ID, 0xEA, 2, WrBuf);
        }
    }
}


/*******************************************************************************
Function:UpdataAfeConfig() 
Description:检查SH367309的配置跟MCU是否一致，不一致就要更新SH367309的配置信息
Input:	NULL 	
Output: NULL
Others:
*******************************************************************************/
u8 UpdataAfeConfig(void)
{
    u8 bufferbak[26], mtpbufferbak[26];
    u8 i;
    u16 tempres[8],refres;
	u8 result =0;
    if(MTPRead(0x00, 26, bufferbak))						//读309配置,包括TR
    {
        ucMTPBuffer[MTP_TR] = bufferbak[MTP_TR] & 0x7F;     //先获取TR[6~0]的值
        memcpy(mtpbufferbak, ucMTPBuffer, sizeof(ucMTPBuffer));
        tempres[0] = NTC103AT[((s8)ucMTPBuffer[MTP_OTC])+40];
        tempres[1] = NTC103AT[((s8)ucMTPBuffer[MTP_OTCR])+40];
        tempres[2] = NTC103AT[((s8)ucMTPBuffer[MTP_UTC])+40];
        tempres[3] = NTC103AT[((s8)ucMTPBuffer[MTP_UTCR])+40];
        tempres[4] = NTC103AT[((s8)ucMTPBuffer[MTP_OTD])+40];
        tempres[5] = NTC103AT[((s8)ucMTPBuffer[MTP_OTDR])+40];
        tempres[6] = NTC103AT[((s8)ucMTPBuffer[MTP_UTD])+40];
        tempres[7] = NTC103AT[((s8)ucMTPBuffer[MTP_UTDR])+40];
        refres = 6800 + 50*ucMTPBuffer[MTP_TR];
        mtpbufferbak[MTP_OTC]  = ((float)tempres[0] / ((float)refres + tempres[0])) * 512;
        mtpbufferbak[MTP_OTCR] = ((float)tempres[1] / ((float)refres + tempres[1])) * 512;
        mtpbufferbak[MTP_UTC]  = ((float)tempres[2] / ((float)refres + tempres[2]) - 0.5) * 512;
        mtpbufferbak[MTP_UTCR] = ((float)tempres[3] / ((float)refres + tempres[3]) - 0.5) * 512;
        mtpbufferbak[MTP_OTD]  = ((float)tempres[4] / ((float)refres + tempres[4])) * 512;
        mtpbufferbak[MTP_OTDR] = ((float)tempres[5] / ((float)refres + tempres[5])) * 512;
        mtpbufferbak[MTP_UTD]  = ((float)tempres[6] / ((float)refres + tempres[6]) - 0.5) * 512;
        mtpbufferbak[MTP_UTDR] = ((float)tempres[7] / ((float)refres + tempres[7]) - 0.5) * 512;
        
        for(i=0; i<25; i++)                                 //最后一个TR不做对比
        {
//            RSTSTAT = RSTSTAT;
            if(bufferbak[i] != mtpbufferbak[i])
            {
				CellNum = 16;
                UpdateDataFromSH367309();
                if(Info.afe.Voltage < 18000)		//如果AFE供电低于18V，烧写时VPRO电压可能不足8V，不能烧写
                {
                    result = 1;
                    break;
                }

                VPRO_EN_ON();
				delay_ms(10);
                if(!MTPWriteROM(0x00, 25, mtpbufferbak))     //try 2 次
                {
                    result = 1;
                }
                VPRO_EN_OFF();
                
                if(!result)
                {
                    ResetAFE();                             //Reset IC
					delay_ms(5);
                    InitAFE();
                }
                
                break;
            }
        }
        
		if(!result)
		{
			if(MTPRead(0x00, 26, bufferbak))						//Read MTP value
			{
				for(i=0; i<25; i++)                                 //最后一个TR不做对比
				{
					if(bufferbak[i] != mtpbufferbak[i])
					{
						result = 1;
						break;
					}
				}
			}
			else
			{
				result = 1;				
			}
		}

		if(!result)
		{
			ucMTPBuffer[0] = bufferbak[0];
			CellNum = ucMTPBuffer[0] & 0x0F;              //Get System Cell number							        
			if(CellNum < 5)
			{
				CellNum = 16;
			}
			ucMTPBuffer[MTP_TR] = bufferbak[MTP_TR] & 0x7F;     //获取TR[6~0]的值			
		}
    }
    else
    {
        result = 1;
    }
	
	return result;
}
/****************************************************************************
FUNCTION		: MTPWriteROM
DESCRIPTION		: AFE SH367309 固件更新函数
INPUT			: WrAddr 目标寄存器地址,Length数据长度 *WrBuf 写入的寄存器
OUTPUT			: 0失败,1成功
UPDATE			: 
DATE			: 2019/05/07
*****************************************************************************/
u8 MTPWriteROM(u8 WrAddr, u8 Length, u8 *WrBuf)
{
	u8 result =0;
	u8 i;

	for(i=0; i<Length; i++)
	{
		result = I2CWrite(MTP_ID, WrAddr, 1, WrBuf);
		if(!result)
		{
			delay_ms(40);
			result = I2CWrite(MTP_ID, WrAddr, 1, WrBuf);
			if(!result)
			{
				break;
			}
		}
		WrAddr++;
		WrBuf++;
		delay_ms(40);
	}

  return result;  
}

/**
  * @brief  从NTC电阻值获取NTC温度.
  * @param  NTC电阻值，单位欧
  * @retval NTC温度值，单位0.01摄氏度
  */
s16 Get_Ntc_T(u32 ntc_r)
{
	s16	Temperature = 0;
	u8 TempeMiddle =0;
	u8 li = 0, hi = NTC_T_NB;
	
 
	if(ntc_r >= NTC103AT[0])		            	   //look up table to find the resieter  correspond temp
	{
		Temperature = -4000;			
	}
	else if(ntc_r <= NTC103AT[NTC_T_NB-1])
	{
		Temperature = 12500;
	}
	else
	{
		while((hi - li) > 1)
		{
			TempeMiddle = (hi + li) / 2;
			if(ntc_r > NTC103AT[TempeMiddle])
			{
				hi = TempeMiddle;
			}
			else if(ntc_r < NTC103AT[TempeMiddle])
			{
				li = TempeMiddle;
			}
			else
			{
				break;
			}
		}
		TempeMiddle--;
		
		Temperature = (s16)(TempeMiddle-40)*100+(float)(NTC103AT[TempeMiddle]-ntc_r)/(NTC103AT[TempeMiddle]-NTC103AT[TempeMiddle+1])*100;
	}
	return Temperature;
}

/****************************************************************************
FUNCTION  : Chg_Control
DESCRIPTION  : AFE SH367309 充电控制
INPUT   : state 控制状态 1为打开 0为关闭
OUTPUT   : None
UPDATE   : CONF的寄存器为MCU告诉7309该怎么操作，并非直接影响MOS状态，
       无需保护现场，以MCU的RAM为准
DATE   : 2019/05/07
*****************************************************************************/
void Chg_Control(u8 state)
{
	if(state)
	{
		AFE.CONF.Bit.CHGMOS = 1;
		MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
	}
	else
	{
		AFE.CONF.Bit.CHGMOS = 0;
		MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
	}
}
/****************************************************************************
FUNCTION  : Dsg_Control
DESCRIPTION  : AFE SH367309 放电控制
INPUT   : state 控制状态 1为打开 0为关闭
OUTPUT   : None
UPDATE   : CONF的寄存器为MCU告诉7309该怎么操作，并非直接影响MOS状态，
       无需保护现场，以MCU的RAM为准
DATE   : 2019/05/07
*****************************************************************************/
void Dsg_Control(u8 state)
{
	if(state)
	{
		AFE.CONF.Bit.DSGMOS = 1;
		MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
		
	}
	else
	{
		AFE.CONF.Bit.DSGMOS = 0;
		MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
	}
}
/****************************************************************************
FUNCTION  : MOSFET_Control
DESCRIPTION  : AFE SH367309 充放电控制
INPUT   : state 控制状态 1为打开 0为关闭
OUTPUT   : None
UPDATE   :CONF的寄存器为MCU告诉7309该怎么操作，并非直接影响MOS状态，
       无需保护现场，以MCU的RAM为准
DATE   : 2019/05/07
*****************************************************************************/
void MOSFET_Control(u8 chg_state,u8 dsg_state)
{
	if(chg_state)
	{
		AFE.CONF.Bit.CHGMOS = 1;
	}
	else
	{
		AFE.CONF.Bit.CHGMOS = 0;
	}
	if(dsg_state)
	{
		AFE.CONF.Bit.DSGMOS = 1;
	}
	else
	{
		AFE.CONF.Bit.DSGMOS = 0;
	}
	MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
}


/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
