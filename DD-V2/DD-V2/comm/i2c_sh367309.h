/*
 * i2c_sh367309.h
 *
 *  Created on: 2019-10-28
 *      Author: AID
 */
#ifndef __I2C_SH367309_H
#define __I2C_SH367306_H

//#ifdef __cplusplus
 //extern "C" {
//#endif

/* Includes ------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "io_iic.h"
#include <string.h>

#define SH_SHUTDOWN()         {gpio_set_pin_level(EN_SHIP,true);}//输出高

     //Define Calibration
#define CALIPACKVOL				3800
#define CALIVOL					3800
#define CALICUR					-1000

#define CELL_NUM 16
     

/**
  * @brief  AFE work mode
  */
typedef enum
{
  PROTECT_MODE = 0U,
  SAMPLE_MODE
} WORK_MODE;
	 
#define MTP_ID			0x34


//Define MTP register addr    
#define MTP_SCONF1 0x00
#define MTP_OTC             0x11
#define MTP_OTCR            0x12
#define MTP_UTC             0x13
#define MTP_UTCR            0x14
#define MTP_OTD             0x15
#define MTP_OTDR            0x16
#define MTP_UTD             0x17
#define MTP_UTDR            0x18
#define MTP_TR              0x19


#define MTP_CONF			0x40
#define MTP_BALANCEH		0x41
#define MTP_BALANCEL		0x42
#define MTP_BSTATUS1		0x43
#define MTP_BSTATUS2		0x44
#define MTP_BSTATUS3		0x45
#define MTP_TEMP1			0x46
#define MTP_TEMP2			0x48
#define MTP_TEMP3			0x4A
#define MTP_CUR				0x4C
#define MTP_CELL1			0x4E
#define MTP_CELL2			0x50
#define MTP_CELL3			0x52
#define MTP_CELL4			0x54
#define MTP_CELL5			0x56
#define MTP_CELL6			0x58
#define MTP_CELL7			0x5A
#define MTP_CELL8			0x5C
#define MTP_CELL9			0x5E
#define MTP_CELL10			0x60
#define MTP_CELL11			0x62
#define MTP_CELL12			0x64
#define MTP_CELL13			0x66
#define MTP_CELL14			0x68
#define MTP_CELL15			0x6A
#define MTP_CELL16			0x6C
#define MTP_CADC			0x6E
#define MTP_BFLAG1			0x70
#define MTP_BFLAG2			0x71
#define MTP_RSTSTAT			0x72
     
typedef struct	_AFE_VAL_							
{
	int16_t		VCell[16];
	int32_t		Voltage;
	int32_t		CurCadc;
	int16_t		Temperature1;
	int16_t		Temperature2;
	int16_t		Temperature3;
	int16_t		Temperature4;
}AFE_VAL;

typedef struct	_SYSINFOR_							
{
	AFE_VAL		afe;
	int16_t		g_NTC1_DSGFET_T;//放电MOS管温度
	int16_t		g_NTC2_CHGFET_T;//充电MOS管温度
	int16_t 	max_temp;
	int16_t 	min_temp;
	int16_t 	vcell_min;
	int16_t 	vcell_max;
}SYSINFOR;
extern SYSINFOR	Info; 



//芯片的寄存器结构(这里仅列出使用到的寄存器，不完全)
typedef struct	_AFE_REG_						//data  get form AFE register
{
	union
	{
		struct
		{
			unsigned char IDLE		        :1;
			unsigned char SLEEP			    :1;
			unsigned char ENWDT	            :1;
			unsigned char CADCON	        :1;
			unsigned char CHGMOS			:1;
			unsigned char DSGMOS			:1;
			unsigned char PCHMOS			:1;//
			unsigned char OCRC			    :1;//
		}Bit;
		u8 Byte;
	}CONF;
	
	union
	{
		struct
		{
			unsigned char OV			    :1;// 43h-0
			unsigned char UV			    :1;// 欠压保护
			unsigned char OCD1			    :1;
			unsigned char OCD2			    :1;
			unsigned char OCC	            :1;
			unsigned char SC	            :1;
			unsigned char PF			    :1;
			unsigned char A_WDT		        :1;
		}Bit;
		u8 Byte;
	}BSTATUS1;
	
	union
	{
		struct
		{
			unsigned char UTC			    :1;// 44h-0
			unsigned char OTC			    :1;
			unsigned char UTD			    :1;
			unsigned char OTD			    :1;
			unsigned char Reserved0	        :4;
		}Bit;
		u8 Byte;
	}BSTATUS2;
	
	union
	{
		struct
		{
			unsigned char DSG_FET			:1;
			unsigned char CHG_FET			:1;
			unsigned char PCHG_FET			:1;
			unsigned char L0V	            :1;// 低压禁止充电
			unsigned char EEPR_WR	        :1;
			unsigned char Reserved0			:1;
			unsigned char DSGING		    :1;
			unsigned char CHGING			:1;
		}Bit;
		u8 Byte;
	}BSTATUS3;
	
	u8 BALL;
	u8 BALH;

	int16_t Temp1;
	int16_t Temp2;
	int16_t Temp3;
	int16_t Cur1;
	int16_t Cell[16];
	int16_t Cadc;
	
	union
	{
		struct
		{
			unsigned char OV_FLG			:1;
			unsigned char UV_FLG			:1;
			unsigned char OCD_FLG			:1;
			unsigned char LOAD_FLG	        :1;//
			unsigned char OCC_FLG	        :1;
			unsigned char SC_FLG			:1;
			unsigned char PF_FLG		    :1;
			unsigned char WDT_FLG			:1;
		}Bit;
		u8 Byte;
	}BFLAG1;
	
	union
	{
		struct
		{
			unsigned char UTC_FLG			:1;
			unsigned char OTC_FLG			:1;
			unsigned char UTD_FLG			:1;
			unsigned char OTD_FLG	        :1;//
			unsigned char VADC_FLG	        :1;
			unsigned char CADC_FLG			:1;
			unsigned char WAKE_FLG		    :1;
			unsigned char RST_FLG			:1;
		}Bit;
		u8 Byte;
	}BFLAG2;
	
	
}AFE_REG;


extern AFE_REG  AFE;

extern uint16_t afe_com_count;
extern uint16_t afe_com_first;
extern uint16_t afe_com_second;
extern uint16_t afe_com_faild;

u8 MTPRead(u8 RdAddr, u8 Length, u8 *RdBuf);
u8 MTPWrite(u8 WrAddr, u8 Length, u8 *WrBuf);
u8 I2CRead(u8 SlaveID, u16 RdAddr, u8 Length, u8 *RdBuf);
u8 I2CWrite(u8 SlaveID, u16 WrAddr, u8 Length, u8 *WrBuf);
u8 CRC8Calcu(u8 *p, u8 Length);
u8 InitAFE(void);
void UpdateDataFromSH367309(void);

u8 CaliPackVol(void);
u8 CaliTemp(u8 Addr, s16 ExtTemp, s16 *TempeDiff);
u8 CaliCur(void);

u8 MTPWriteROM(u8 WrAddr, u8 Length, u8 *WrBuf);
u8 UpdataAfeConfig(void);
void UpdateVoltage(void);
void UpdateStates(void);
void UpdateCurrent(void);
void UpdateTemperature(void);
s16 CalcuTemp(s16 getdata);
void AFERamCheck(void);
void ShutDown(void);
void WorkMode(WORK_MODE mode);
s16 Get_Ntc_T(u32 ntc_r);
void PChg_Control(u8 state);

void Chg_Control(u8 state);
void Dsg_Control(u8 state);
void MOSFET_Control(u8 chg_state,u8 dsg_state);


#endif /* __I2C_SH367306_H */

