/*
 * io_iic.c
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 

/* Includes ------------------------------------------------------------------------------------------------*/
#include "io_iic.h"

/****************************************************************************
FUNCTION		: IIC_Init
DESCRIPTION		: ??IIC???
INPUT			: None
OUTPUT			: None
NOTICE			: ??IO????????,???????????????
DATE			: 2016/09/21
*****************************************************************************/
void IIC_Init(void)
{	
	IIC_SDA_OUT();
	IIC_SCL_OUT();
	IIC_SDA_L();
	IIC_SCL_L();
	delay_us(10);//µÈµ½ÑÓ³ÙÎÈ¶¨
	IIC_SCL_H();
	IIC_SDA_H();
}

/****************************************************************************
FUNCTION		: IIC_Start
DESCRIPTION		: ??IIC??IIC????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC_Start(void)
{
    IIC_SCL_OUT();
	IIC_SDA_OUT();//SDA OUT
	IIC_SCL_H();	  	  
	IIC_SDA_H();
	delay_us(10);
 	IIC_SDA_L();//START:when CLK is high,DATA change form high to low 
	delay_us(10);
	IIC_SCL_L();//
}	  
/****************************************************************************
FUNCTION		: IIC_Stop
DESCRIPTION		: ??IIC??IIC????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC_Stop(void)
{
    IIC_SCL_OUT();
	IIC_SDA_OUT();//SDA
	IIC_SCL_L();
	IIC_SDA_L();//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC_SCL_H();	
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	IIC_SDA_H();
	delay_us(10);		
}

void I2CReStart(void)
{
	IIC_SDA_H();
	IIC_SCL_H();	
	delay_us(4);
	delay_us(12);
	IIC_SDA_L();
	delay_us(4);
	delay_us(12);
	IIC_SCL_L();
}


/****************************************************************************
FUNCTION		: IIC_Wait_Ack
DESCRIPTION		: ??IIC??ACK??
INPUT			: None
OUTPUT			: None
NOTICE			: ????1??,0??
DATE			: 2016/09/21
*****************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC_SDA_H();
    IIC_SDA_IN();      //SDA???????   
	IIC_SCL_H();	
    delay_us(5);	 
	while(IIC_READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L();
	return 0;  
} 

/****************************************************************************
FUNCTION		: IIC_Ack
DESCRIPTION		: ??IIC??ACK????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL_L();
	IIC_SDA_OUT();//SDA??
	IIC_SDA_L();//
	delay_us(5);
	IIC_SCL_H();	
	delay_us(5);
	IIC_SCL_L();
}
/****************************************************************************
FUNCTION		: IIC_NAck
DESCRIPTION		: ??IIC??NACK????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_L();
	IIC_SDA_OUT();//SDA??
	IIC_SDA_H();
	delay_us(5);
	IIC_SCL_H();	
	delay_us(5);
	IIC_SCL_L();
}					 				     

//*******************************************************************************

u8 I2CChkClkRelease(void)
{
	u16 TimeoutCnt=1000;						//If Clock is not released within 4ms, is considered overtime
	u8 result=0;

	IIC_SCL_IN();
	while(TimeoutCnt--)
	{
		delay_us(4);
		if(IIC_READ_SCL())
		{
			result = 1;
			break;
		}
	}

	IIC_SCL_H();
	IIC_SCL_OUT();
	
	return result;
}

/*******************************************************************************
Function: I2CSendData()
Description:  I2C subfunction  send a byte
Input:	NULL 	
Output: NULL
Others:      HLX
*******************************************************************************/
u8 I2CSendData(u8 Data, u8 ClkFlg)
{

	u8 i;
	u8 result=0;

//1. After sending the Start signal, there is no need to detect Clock is released, And sending the first bit
	if(Data&0x80)
	{
		IIC_SDA_H();
	}
	else
	{
		IIC_SDA_L();
	}
	if(ClkFlg == 1)
	{
		delay_us(4);
		if(I2CChkClkRelease())
		{
			IIC_SCL_L();
		}
		else
		{
			return result;
		}
	}
	else
	{
		delay_us(4);
		IIC_SCL_H();
		delay_us(4);
		IIC_SCL_L();
	}
	
//2. Send the remaining seven bit
	Data = Data<<1;
	for(i=0; i<7; i++)
	{
		if(Data&0x80)
		{
			IIC_SDA_H();
		}
		else
		{
			IIC_SDA_L();
		}
		Data = Data<<1;
		delay_us(4);
		IIC_SCL_H();
		delay_us(4);
		IIC_SCL_L();
	}
      
	
	IIC_SDA_IN();
	IIC_SDA_H();
 	delay_us(4);
	
 	for(i=0; i<10; i++)
 	{
		if(IIC_READ_SDA() == 0)
		{
			result = 1;
 			break;
		}
 	}
	IIC_SCL_H();

	delay_us(4);
	IIC_SDA_L();
	IIC_SDA_OUT();
	IIC_SCL_L();
	delay_us(4);
	
	return result;
	


}

/*******************************************************************************
Function: I2CGetData()
Description:  I2C subfunction  get a byte
Input:	NULL 	
Output: NULL
Others:      HLX
*******************************************************************************/

u8 I2CGetData(u8 AckFlg)
{
	u8 i, RdData=0;
	
	IIC_SDA_IN()
	delay_us(4);
	for(i=0; i<8; i++)
	{
		IIC_SCL_H();
		delay_us(4);
		if(IIC_READ_SDA())
		{
			RdData |= (1<<(7-i));
		}
		IIC_SCL_L();
		delay_us(4);
	}
    
	IIC_SDA_OUT();
	if(AckFlg != 0)
	{
		IIC_SDA_L();
	}
	else
	{
		IIC_SDA_H();
	}
	IIC_SCL_H();
	delay_us(4);
	IIC_SCL_L();
	delay_us(4);
	return RdData;
}





























