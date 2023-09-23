/*
 * io_iic1.c
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 

/* Includes ------------------------------------------------------------------------------------------------*/
#include "io_iic1.h"


/****************************************************************************
FUNCTION		: IIC_Init
DESCRIPTION		: ??IIC???
INPUT			: None
OUTPUT			: None
NOTICE			: ??IO????????,???????????????
DATE			: 2016/09/21
*****************************************************************************/
void IIC1_Init(void)
{	
	IIC1_SDA_OUT();
	IIC1_SCL_OUT();
	IIC1_SDA_L();
	IIC1_SCL_L();
	delay_ms(50);//??????IIC??,???IIC,?????
	IIC1_SCL_H();
	IIC1_SDA_H();
}

/****************************************************************************
FUNCTION		: IIC_Start
DESCRIPTION		: ??IIC??IIC????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC1_Start(void)
{
	IIC1_SDA_OUT();//SDA??
	IIC1_SCL_H();	  	  
	IIC1_SDA_H();
	delay_us(10);
 	IIC1_SDA_L();//START:when CLK is high,DATA change form high to low 
	delay_us(10);
	IIC1_SCL_L();//??CLK??,??????
}	  
/****************************************************************************
FUNCTION		: IIC_Stop
DESCRIPTION		: ??IIC??IIC????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC1_Stop(void)
{
	IIC1_SDA_OUT();//SDA??
	IIC1_SCL_L();
	IIC1_SDA_L();//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC1_SCL_H();	//添加停止建立时间 4个nop 500nS
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	IIC1_SDA_H();
	delay_us(10);							   	
}
/****************************************************************************
FUNCTION		: IIC_Wait_Ack
DESCRIPTION		: ??IIC??ACK??
INPUT			: None
OUTPUT			: None
NOTICE			: ????1??,0??
DATE			: 2016/09/21
*****************************************************************************/
u8 IIC1_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC1_SDA_H();
  IIC1_SDA_IN();      //SDA???????   
	IIC1_SCL_H();	
    delay_us(5);	 
	while(IIC1_READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC1_Stop();
			return 1;
		}
	}
	IIC1_SCL_L();
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
void IIC1_Ack(void)
{
	IIC1_SCL_L();
	IIC1_SDA_OUT();//SDA??
	IIC1_SDA_L();//
	delay_us(5);
	IIC1_SCL_H();	
	delay_us(5);
	IIC1_SCL_L();
}
/****************************************************************************
FUNCTION		: IIC_NAck
DESCRIPTION		: ??IIC??NACK????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC1_NAck(void)
{
	IIC1_SCL_L();
	IIC1_SDA_OUT();//SDA??
	IIC1_SDA_H();
	delay_us(5);
	IIC1_SCL_H();	
	delay_us(5);
	IIC1_SCL_L();
}					 				     
/****************************************************************************
FUNCTION		: IIC_Send_Byte
DESCRIPTION		: ??IIC??????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC1_Send_Byte(u8 txd)
{                        
    u8 t;   
	IIC1_SDA_OUT();//SDA??
    IIC1_SCL_L();
    for(t=0;t<8;t++)
    {              
		__NOP();
		__NOP();
		__NOP();
		__NOP();
        if((txd&0x80)>>7)
        {
            IIC1_SDA_H();
        }
        else
        {
            IIC1_SDA_L();
        }
        txd<<=1; 	  
		delay_us(5);  
		IIC1_SCL_H();
		delay_us(5); 
		IIC1_SCL_L();
    }
	delay_us(5); 
    IIC1_Wait_Ack();	 
} 	    
/****************************************************************************
FUNCTION		: IIC_Read_Byte
DESCRIPTION		: ??IIC??????
INPUT			: None
OUTPUT			: None
NOTICE			: ??1???ACK?1??ACK,?0??NACK
DATE			: 2016/09/21
*****************************************************************************/
u8 IIC1_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC1_SDA_IN();//SDA??
	IIC1_SCL_L();
	delay_us(5);
    for(i=0;i<8;i++ )
	{
		IIC1_SCL_H();
        delay_us(5);
        receive<<=1;
        if(IIC1_READ_SDA())
        {
        	receive++;   
        }
		IIC1_SCL_L();
		delay_us(5); 
    }					 
    if (!ack)
        IIC1_NAck();
    else
        IIC1_Ack(); 
    return receive;
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
