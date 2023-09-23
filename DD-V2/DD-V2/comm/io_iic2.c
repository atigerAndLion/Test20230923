/*
 * io_iic2.c
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 
/* Includes ------------------------------------------------------------------------------------------------*/
#include "io_iic2.h"


/*
 * File:   IO_I2C.c
 * Author: AID
 *
 * ??IIC????
 * 
 * Created on 2016?9?22?, ??5:46
 */

/****************************************************************************
FUNCTION		: IIC_Init
DESCRIPTION		: ??IIC???
INPUT			: None
OUTPUT			: None
NOTICE			: ??IO????????,???????????????
DATE			: 2016/09/21
*****************************************************************************/
void IIC2_Init(void)
{	
	IIC2_SDA_OUT();
	IIC2_SCL_OUT();
	IIC2_SDA_L();
	IIC2_SCL_L();
	delay_us(50);//??????IIC??,???IIC,?????
	IIC2_SCL_H();
	IIC2_SDA_H();
}

/****************************************************************************
FUNCTION		: IIC_Start
DESCRIPTION		: ??IIC??IIC????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC2_Start(void)
{
	IIC2_SDA_OUT();//SDA??
	IIC2_SCL_H();	  	  
	IIC2_SDA_H();
	delay_us(10);
 	IIC2_SDA_L();//START:when CLK is high,DATA change form high to low 
	delay_us(10);
	IIC2_SCL_L();//??CLK??,??????
}	  
/****************************************************************************
FUNCTION		: IIC_Stop
DESCRIPTION		: ??IIC??IIC????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC2_Stop(void)
{
	IIC2_SDA_OUT();//SDA??
	IIC2_SCL_L();
	IIC2_SDA_L();//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC2_SCL_H();	
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	IIC2_SDA_H();
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
u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC2_SDA_H();
	IIC2_SDA_IN();      //SDA???????   
	IIC2_SCL_H();	
    delay_us(5);	 
	while(IIC2_READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC2_SCL_L();
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
void IIC2_Ack(void)
{
	IIC2_SCL_L();
	IIC2_SDA_OUT();//SDA??
	IIC2_SDA_L();//
	delay_us(5);
	IIC2_SCL_H();	
	delay_us(5);
	IIC2_SCL_L();
}
/****************************************************************************
FUNCTION		: IIC_NAck
DESCRIPTION		: ??IIC??NACK????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC2_NAck(void)
{
	IIC2_SCL_L();
	IIC2_SDA_OUT();//SDA??
	IIC2_SDA_H();
	delay_us(5);
	IIC2_SCL_H();	
	delay_us(5);
	IIC2_SCL_L();
}					 				     
/****************************************************************************
FUNCTION		: IIC_Send_Byte
DESCRIPTION		: ??IIC??????
INPUT			: None
OUTPUT			: None
NOTICE			: 
DATE			: 2016/09/21
*****************************************************************************/
void IIC2_Send_Byte(u8 txd)
{                        
    u8 t;   
	IIC2_SDA_OUT();//SDA??
    IIC2_SCL_L();
    for(t=0;t<8;t++)
    {   
		__NOP();
		__NOP();
		__NOP();
		__NOP();        
        if((txd&0x80)>>7)
        {
            IIC2_SDA_H();
        }
        else
        {
            IIC2_SDA_L();
        }
        txd<<=1; 	  
		delay_us(5);  
		IIC2_SCL_H();
		delay_us(5); 
		IIC2_SCL_L();
    }
	delay_us(5); 
    IIC2_Wait_Ack();	 
} 	    
/****************************************************************************
FUNCTION		: IIC_Read_Byte
DESCRIPTION		: ??IIC??????
INPUT			: None
OUTPUT			: None
NOTICE			: ??1???ACK?1??ACK,?0??NACK
DATE			: 2016/09/21
*****************************************************************************/
u8 IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC2_SDA_IN();//SDA??
	IIC2_SCL_L();
	delay_us(5);
    for(i=0;i<8;i++ )
	{
		IIC2_SCL_H();
        delay_us(5);
        receive<<=1;
        if(IIC2_READ_SDA())
        {
        	receive++;   
        }
		IIC2_SCL_L();
		delay_us(5); 
    }					 
    if (!ack)
        IIC2_NAck();
    else
        IIC2_Ack(); 
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
