/*
 * io_iic1.h
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 
#ifndef __IO_I2C1_H
#define __IO_I2C1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------------------------------------*/
#include "main.h"



#define ACK  1
#define NACK 0

#define IIC1_SDA_IN()          {gpio_set_pin_direction(RTC_SDA, GPIO_DIRECTION_IN);}
#define IIC1_SDA_OUT()         {gpio_set_pin_direction(RTC_SDA, GPIO_DIRECTION_OUT);}
#define IIC1_SCL_OUT()         {gpio_set_pin_direction(RTC_SCL, GPIO_DIRECTION_OUT);}

#define IIC1_SCL_H()           {gpio_set_pin_level(RTC_SCL,true);}
#define IIC1_SCL_L()           {gpio_set_pin_level(RTC_SCL,false);}
#define IIC1_SDA_H()           {gpio_set_pin_level(RTC_SDA,true);}
#define IIC1_SDA_L()           {gpio_set_pin_level(RTC_SDA,false);}
#define IIC1_READ_SDA()        gpio_get_pin_level(RTC_SDA)
//api
void IIC1_Init(void);                //???		 
void IIC1_Start(void);				//??
void IIC1_Stop(void);	  			//??
void IIC1_Send_Byte(u8 txd);			//??????
u8 IIC1_Read_Byte(unsigned char ack);//??????
u8 IIC1_Wait_Ack(void); 				//WAIT_ACK
void IIC1_Ack(void);					//SEND_ACK
void IIC1_NAck(void);				//NACK

#endif	/* __IO_I2C_H */

