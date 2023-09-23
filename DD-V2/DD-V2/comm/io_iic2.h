/*
 * io_iic2.h
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 
#ifndef __IO_I2C2_H
#define __IO_I2C2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------------------------------------*/
#include "main.h"



#define ACK  1
#define NACK 0

#define IIC2_SDA_IN()          {gpio_set_pin_direction(RH_SDA, GPIO_DIRECTION_IN);}
#define IIC2_SDA_OUT()         {gpio_set_pin_direction(RH_SDA, GPIO_DIRECTION_OUT);}
#define IIC2_SCL_OUT()         {gpio_set_pin_direction(RH_SCL, GPIO_DIRECTION_OUT);}

#define IIC2_SCL_H()           {gpio_set_pin_level(RH_SCL,true);}
#define IIC2_SCL_L()           {gpio_set_pin_level(RH_SCL,false);}
#define IIC2_SDA_H()           {gpio_set_pin_level(RH_SDA,true);}
#define IIC2_SDA_L()           {gpio_set_pin_level(RH_SDA,false);}
#define IIC2_READ_SDA()        gpio_get_pin_level(RH_SDA)
//api
void IIC2_Init(void);                //???		 
void IIC2_Start(void);				//??
void IIC2_Stop(void);	  			//??
void IIC2_Send_Byte(u8 txd);			//??????
u8 IIC2_Read_Byte(unsigned char ack);//??????
u8 IIC2_Wait_Ack(void); 				//WAIT_ACK
void IIC2_Ack(void);					//SEND_ACK
void IIC2_NAck(void);				//NACK

#endif	/* __IO_I2C_H */

