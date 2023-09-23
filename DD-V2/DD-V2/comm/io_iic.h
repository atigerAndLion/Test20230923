/*
 * io_iic.h
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 
#ifndef __IO_I2C_H
#define __IO_I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------------------------------------*/
#include "main.h"

#define ACK  1
#define NACK 0

#define IIC_SDA_IN()          {gpio_set_pin_direction(SDA, GPIO_DIRECTION_IN);}  
#define IIC_SDA_OUT()         {gpio_set_pin_direction(SDA, GPIO_DIRECTION_OUT);}  
#define IIC_SCL_OUT()         {gpio_set_pin_direction(SCL, GPIO_DIRECTION_OUT);}  
#define IIC_SCL_IN()          {gpio_set_pin_direction(SCL, GPIO_DIRECTION_IN);}  

#define IIC_SCL_H()           {gpio_set_pin_level(SCL,true);}
#define IIC_SCL_L()           {gpio_set_pin_level(SCL,false);}
#define IIC_SDA_H()           {gpio_set_pin_level(SDA,true);}
#define IIC_SDA_L()           {gpio_set_pin_level(SDA,false);}
#define IIC_READ_SDA()        gpio_get_pin_level(SDA)
#define IIC_READ_SCL()        gpio_get_pin_level(SCL)

//api
void IIC_Init(void);                //		 
void IIC_Start(void);				//
void IIC_Stop(void);	  			//
void I2CReStart(void);

u8 I2CChkClkRelease(void);
u8 I2CSendData(u8 Data, u8 ClkFlg);
u8 I2CGetData(u8 AckFlg);
u8 IIC_Wait_Ack(void); 				//WAIT_ACK
void IIC_Ack(void);					//SEND_ACK
void IIC_NAck(void);				//NACK


#endif	/* __IO_I2C_H */

