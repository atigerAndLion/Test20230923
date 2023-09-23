/*
 * i2c_cht8305.h
 *
 * Created: 2020/5/15 14:42:07
 *  Author: chenjiawei
 */ 


#ifndef I2C_CHT8305_H_
#define I2C_CHT8305_H_

#include "main.h"
#include "io_iic2.h"
#include <string.h>

#define CHT8305_TEMPERATURE  0x00
#define CHT8305_HUMIDITY     0x01
#define CHT8305_CONFIG       0X02
#define CHT8305_ALERT        0X03
#define CHT8305_MANUFACTURE  0XFE
#define CHT8305_VERSION      0XFF

#define CHT8305_ADDRESS_W    0X80
#define CHT8305_ADDRESS_R    0X81

typedef struct	_CHT8305_VAL_
{
	u16		Temperature_AD;
	u16     Humidity_AD;	
	s16		Temperature;
	s16     Humidity;
}CHT8305_VAL;

extern CHT8305_VAL cht8305_val;

void CHT8305_Init(void);
void I2C2_ReadBytePre(void);
void I2C2_ReadBytes(unsigned char targer, unsigned char *DataBuffer, unsigned int ExpectedByteNumber);
void CHT8305_GetData(void);


#endif /* I2C_CHT8305_H_ */