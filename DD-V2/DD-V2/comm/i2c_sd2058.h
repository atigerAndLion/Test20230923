/*
 * i2c_sd2058.h
 *
 * Created: 2020/5/15 15:55:42
 *  Author: chenjiawei
 */ 


#ifndef I2C_SD2058_H_
#define I2C_SD2058_H_

#include "main.h"
#include "io_iic1.h"
#include <string.h>

#define SD2058_ADDRESS_W    0x64
#define SD2058_ADDRESS_R    0x65

#define SD2058_SECONDS_MASK	0x7F	/* Mask over seconds value */
#define SD2058_MINUTES_MASK	0x7F	/* Mask over minutes value */
#define SD2058_HOURS_MASK	0x3F	/* Mask over hours value */

typedef struct	_SD2058_VAL_
{
	uint8_t		seconds ;
	uint8_t     minutes ;
	uint8_t		hours ;
	uint8_t		date	; 
	uint8_t		month ;
	uint8_t		years ;
	uint8_t     wday ;
}SD2058_VAL;



extern SD2058_VAL sd2058_val;

void I2C1_SendBytes(unsigned char targer, unsigned char *DataBuffer, unsigned char ByteCount);
void I2C1_ReadBytes(unsigned char targer, unsigned char *DataBuffer, unsigned int ExpectedByteNumber);

void SD2058_Time_Init(void);
void SD2058_Rtc_Read_Time(SD2058_VAL *p);
void SD2058_Rtc_Write_Time(SD2058_VAL *p);

unsigned char bcd2bin(unsigned char val);
unsigned char bin2bcd(unsigned char val);

struct rtc_time {
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
};

extern struct rtc_time systmtime;

void GregorianDay(struct rtc_time * tm);
u32 mktimev(struct rtc_time *tm);
void to_tm(u32 tim, struct rtc_time * tm);
void Struct_Time_To_Buff_Time(struct rtc_time *tm,u8 *buff);
void Buff_Time_To_Struct_Time(u8 *buff,struct rtc_time *tm);


#endif /* I2C_SD2058_H_ */