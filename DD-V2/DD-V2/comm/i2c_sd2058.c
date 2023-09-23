/*
 * i2c_sd2058.c
 *
 * Created: 2020/5/15 15:55:32
 *  Author: chenjiawei
 */ 

#include "comm/i2c_sd2058.h"

SD2058_VAL sd2058_val;
struct rtc_time systmtime;

/****************************************************************************
FUNCTION		: I2C1_SendBytes
DESCRIPTION		: 发送一组数据给SD2058
INPUT			: targer 寄存器地址 DataBuffer 数据存放指针  ExpectedByteNumber 发送多少字节的数据
OUTPUT			: None
*****************************************************************************/

void I2C1_SendBytes(unsigned char targer, unsigned char *DataBuffer, unsigned char ByteCount)
{
	uint8_t i;
	IIC1_Start();
	IIC1_Send_Byte(SD2058_ADDRESS_W);
	IIC1_Send_Byte(targer);
	for(i=0;i<ByteCount;i++)
	{
		IIC1_Send_Byte(*DataBuffer);
		DataBuffer++;
	}
	IIC1_Stop();
}

/****************************************************************************
FUNCTION		: I2C1_ReadBytes
DESCRIPTION		: 从SD2058读取一组数据
INPUT			: targer 寄存器地址 DataBuffer 数据存放指针  ExpectedByteNumber 读取多少字节的数据
OUTPUT			: None
*****************************************************************************/

void I2C1_ReadBytes(unsigned char targer, unsigned char *DataBuffer, unsigned int ExpectedByteNumber)
{
	unsigned char i = 0;
	IIC1_Start();
	IIC1_Send_Byte(SD2058_ADDRESS_W);
	IIC1_Send_Byte(targer);
	delay_us(20);
	IIC1_Start();
	IIC1_Send_Byte(SD2058_ADDRESS_R);
	for(i=0;i<ExpectedByteNumber-1;i++)
	{
		*DataBuffer =IIC1_Read_Byte(ACK);
		DataBuffer++;
	}
	*DataBuffer =IIC1_Read_Byte(NACK);
	IIC1_Stop();
}

/****************************************************************************
FUNCTION		: SD2058_Time_Init
DESCRIPTION		: SD2058 初始化函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/

void SD2058_Time_Init(void)
{
	unsigned char temp[10] = {0} ;
	IIC1_Init();
		
	//temp[7] = 0x80 | 0x21; //取消写保护  设置频率中断
	temp[7] = 0x80; //取消写保护  设置频率中断
	I2C1_SendBytes( 0x10 , temp+7 , 1);
	temp[7] = 0x84;
	I2C1_SendBytes( 0x0f , temp+7 , 1);
	
    //temp[7] = 0x0c;
	//I2C1_SendBytes( 0x11 , temp+7 , 1);
	//temp[7] = 0x80 | 0x21; // 设置频率中断
	//I2C1_SendBytes( 0x10 , temp+7 , 1);
	
    temp[7] = 0x00;
    I2C1_SendBytes( 0x11 , temp+7 , 1);
    temp[7] = 0x80 | 0x00; // 设置频率中断
    I2C1_SendBytes( 0x10 , temp+7 , 1);

	temp[7] = 0x00;
	I2C1_SendBytes( 0x0f , temp+7 , 1);
	
	temp[7] = 0x00;
	I2C1_SendBytes( 0x10 , temp+7 , 1);	
				
	I2C1_ReadBytes( 0 , temp , 7);
	if (bcd2bin(temp[6]) < 30 && bcd2bin(temp[6]) >= 20) //大于30 认为时钟未初始化
	{
		return;
	}
	sd2058_val.years = 20;
	sd2058_val.month = 5;
	sd2058_val.date = 29;
	sd2058_val.hours = 2;
	sd2058_val.minutes = 3;
	sd2058_val.seconds = 4;
	sd2058_val.wday = 5;
	SD2058_Rtc_Write_Time(&sd2058_val);
	//暂时无其他初始化
}

/****************************************************************************
FUNCTION		: SD2058_Rtc_Read_Time
DESCRIPTION		: 从 SD2058 读取时间数据
INPUT			: SD2058_VAL 结构体指针
OUTPUT			: None
*****************************************************************************/

void SD2058_Rtc_Read_Time(SD2058_VAL *p)
{
    unsigned char temp[10] = {0} ;
    I2C1_ReadBytes( 0 , temp , 7); //reg address first is 0,buff is temp,length of reg is 7

	p->seconds = bcd2bin(temp[0] & SD2058_SECONDS_MASK);
	p->minutes = bcd2bin(temp[1] & SD2058_MINUTES_MASK);
	p->hours = bcd2bin(temp[2] & SD2058_HOURS_MASK);
	p->wday = bcd2bin(temp[3]);
	p->date = bcd2bin(temp[4]);
	p->month = bcd2bin(temp[5]);
	p->years= bcd2bin(temp[6]);

}

/****************************************************************************
FUNCTION		: SD2058_Rtc_Write_Time
DESCRIPTION		: 写时间数据到 SD2058 
INPUT			: SD2058_VAL 结构体指针
OUTPUT			: None
*****************************************************************************/

void SD2058_Rtc_Write_Time(SD2058_VAL *p)
{
    unsigned char temp[10] ;
	I2C1_ReadBytes( 0 , temp , 7);

    temp[0] = bin2bcd(p->seconds);
    temp[1] = bin2bcd(p->minutes);
    temp[2] = bin2bcd(p->hours);
    temp[3] = bin2bcd(p->wday);
    temp[4] = bin2bcd(p->date);
    temp[5] = bin2bcd(p->month);
    temp[6] = bin2bcd(p->years);
	
	//temp[7] = 0x80 | 0x21; //取消写保护  设置频率中断
	temp[7] = 0x80; //取消写保护  设置频率中断
	I2C1_SendBytes( 0x10 , temp+7 , 1);
	temp[7] = 0x84;
	I2C1_SendBytes( 0x0f , temp+7 , 1);
	
    I2C1_SendBytes( 0 , temp , 7);

	temp[7] = 0x00;
	I2C1_SendBytes( 0x0f , temp+7 , 1);
	
	//temp[7] = 0x21;
	temp[7] = 0x00;
	I2C1_SendBytes( 0x10 , temp+7 , 1);

}


/**
  * @brief  This function is bcd2bin.
  * @param  unsigned char val
  * @retval None
  */
  
unsigned char bcd2bin(unsigned char val)
{
    return (val & 0x0f) + (val >> 4) * 10;
}
 
/**
  * @brief  This function is bin2bcd.
  * @param  unsigned char val
  * @retval None
  */
  
unsigned char bin2bcd(unsigned char val)
{
    return ((val / 10) << 4) + val % 10;
}


//RTC转换为时间戳

#define FEBRUARY		2
#define	STARTOFTIME		1970
#define SECDAY			86400L           /*  一天有多少s */
#define SECYR			(SECDAY * 365)
#define	leapyear(year)		((year) % 4 == 0)
#define	days_in_year(a) 	(leapyear(a) ? 366 : 365)
#define	days_in_month(a) 	(month_days[(a) - 1])

static int month_days[12] = {	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*
 * This only works for the Gregorian calendar - i.e. after 1752 (in the UK)
 */
 /*计算公历*/
void GregorianDay(struct rtc_time * tm)
{
	int leapsToDate;
	int lastYear;
	int day;
	int MonthOffset[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };

	lastYear=tm->tm_year-1;

	/*计算从公元元年到计数的前一年之中一共经历了多少个闰年*/
	leapsToDate = lastYear/4 - lastYear/100 + lastYear/400;      

     /*如若计数的这一年为闰年，且计数的月份在2月之后，则日数加1，否则不加1*/
	if((tm->tm_year%4==0) &&
	   ((tm->tm_year%100!=0) || (tm->tm_year%400==0)) &&
	   (tm->tm_mon>2)) {
		/*
		 * We are past Feb. 29 in a leap year
		 */
		day=1;
	} else {
		day=0;
	}

	day += lastYear*365 + leapsToDate + MonthOffset[tm->tm_mon-1] + tm->tm_mday; /*计算从公元元年元旦到计数日期一共有多少天*/

	tm->tm_wday=day%7;
}

/* Converts Gregorian date to seconds since 1970-01-01 00:00:00.
 * Assumes input in normal date format, i.e. 1980-12-31 23:59:59
 * => year=1980, mon=12, day=31, hour=23, min=59, sec=59.
 *
 * [For the Julian calendar (which was used in Russia before 1917,
 * Britain & colonies before 1752, anywhere else before 1582,
 * and is still in use by some communities) leave out the
 * -year/100+year/400 terms, and add 10.]
 *
 * This algorithm was first published by Gauss (I think).
 *
 * WARNING: this function will overflow on 2106-02-07 06:28:16 on
 * machines were long is 32-bit! (However, as time_t is signed, we
 * will already get problems at other places on 2038-01-19 03:14:08)
 *ADD by fire：本函数在工程中的输入参数为北京时间，
 所以在转换成时间戳时最后要从北京时间转换为标准时间的时间戳
 */
u32 mktimev(struct rtc_time *tm)
{
	if (0 >= (int) (tm->tm_mon -= 2)) {	/* 1..12 -> 11,12,1..10 */
		tm->tm_mon += 12;		/* Puts Feb last since it has leap day */
		tm->tm_year -= 1;
	}

	return (((
		(u32) (tm->tm_year/4 - tm->tm_year/100 + tm->tm_year/400 + 367*tm->tm_mon/12 + tm->tm_mday) +
			tm->tm_year*365 - 719499
	    )*24 + tm->tm_hour /* now have hours */
	  )*60 + tm->tm_min /* now have minutes */
	)*60 + tm->tm_sec;//-8*60*60; /* finally seconds */
	/*Add by fire: -8*60*60 把输入的北京时间转换为标准时间，
	再写入计时器中，确保计时器的数据为标准的UNIX时间戳*/ 
	 
}



void to_tm(u32 tim, struct rtc_time * tm)
{
	register u32    i;
	register long   hms, day;


	day = tim / SECDAY;			/* 有多少天 */
	hms = tim % SECDAY;			/* 今天的时间，单位s */

	/* Hours, minutes, seconds are easy */
	tm->tm_hour = hms / 3600;
	tm->tm_min = (hms % 3600) / 60;
	tm->tm_sec = (hms % 3600) % 60;

	/* Number of years in days */ /*算出当前年份，起始的计数年份为1970年*/
	for (i = STARTOFTIME; day >= days_in_year(i); i++) {
		day -= days_in_year(i);
	}
	tm->tm_year = i;

	/* Number of months in days left */ /*计算当前的月份*/
	if (leapyear(tm->tm_year)) {
		days_in_month(FEBRUARY) = 29;
	}
	for (i = 1; day >= days_in_month(i); i++) {
		day -= days_in_month(i);
	}
	days_in_month(FEBRUARY) = 28;
	tm->tm_mon = i;

	/* Days are what is left over (+1) from all that. *//*计算当前日期*/
	tm->tm_mday = day + 1;

	/*
	 * Determine the day of week
	 */
	GregorianDay(tm);
	
}

void Buff_Time_To_Struct_Time(u8 *buff,struct rtc_time *tm)
{	
	
	tm->tm_sec = buff[0];
	tm->tm_min = buff[1];
	tm->tm_hour = buff[2];
	tm->tm_mday = buff[3];
	tm->tm_mon = buff[4];
	tm->tm_year = buff[5] + 2000;
	tm->tm_wday = buff[6];	
}

void Struct_Time_To_Buff_Time(struct rtc_time *tm,u8 *buff)
{	
	buff[0] = tm->tm_sec;
	buff[1] = tm->tm_min;
	buff[2] = tm->tm_hour;
	buff[3] = tm->tm_mday ;
	buff[4] = tm->tm_mon ;
	buff[5] = tm->tm_year -2000;
	buff[6] = tm->tm_wday;	
}