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
DESCRIPTION		: ����һ�����ݸ�SD2058
INPUT			: targer �Ĵ�����ַ DataBuffer ���ݴ��ָ��  ExpectedByteNumber ���Ͷ����ֽڵ�����
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
DESCRIPTION		: ��SD2058��ȡһ������
INPUT			: targer �Ĵ�����ַ DataBuffer ���ݴ��ָ��  ExpectedByteNumber ��ȡ�����ֽڵ�����
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
DESCRIPTION		: SD2058 ��ʼ������
INPUT			: None
OUTPUT			: None
*****************************************************************************/

void SD2058_Time_Init(void)
{
	unsigned char temp[10] = {0} ;
	IIC1_Init();
		
	//temp[7] = 0x80 | 0x21; //ȡ��д����  ����Ƶ���ж�
	temp[7] = 0x80; //ȡ��д����  ����Ƶ���ж�
	I2C1_SendBytes( 0x10 , temp+7 , 1);
	temp[7] = 0x84;
	I2C1_SendBytes( 0x0f , temp+7 , 1);
	
    //temp[7] = 0x0c;
	//I2C1_SendBytes( 0x11 , temp+7 , 1);
	//temp[7] = 0x80 | 0x21; // ����Ƶ���ж�
	//I2C1_SendBytes( 0x10 , temp+7 , 1);
	
    temp[7] = 0x00;
    I2C1_SendBytes( 0x11 , temp+7 , 1);
    temp[7] = 0x80 | 0x00; // ����Ƶ���ж�
    I2C1_SendBytes( 0x10 , temp+7 , 1);

	temp[7] = 0x00;
	I2C1_SendBytes( 0x0f , temp+7 , 1);
	
	temp[7] = 0x00;
	I2C1_SendBytes( 0x10 , temp+7 , 1);	
				
	I2C1_ReadBytes( 0 , temp , 7);
	if (bcd2bin(temp[6]) < 30 && bcd2bin(temp[6]) >= 20) //����30 ��Ϊʱ��δ��ʼ��
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
	//��ʱ��������ʼ��
}

/****************************************************************************
FUNCTION		: SD2058_Rtc_Read_Time
DESCRIPTION		: �� SD2058 ��ȡʱ������
INPUT			: SD2058_VAL �ṹ��ָ��
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
DESCRIPTION		: дʱ�����ݵ� SD2058 
INPUT			: SD2058_VAL �ṹ��ָ��
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
	
	//temp[7] = 0x80 | 0x21; //ȡ��д����  ����Ƶ���ж�
	temp[7] = 0x80; //ȡ��д����  ����Ƶ���ж�
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


//RTCת��Ϊʱ���

#define FEBRUARY		2
#define	STARTOFTIME		1970
#define SECDAY			86400L           /*  һ���ж���s */
#define SECYR			(SECDAY * 365)
#define	leapyear(year)		((year) % 4 == 0)
#define	days_in_year(a) 	(leapyear(a) ? 366 : 365)
#define	days_in_month(a) 	(month_days[(a) - 1])

static int month_days[12] = {	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*
 * This only works for the Gregorian calendar - i.e. after 1752 (in the UK)
 */
 /*���㹫��*/
void GregorianDay(struct rtc_time * tm)
{
	int leapsToDate;
	int lastYear;
	int day;
	int MonthOffset[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };

	lastYear=tm->tm_year-1;

	/*����ӹ�ԪԪ�굽������ǰһ��֮��һ�������˶��ٸ�����*/
	leapsToDate = lastYear/4 - lastYear/100 + lastYear/400;      

     /*������������һ��Ϊ���꣬�Ҽ������·���2��֮����������1�����򲻼�1*/
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

	day += lastYear*365 + leapsToDate + MonthOffset[tm->tm_mon-1] + tm->tm_mday; /*����ӹ�ԪԪ��Ԫ������������һ���ж�����*/

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
 *ADD by fire���������ڹ����е��������Ϊ����ʱ�䣬
 ������ת����ʱ���ʱ���Ҫ�ӱ���ʱ��ת��Ϊ��׼ʱ���ʱ���
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
	/*Add by fire: -8*60*60 ������ı���ʱ��ת��Ϊ��׼ʱ�䣬
	��д���ʱ���У�ȷ����ʱ��������Ϊ��׼��UNIXʱ���*/ 
	 
}



void to_tm(u32 tim, struct rtc_time * tm)
{
	register u32    i;
	register long   hms, day;


	day = tim / SECDAY;			/* �ж����� */
	hms = tim % SECDAY;			/* �����ʱ�䣬��λs */

	/* Hours, minutes, seconds are easy */
	tm->tm_hour = hms / 3600;
	tm->tm_min = (hms % 3600) / 60;
	tm->tm_sec = (hms % 3600) % 60;

	/* Number of years in days */ /*�����ǰ��ݣ���ʼ�ļ������Ϊ1970��*/
	for (i = STARTOFTIME; day >= days_in_year(i); i++) {
		day -= days_in_year(i);
	}
	tm->tm_year = i;

	/* Number of months in days left */ /*���㵱ǰ���·�*/
	if (leapyear(tm->tm_year)) {
		days_in_month(FEBRUARY) = 29;
	}
	for (i = 1; day >= days_in_month(i); i++) {
		day -= days_in_month(i);
	}
	days_in_month(FEBRUARY) = 28;
	tm->tm_mon = i;

	/* Days are what is left over (+1) from all that. *//*���㵱ǰ����*/
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