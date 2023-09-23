/*
 * i2c_cht8305.c
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 

#include "comm/i2c_cht8305.h"

CHT8305_VAL cht8305_val;

/****************************************************************************
FUNCTION		: CHT8305_Init
DESCRIPTION		: 初始化CHT8305温湿度变量 和 I2C2 GPIO口
INPUT			: None
OUTPUT			: None
*****************************************************************************/
void CHT8305_Init(void)
{
	IIC2_Init();
	cht8305_val.Humidity = 0;
	cht8305_val.Humidity_AD = 0;
	cht8305_val.Temperature	 = 0;
	cht8305_val.Temperature_AD = 0;
}

/****************************************************************************
FUNCTION		: I2C2_SendByte
DESCRIPTION		: 发送一个 Data数据 给 CHT8305
INPUT			: targer 寄存器地址 Data 相应数据
OUTPUT			: None
*****************************************************************************/

void I2C2_ReadBytePre(void)
{
	IIC2_Start();
	IIC2_Send_Byte(CHT8305_ADDRESS_W);
	IIC2_Send_Byte(CHT8305_TEMPERATURE);
	IIC2_Stop();
}

/****************************************************************************
FUNCTION		: I2C2_ReadBytes
DESCRIPTION		: 从CHT8305的对应地址读取数据
INPUT			: targer 寄存器地址 DataBuffer 数据存放指针  ExpectedByteNumber 读取多少字节的数据
OUTPUT			: None
*****************************************************************************/

void I2C2_ReadBytes(unsigned char targer, unsigned char *DataBuffer, unsigned int ExpectedByteNumber)
{
	unsigned char i = 0;
	IIC2_Start();
	IIC2_Send_Byte(CHT8305_ADDRESS_R);
	for(i=0;i<ExpectedByteNumber-1;i++)
	{
		*DataBuffer =IIC2_Read_Byte(ACK);
		DataBuffer++;
	}
	*DataBuffer =IIC2_Read_Byte(NACK);
	IIC2_Stop();
}

/****************************************************************************
FUNCTION		: CHT8305_GetData
DESCRIPTION		: 启动CHT8305的温湿度转换，并读取数据
INPUT			: None
OUTPUT			: None
*****************************************************************************/

void CHT8305_GetData(void)
{
	uint8_t data[4];
	uint16_t ad_val = 0;
	int32_t temp = 0;
	static uint8_t i = 0;
	//I2C2_SendByte(CHT8305_TEMPERATURE,0x00); //写0寄存器，启动转换
	//delay_ms(100);
	//I2C2_ReadBytes(CHT8305_TEMPERATURE,data,4);//一下子读取4个字节，读出温度和湿度
	if (  i % 2 == 0)
	{
		I2C2_ReadBytePre();
		delay_us(500);
		delay_ms(2);
	}
	else
	{
		I2C2_ReadBytes(CHT8305_TEMPERATURE,data,4);
		//温度和湿度转换，并处理
		ad_val = data[0] << 8 | data[1]; //温度
		cht8305_val.Temperature_AD = ad_val;
		temp = (int16_t)(ad_val * 165 / 65535) - 40;		//涉及无符号转符号数  需要验证
		cht8305_val.Temperature = temp;
	
		ad_val = data[2] << 8 | data[3]; //湿度
		cht8305_val.Humidity_AD = ad_val;	
		temp = ad_val * 100 / 65535 ;
		cht8305_val.Humidity = temp; 	
	}

	i++;

}