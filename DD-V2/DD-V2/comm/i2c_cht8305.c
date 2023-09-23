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
DESCRIPTION		: ��ʼ��CHT8305��ʪ�ȱ��� �� I2C2 GPIO��
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
DESCRIPTION		: ����һ�� Data���� �� CHT8305
INPUT			: targer �Ĵ�����ַ Data ��Ӧ����
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
DESCRIPTION		: ��CHT8305�Ķ�Ӧ��ַ��ȡ����
INPUT			: targer �Ĵ�����ַ DataBuffer ���ݴ��ָ��  ExpectedByteNumber ��ȡ�����ֽڵ�����
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
DESCRIPTION		: ����CHT8305����ʪ��ת��������ȡ����
INPUT			: None
OUTPUT			: None
*****************************************************************************/

void CHT8305_GetData(void)
{
	uint8_t data[4];
	uint16_t ad_val = 0;
	int32_t temp = 0;
	static uint8_t i = 0;
	//I2C2_SendByte(CHT8305_TEMPERATURE,0x00); //д0�Ĵ���������ת��
	//delay_ms(100);
	//I2C2_ReadBytes(CHT8305_TEMPERATURE,data,4);//һ���Ӷ�ȡ4���ֽڣ������¶Ⱥ�ʪ��
	if (  i % 2 == 0)
	{
		I2C2_ReadBytePre();
		delay_us(500);
		delay_ms(2);
	}
	else
	{
		I2C2_ReadBytes(CHT8305_TEMPERATURE,data,4);
		//�¶Ⱥ�ʪ��ת����������
		ad_val = data[0] << 8 | data[1]; //�¶�
		cht8305_val.Temperature_AD = ad_val;
		temp = (int16_t)(ad_val * 165 / 65535) - 40;		//�漰�޷���ת������  ��Ҫ��֤
		cht8305_val.Temperature = temp;
	
		ad_val = data[2] << 8 | data[3]; //ʪ��
		cht8305_val.Humidity_AD = ad_val;	
		temp = ad_val * 100 / 65535 ;
		cht8305_val.Humidity = temp; 	
	}

	i++;

}