/*
 * CAN_RW_Manage.c
 *
 * Created: 2020/4/16 15:28:35
 *  Author: fjiangqing
 */

#include "CAN_RW_Manage.h"
#include "i2c_sh367309.h"
#include "i2c_cht8305.h"
#include "i2c_sd2058.h"
#include "error_save.h"

uint8_t Send_Succeed = 0; //数据发送成功标志。

//数据缓冲区变量
static uint8_t callback_buffer[XMODEM_BUFLEN]; //数据缓冲区
volatile static uint32_t readOffset, writeOffset; //缓冲区读写指针

static uint32_t CAN_ID_buffer[XMODEM_BUFLEN>>3];

//协议解析区变量
volatile u8 rx_cnt = 0;
volatile unsigned char rx_buffer[100];
unsigned char tx_buffer[250];
uint8_t sn_buffer[16];
uint16_t crc = 0;
// 0x45命令只用到的变量
unsigned int  write_index,firmware_size,rev_packcount,rev_crc16_firmware,write_size,write_crc,rev_version,head_datelen;  
uint8_t boot_state = 0;
uint8_t head_check = 0;
uint16_t crc_MODBUS = 0;
uint16_t crc_i = 0;
uint32_t write_addr;
uint8_t crc_buffer[1];

DD_CAN_EXTEN_ID can_id_old;
DD_CAN_EXTEN_ID can_id_new;




//TODO:标准帧滤波器ID设置
const uint32_t REC_STD_ID[CONF_CAN0_RX_STDID_ID_FILTER_NUM] =
{
    CAN_RX_STDID_CMD
};

//TODO:扩展帧滤波器ID设置
const uint32_t REC_EXTENDED_ID[CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM] =
{
    CAN_RX_EXTENDED_CMD
};

void CAN_RW_Init(void)
{
    struct can_filter filter;

    //CAN外设配置
    //CAN_0_init();
	
	can_id_old.VAL = 0xffffffff;
	can_id_new.VAL = 0xffffffff;
    //CAN接收缓存区
    for (uint16_t CAN_Buffer_Init_Index = 0; CAN_Buffer_Init_Index < CAN_RX_MASSAGE_BUFFER_LENGTH; ++CAN_Buffer_Init_Index)
    {
        CAN_Rx_Message_Buffer[CAN_Buffer_Init_Index].data = &CAN_Rx_Data_Buffer[CAN_Buffer_Init_Index * CAN_RX_DATE_LENGTH_BYTE];
    }

    //接收完成回调
    can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_rx_callback);
    //标准帧滤波器
    filter.mask = 0x1FFFFFFF;
    for (int CAN_Filter_ID_Index = 0; CAN_Filter_ID_Index < CONF_CAN0_RX_STDID_ID_FILTER_NUM; ++CAN_Filter_ID_Index)
    {
        filter.id = REC_STD_ID[CAN_Filter_ID_Index];
        can_async_set_filter(&CAN_0, CAN_Filter_ID_Index, CAN_FMT_STDID, &filter);
    }

    //扩展帧滤波器
    filter.mask = 0x00003f00; //目的地址为6的都可以进来
    for (int CAN_Filter_ID_Index = 0; CAN_Filter_ID_Index < CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM; ++CAN_Filter_ID_Index)
    {
        filter.id = REC_EXTENDED_ID[CAN_Filter_ID_Index];
        can_async_set_filter(&CAN_0, CAN_Filter_ID_Index + CONF_CAN0_RX_STDID_ID_FILTER_NUM, CAN_FMT_EXTID, &filter);
    }

    //配置发送完成回调
    can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_tx_callback);
	can_async_register_callback(&CAN_0, CAN_ASYNC_IRQ_CB, (FUNC_PTR)CAN_error_callback);
    can_async_enable(&CAN_0);
	
	buff_init();
}

//接收回调，将数据存储到内存中待处理。
void CAN_rx_callback(struct can_async_descriptor *const descr)
{
    //TODO:考虑添加大数据量数组存储
    //可以整理成函数
	uint8_t i;
	
    can_async_read(descr, &CAN_Rx_Message_Buffer[CAN_Rx_Buffer_Write_Index]);
	can_id_new.VAL = CAN_Rx_Message_Buffer[CAN_Rx_Buffer_Write_Index].id;
	
	if (can_id_new.bit.pack_queue_num != can_id_old.bit.pack_queue_num) //区分是否需要补充缓冲队列
	{
		write_byte(can_id_new.bit.source_addr);
		write_byte(can_id_new.bit.target_addr);
	}
	can_id_old.VAL = can_id_new.VAL;
	
	for (i=0;i < CAN_Rx_Message_Buffer[CAN_Rx_Buffer_Write_Index].len;i++ )
	{
		write_byte(CAN_Rx_Message_Buffer[CAN_Rx_Buffer_Write_Index].data[i]);
	}
	
	if (can_id_new.bit.frame_num == 0)
	{
		while(writeOffset % 8 != 0)
		{
			write_byte(0x00);
		}
		CAN_ID_buffer[writeOffset >> 3] = can_id_new.VAL;
	}
	
    ++CAN_Rx_Buffer_Write_Index;
    if (CAN_Rx_Buffer_Write_Index >= CAN_RX_MASSAGE_BUFFER_LENGTH)
    {
        CAN_Rx_Buffer_Write_Index = 0;
    }
	
	
}


void CAN_tx_callback(struct can_async_descriptor *const descr)
{
	//发送成功标志
    Send_Succeed = 1;
}

void CAN_error_callback(struct can_async_descriptor *const descr, enum can_async_interrupt_type type)
{
	if (type == CAN_IRQ_EW)
	{
		//flash_save_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,chg_cap_cnt_write_index,sys_cap.val.chg_cap_cnt);
		//chg_cap_cnt_write_index++;
		//chg_cap_cnt_write_index &= 0x07;
		//
		//flash_save_data(VER_FLASH_ADDR+FLASH_CYCLES_ADDR,bat_cycle_cnt_write_index,sys_cap.val.bat_cycle_cnt);
		//bat_cycle_cnt_write_index++;
		//bat_cycle_cnt_write_index &= 0x07;
		//
		//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
		//re_cap_rate_write_index++;
		//re_cap_rate_write_index &= 0x07;
		//NVIC_SystemReset();
	}
	else if (type == CAN_IRQ_EP)
	{
		//flash_save_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,chg_cap_cnt_write_index,sys_cap.val.chg_cap_cnt);
		//chg_cap_cnt_write_index++;
		//chg_cap_cnt_write_index &= 0x07;
		//
		//flash_save_data(VER_FLASH_ADDR+FLASH_CYCLES_ADDR,bat_cycle_cnt_write_index,sys_cap.val.bat_cycle_cnt);
		//bat_cycle_cnt_write_index++;
		//bat_cycle_cnt_write_index &= 0x07;
		//
		//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
		//re_cap_rate_write_index++;
		//re_cap_rate_write_index &= 0x07;
		//NVIC_SystemReset();
	}
	else if (type == CAN_IRQ_BO)
	{
		//flash_save_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,chg_cap_cnt_write_index,sys_cap.val.chg_cap_cnt);
		//chg_cap_cnt_write_index++;
		//chg_cap_cnt_write_index &= 0x07;
		//
		//flash_save_data(VER_FLASH_ADDR+FLASH_CYCLES_ADDR,bat_cycle_cnt_write_index,sys_cap.val.bat_cycle_cnt);
		//bat_cycle_cnt_write_index++;
		//bat_cycle_cnt_write_index &= 0x07;
		//
		//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
		//re_cap_rate_write_index++;
		//re_cap_rate_write_index &= 0x07;
		//NVIC_SystemReset();
	}

	
}


/**
 * \brief 通过CAN0 发送数据
 *
 * \param[in] ID				发送消息ID
 * \param[in] format			标准帧或者扩展帧	 CAN_FMT_STDID/CAN_FMT_EXTID
 * \param[in] send_data_addr	发送数据的首地址
 * \param[in] length		    要求发送数据长度
 * \备注：可用于在一个ID中传输大于8byte，代码自动分帧。
 */
uint8_t CAN_Send(uint8_t priority, enum can_format format, uint8_t *send_data_addr, uint16_t length)
{
    uint8_t const One_Send_Byte_Num = 8;
    uint16_t Cycle_Cnt = (length-2) / One_Send_Byte_Num;
	uint8_t frame_num = Cycle_Cnt;
	uint8_t buff_temp[1];

    struct  can_message msg;
	
	DD_CAN_EXTEN_ID Send_ID_Value;	
	DD_CAN_EXTEN_ID Rec_ID_Value;	
	
	while (readOffset % 8 != 0) //读写都保持8个字节对齐
	{
		read_bytes(buff_temp,1);
	}
	
	Rec_ID_Value.VAL = CAN_ID_buffer[readOffset >> 3];
	
	Send_ID_Value.VAL = 0;
	
	Send_ID_Value.bit.source_addr = send_data_addr[0];
	Send_ID_Value.bit.target_addr = send_data_addr[1];
	Send_ID_Value.bit.protocol_num = 0x03;
	Send_ID_Value.bit.priority = priority & 0x07;
	Send_ID_Value.bit.pack_queue_num = Rec_ID_Value.bit.pack_queue_num;
	
	Send_ID_Value.bit.frame_num = frame_num & 0x1f;
	
	send_data_addr = send_data_addr+2;
	length = length-2;
	
	msg.id = Send_ID_Value.VAL;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data_addr;
	msg.len  = 8;
	msg.fmt  = format;
	
	if (length % One_Send_Byte_Num == 0)
	{
		frame_num--;
	}
	
    for (uint16_t Send_Cycle_Index = 0; Send_Cycle_Index < Cycle_Cnt; ++Send_Cycle_Index)
    {

		Send_ID_Value.bit.frame_num = frame_num & 0x1f;
		msg.id = Send_ID_Value.VAL;
		
        Send_Succeed = 0;
        can_async_write(&CAN_0, &msg);
        //要求确认发送完成
        if (false == CAN_Send_Wait(SEND_WAIT_TIMER))
        {
			if (hri_can_get_CCCR_INIT_bit((&(&CAN_0)->dev)->hw) == 1)
			{
				__NOP();
				__NOP();
				__NOP();
				__NOP();
				
				hri_can_set_CCCR_CCE_bit((&(&CAN_0)->dev)->hw);
				hri_can_clear_CCCR_INIT_bit((&(&CAN_0)->dev)->hw);
				
				while(hri_can_get_CCCR_INIT_bit((&(&CAN_0)->dev)->hw) == 1);
				__NOP();
				__NOP();
				__NOP();
				
			}
            //return false;
        }
        msg.data += One_Send_Byte_Num;
		if (frame_num > 0)
		{
			frame_num--;
		}
    }
	//发送小于8的长度数据。
    if (length % One_Send_Byte_Num != 0)
    {
		Send_ID_Value.bit.frame_num = frame_num & 0x1f;
		msg.id = Send_ID_Value.VAL;
		msg.len = length % One_Send_Byte_Num;
        can_async_write(&CAN_0, &msg);
    }
	
    return true;
}


//timer: 单位1us
uint8_t CAN_Send_Wait(uint16_t timer)
{
    uint16_t Send_Wait_Timer = 0;

    timer /= 10;
    while (!Send_Succeed)
    {
        delay_us(10);
        ++Send_Wait_Timer;
        if (Send_Wait_Timer >= timer)
        {
            ++CAN_Tx_Error_Cnt;
            return false;       //错误
        }
        //TODO:添加看门狗？
    }
    CAN_Tx_Error_Cnt = 0;
    return true;               //正确
}


void buff_init(void)
{
	readOffset = 0;
	writeOffset = 0;
}

void write_byte(uint8_t byte)
{
	callback_buffer[writeOffset++] = byte;
	writeOffset &= XMODEM_BUFLEN - 1;
}

void read_bytes(uint8_t * buffer, uint32_t byteCount)
{
	uint32_t currentBytesRead = 0;

	while(currentBytesRead != byteCount)
	{
		//wdt_reset_count();
		if (readOffset != writeOffset)
		{
			buffer[currentBytesRead++] = callback_buffer[readOffset++];
			readOffset &= XMODEM_BUFLEN - 1;
		}
	}
}

uint8_t buff_empty(void)
{
	return (readOffset == writeOffset);
}


//协议解析函数
void Comm_Proc(void)
{
	while(sys_uarts.flag.uart_receive_flag == 0 && readOffset != writeOffset )
	{
		if(rx_cnt == 0)
		{
			//rx_buffer[0]= USART_ReceiveData(HT_UART1);
			read_bytes(&rx_buffer[rx_cnt],1); // 源地址 
			rx_cnt++;
		}
		else if (rx_cnt == 1)
		{
			read_bytes(&rx_buffer[rx_cnt],1);
			if (rx_buffer[rx_cnt] == BMS_ADDRESS)
			{
				rx_cnt++;
			} 
			else
			{
				rx_cnt = 0;
			}
		}
		else
		{
			//rx_buffer[rx_cnt]= USART_ReceiveData(HT_UART1);
			read_bytes(&rx_buffer[rx_cnt],1);
			rx_cnt++;
	
			if(rx_buffer[2] == READ_R_FUNTION)
			{
				if(rx_cnt > 15)
				{
					rx_cnt = 0;
				}
				if(rx_cnt > 7) // Read_R_FUNTION 功能码一帧 8个字节
				{
					if(rx_buffer[3] ==0xa0 || rx_buffer[3] ==0xa2) //A0XX 或者 A2XX 寄存器地址
					{
						sys_uarts.flag.uart_receive_flag =1;
					}
					if(rx_buffer[3] ==0xa3 && rx_buffer[4] ==0x00) //暂未看到0XA300 的寄存器地址
					{
						sys_uarts.flag.uart_receive_flag =1;
					}
				}
				if(rx_cnt > 9)  
				{
					if(rx_buffer[3] ==0xa3 && rx_buffer[4] ==0x01)
					{
						sys_uarts.flag.uart_receive_flag =1;
					}
				}
			}
			else
			{
				if(rx_buffer[2] == 0xF3) //0XF3 功能码 暂不修改
				{
					if(rx_cnt >2)
					{
						if(rx_buffer[3] == 0x00)
						{
							if(rx_cnt > 39)
							{
								rx_cnt =0;
							}
							if(rx_cnt >4)
							{
								if(rx_cnt == rx_buffer[4]+6)
								{
									sys_uarts.flag.uart_receive_flag =1;
								}
							}
						}
						else
						{
							if(rx_buffer[3] == 0x01)
							{
								if(rx_cnt > 7)
								{
									rx_cnt =0;
								}
								if(rx_cnt == 6)
								{
									sys_uarts.flag.uart_receive_flag =1;
								}
							}
							else
							{
								rx_cnt =0;
							}
						}
					}
				}
				else
				{
					if(rx_buffer[2] == WRITE_R_FUNTION || rx_buffer[2] == 0xE0 || rx_buffer[2] == 0xE2 ) // E0 E2功能码 暂时不修改
					{
						if(rx_cnt >39)
						{
							rx_cnt =0;
						}
						if(rx_cnt >6)
						{
							if(rx_cnt == (rx_buffer[5]<<1)+8)
							{
								sys_uarts.flag.uart_receive_flag =1;
							}
						}
					}
					else
					{
						if(rx_buffer[2] == UPDATE_FUNTION)
						{
							if(rx_cnt >99)
							{
								rx_cnt =0;
							}
							if(rx_cnt >6)
							{
								if(rx_cnt == ((rx_buffer[4] << 8) + rx_buffer[5]+8 ) )
								{
									sys_uarts.flag.uart_receive_flag =1;
								}
							}
						}
						else
						{
							if(rx_buffer[2] == 0xF0)
							{
								if(rx_cnt >39)
								{
									rx_cnt =0;
								}
								if(rx_cnt >7)
								{
									if(rx_cnt == rx_buffer[4]+6)
									{
								
										sys_uarts.flag.uart_receive_flag =1;
									}
								}
							}
							else
							{
								if(rx_buffer[2] == 0xE3)
								{
									if(rx_cnt >39)
									{
										rx_cnt =0;
									}
									if(rx_cnt >22)
									{
										if(rx_cnt == rx_buffer[4]+7)
										{
											sys_uarts.flag.uart_receive_flag =1;
										}
									}
								}
								if(rx_buffer[2] == 0xE4 || rx_buffer[2] == 0xE1)
								{
									if(rx_cnt >39)
									{
										rx_cnt =0;
									}
									if(rx_cnt >5)
									{
										if(rx_cnt == rx_buffer[3]+5)
										{
											sys_uarts.flag.uart_receive_flag =1;
										}
									}
								}
								if(rx_cnt >25)
								{
									rx_cnt =0;
								}
							}
						}
					}
				}
			}
		}
	Uart_DIDI_Rx();
	}
}

//DIDI 协议解析
void Uart_DIDI_Rx(void)
{
	u16 crc,i;
	u8 check_sum;
	if(sys_uarts.flag.uart_receive_flag == 1)
	{
		if(rx_buffer[2] == 0xF3)
		{
			check_sum =0;
			for(i=0;i<(rx_cnt-1);i++)
			{
				check_sum+= rx_buffer[i];
			}
			if(rx_buffer[rx_cnt-1] == check_sum)
			{
				//uart_sleep ++;
				//								Uart_DIDI_Tx(rx_buffer[2],rx_buffer[1]);
			}
		}
		else
		{
			if(rx_buffer[2] == UPDATE_FUNTION)
			{
				crc = CRC16_MODBUS(rx_buffer,((rx_buffer[4] <<8) + rx_buffer[5] + 6),0);
			}
			else
			{
				if(rx_buffer[2] == WRITE_R_FUNTION || rx_buffer[2] == 0xE0 || rx_buffer[2] == 0xE2)
				{
					crc = CRC16_MODBUS(rx_buffer,((rx_buffer[5]<<1)+6),0);
				}
				else
				{
					if(rx_buffer[2] == 0xF0)
					{
						crc = RX_CheckSum(rx_buffer,6);
					}
					else
					{
						if(rx_buffer[2] == 0xE3)
						{
							crc = CRC16_MODBUS(rx_buffer,21,0);
						}
						else
						{
							if(rx_buffer[2] == 0xE4 || rx_buffer[2] == 0xE1)
							{
								crc = CRC16_MODBUS(rx_buffer,4,0);
							}
							else // 剩下0X03 读寄存器指令
							{
								if(rx_buffer[3] == 0xA3 && rx_buffer[4] == 0x01)
								{
									crc = CRC16_MODBUS(rx_buffer,8,0);
								}
								else
								{
									crc = CRC16_MODBUS(rx_buffer,6,0);
								}
							}
						}
					}
				}
			}
			if(((u8)(crc>>8) == rx_buffer[rx_cnt-1])&&((u8)crc == rx_buffer[rx_cnt-2]))
			{
				//uart_timeout_cnt =0;
				//在这里清标志位
				noid_enter_sleep_count = 0;
				id_enter_sleep_count = 0;
				sys_flags.flag.sleep_flag = 0;
				
				Uart_DIDI_Tx(0,rx_buffer[2]);
			}
		}
		//id_nc_sleep = 0;
		sys_uarts.flag.uart_receive_flag =0;
		rx_cnt =0;
	}
}



/*****************************************************************************

FUNCTION		: CRC16_MODBUS
DESCRIPTION		:
INPUT			: None
OUTPUT			: None
NOTICE			:
DATE			: 2018/10/19

******************************************************************************/
unsigned short CRC16_MODBUS(volatile unsigned char *puchMsg, unsigned int usDataLen,u8 index)
{
	unsigned short wCRCin = 0xFFFF;
	unsigned short wCPoly = 0x8005;
	unsigned char wChar = 0;
	int i=0,j=0;
	j= index;
	while (usDataLen--)
	{
		wChar = puchMsg[j];
		j++;
		wChar = InvertUint8(wChar,wChar);
		wCRCin ^= (wChar << 8);
		for(i = 0;i < 8;i++)
		{
			if(wCRCin & 0x8000) wCRCin = (wCRCin << 1) ^ wCPoly;
			else wCRCin = wCRCin << 1;
		}
	}
	wCRCin = InvertUint16(wCRCin,wCRCin);
	return wCRCin;
}

unsigned short CRC16_MODBUS_ONE(volatile unsigned char *puchMsg, unsigned int usDataLen,u8 index,u32 crc_i,u16 crc)
{
	unsigned short wCRCin = crc;//0xFFFF;
	
	unsigned short wCPoly = 0x8005;
	unsigned char wChar = 0;
	int i=0,j=0;
	j= index;
	while (usDataLen--)
	{
		wChar = puchMsg[j];
		j++;
		wChar = InvertUint8(wChar,wChar);
		wCRCin ^= (wChar << 8);
		for(i = 0;i < 8;i++)
		{
			if(wCRCin & 0x8000) wCRCin = (wCRCin << 1) ^ wCPoly;
			else wCRCin = wCRCin << 1;
		}
	}
	if(crc_i == (rev_packcount - 1))
	{
		wCRCin = InvertUint16(wCRCin,wCRCin);
	}
	return wCRCin;
}


u8  InvertUint8(unsigned char dBuf,unsigned char srcBuf)
{
	int i;
	unsigned char tmp;
	tmp = 0;
	for(i=0;i< 8;i++)
	{
		if(srcBuf& (1 << i))
		tmp|=1<<(7-i);
	}
	return tmp;
}

u16  InvertUint16(unsigned short dBuf,unsigned short srcBuf)
{
	int i;
	unsigned short tmp;
	tmp = 0;
	for(i=0;i< 16;i++)
	{
		if(srcBuf& (1 << i))
		tmp|=1<<(15 - i);
	}
	return tmp;
}

unsigned short RX_CheckSum(volatile unsigned char *buf, u8 len) //buf为数组，len为数组长度
{
	unsigned short i, ret = 0;
	
	for(i=0; i<len; i++)
	{
		ret += *(buf++);
	}
	return ret;
}



void Uart_DIDI_Tx(u8 rw,u8 cmd)
{
	uint32_t tmp,read_addr;
	uint16_t Flash_Buf[150];
	uint16_t tmp_16,data_max_len;
	uint8_t i=0,j=0,m=0,flag_tmp;
	uint8_t check_sum =0;
	int8_t s_temp;
	int16_t s16_tmp;
	uint8_t SNwrite_buffer[16]={0};
	crc = 0;
	switch(cmd)
	{	
		case READ_R_FUNTION:// 0x03指令解析到这里 代表成功
			//标志位待更新 +w sys_w25.val.read_start = 0;//191022 chh 增加在通信读取电池状态时清0，解决在读取EEROM过程停止读取但发生其他通信标志位不会被清0导致EEROM无法再写
			sys_uarts.flag.uart_update_ok_flag =0;
			if(rx_buffer[3]==0xa2)
			{
				//A200---电池组健康百分比SOH + 是否输出电压 默认不输出
				tmp = (u32)sys_cap.val.full_cap*100;
				tmp = tmp/DESIGN_CAP; //  +w 容量待定
				if(tmp >100)
				{
					tmp = 100;
				}
				tmp <<=8;
				tmp &= 0xff00;
				if(sys_flags.flag.dch_en_flag ==1)
				{
					tmp &= 0xff00;
					tmp += 1;
				}
				Flash_Buf[0] = (u16)tmp;
				//A201---BMS输出延迟  默认2S
				Flash_Buf[1] = 0x02;
				//A202---内部温度  开尔文
				s16_tmp =(s16)Info.afe.Temperature1 + 2731;
				//s16_tmp =(s16)Info.afe.Temperature4*10 + 2731; //配合DD上位机测试 方便读取温度值
				Flash_Buf[2] = s16_tmp;

				//A203---电池组电压  Info.afe.Voltage
				tmp = (u32)Info.afe.Voltage;
				Flash_Buf[3] = (u16)(tmp>>16);
				Flash_Buf[4] = (u16)tmp;
				//A205---电池组实时电流  32位有符号 Info.afe.CurCadc
				tmp = Info.afe.CurCadc;
				Flash_Buf[5] = (u16)(tmp>>16);
				Flash_Buf[6] = (u16)tmp;
				//A207---相对容量 + 绝对容量
				tmp_16 = sys_cap.val.re_cap_rate;
				tmp = (u32)sys_cap.val.bat_cap*100/DESIGN_CAP;//17400;
				i = (u8)tmp;
				tmp_16 <<=8;
				tmp_16 &= 0xff00;
				tmp_16 |= i;
				Flash_Buf[7] = tmp_16;
				//A208---电池剩余容量
				Flash_Buf[8] = 0;
				Flash_Buf[9] = sys_cap.val.bat_cap;
				//A20A---电池满电容量
				Flash_Buf[10] = 0;
				Flash_Buf[11] = sys_cap.val.full_cap;
				//A20C---电池循环次数
				Flash_Buf[12] = 0;
				Flash_Buf[13] = sys_cap.val.bat_cycle_cnt;
				//A20E---保留
				Flash_Buf[14] = 0;

				Flash_Buf[15] = 0;
				//A210---高16节电芯电压 待补充 原来DD-H2是10S
				for(i=0;i<16;i++)
				{
					tmp = Info.afe.VCell[i];
					Flash_Buf[16+i] = tmp;
				}

				//A220---低16节电芯电压
				for(i=0;i<16;i++)
				{
					Flash_Buf[32+i] = 0;
				}
				flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
				if (tmp_16 == 0xffff)
				{
					tmp_16 = 0;
				}
				else
				{
					tmp_16 = 0xffff;
				}
				Flash_Buf[32] = tmp_16;
				flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_NO_SOCLP_ADDR,(uint8_t *)&tmp_16, 2);
				if (tmp_16 == 0xffff)
				{
					tmp_16 = 0;
				}
				else
				{
					tmp_16 = 0xffff;
				}
				Flash_Buf[33] = tmp_16;			
								
				//A230---当前充电间隔
				Flash_Buf[48] = 0;
				//A231---最大充电间隔
				Flash_Buf[49] = 0;
				//A232---最大放电电流 mA
				Flash_Buf[50] = (uint16_t)(MAX_DCH_CUR);
				//A233---最大充电电压 mV
				Flash_Buf[51] = (uint16_t)(MAX_CHG_VOL);
				//A234---最大充电电流
				Flash_Buf[52] = (uint16_t)(MAX_CHG_CUR);
				//A235---一级放电电压保护
				//
				Flash_Buf[53] = (uint16_t)VBAT_UVP; //
				//A236---BMS LED灯显示状态
				Flash_Buf[54] = 0;  //LED灯状态 待补充
					
			//	RTC_countvalue = RTC_GetCounter();
				//(RTC_countvalue >> 16);	
				//A237---BMS LED灯显示状态
				Flash_Buf[55] = 0;//sys_cap.val.chg_cap_cnt;//RTC_countvalue;
				//A238---电池状态信息
				flag_tmp =0;
				if(AFE.BSTATUS3.Bit.CHG_FET == 1)
				{
					flag_tmp |= 0x80;
				}
				if(AFE.BSTATUS3.Bit.DSG_FET == 1)
				{
					flag_tmp |= 0x40;
				}
				if(sys_flags.flag.adapter_connect_flag == 1)
				{
					flag_tmp |= 0x04;
				}
				
				if(sys_flags.flag.sys_id_connect_flag == 1)
				{
					flag_tmp |= 0x02;
				}	
							
				if(sys_flags.flag.sys_comm_open_dch_flag == 1)
				{
					flag_tmp |= 0x01;
				}				
				
				
				Flash_Buf[56] = flag_tmp;
				Flash_Buf[56] <<=8;
				flag_tmp =0;
				if(sys_flags.flag.chg_en_flag == 1 && sys_flags.flag.chg_state_flag == 1)
				{
					flag_tmp |= 0x80;
				}
					
				Flash_Buf[56] |= flag_tmp;
				flag_tmp =0;
				if(sys_flags.flag.sys_sh367309_comm_err_flag ==1)
				{
					flag_tmp |= 0x01;
				}
				if(protects.protect.cell_low_2v_flag ==1)
				{
					flag_tmp |= 0x02;
				}
				if(protects.protect.cell_dif_volt_flag ==1)
				{
					flag_tmp |= 0x04;
				}
				if(protects.protect.dch_mos_err_flag ==1)
				{
					flag_tmp |= 0x40;
				}
				if(protects.protect.chg_mos_err_flag ==1)
				{
					flag_tmp |= 0x80;
				}
				Flash_Buf[57] = flag_tmp;
				Flash_Buf[57] <<=8;
				flag_tmp =0;
				if(protects.protect.chg_ovp_flag ==1)
				{
					flag_tmp |= 0x01;
				}
				if( protects2.protect.soc_lp_flag ==1 || protects.protect.dch_lvp_flag == 1)
				{
					flag_tmp |= 0x02;
				}
				if(protects.protect.dch_lvp2_flag ==1)
				{
					flag_tmp |= 0x04;
				}
				if(protects.protect.dch_ocp_flag ==1)
				{
					flag_tmp |= 0x08;
				}
				if(protects.protect.dch_ocp2_flag == 1)
				{
					flag_tmp |= 0x10;
				}
				if(protects.protect.chg_ocp_flag ==1)
				{
					flag_tmp |= 0x20;
				}
				Flash_Buf[57] |= flag_tmp;
					
				flag_tmp =0;
				if((protects2.protect.dch_ntc_err_flag ==1)||(protects2.protect.chg_ntc_err_flag ==1))
				{
					flag_tmp |= 0x01;
				}
				if(protects.protect.ntc_open_flag == 1 || protects.protect.ntc_short_flag == 1) //电芯温度传感器故障
				{
					flag_tmp |= 0x02;
				}
				if(protects2.protect.dch_otp_flag ==1)
				{
					flag_tmp |= 0x04;
				}
				if(protects2.protect.chg_otp_flag ==1)
				{
					flag_tmp |= 0x08;
				}
				if(protects2.protect.dch_ltp_flag ==1)
				{
					flag_tmp |= 0x10;
				}
				if(protects2.protect.chg_ltp_flag ==1)
				{
					flag_tmp |= 0x20;
				}
				if(protects2.protect.mosfet_temp_err_flag ==1)
				{
					flag_tmp |= 0x40 | 0x80;
				}
				
				
				Flash_Buf[58] = flag_tmp;
				Flash_Buf[58] <<=8;
				flag_tmp =0;
				if(protects2.protect.temp_diff_flag ==1)
				{
					flag_tmp |= 0x02;
				}				
				if(protects.protect.cell_dif_volt_flag ==1)
				{
					flag_tmp |= 0x04;
				}				
		
				if(protects.protect.dch_scd_flag ==1)
				{
					flag_tmp |= 0x10;
				}

				Flash_Buf[58] |= flag_tmp;
					
				flag_tmp = 0;
				if(protects2.protect.cell_dif_volt_chgwaring_flag == 1)
				{
					flag_tmp |= 0x40;
				}
				if(protects2.protect.cell_dif_volt_dsgwaring_flag == 1)
				{
					flag_tmp |= 0x80;
				}
				Flash_Buf[59] = flag_tmp;
				Flash_Buf[59] <<=8;
				flag_tmp = 0;
				//待补充 +w if(warings.waring.chg_one_ovw_flag == 1)
				{
					//flag_tmp |= 0x01;
				}
				//待补充 +w if(warings.waring.dsg_sed_uvw_flag == 1)
				{
					//flag_tmp |= 0x04;
				}
				//待补充 +w if(warings.waring.chg_ocw_flag == 1)
				{
					//flag_tmp |= 0x20;
				}
				Flash_Buf[59] |= flag_tmp;
				flag_tmp = 0;
				//待补充 +w if(warings.waring.dsg_otw_flag == 1)
				{
					//flag_tmp |= 0x04;
				}
				//待补充 +w if(warings.waring.chg_otw_flag == 1)
				{
					//flag_tmp |= 0x08;
				}
				//待补充 +w if(warings.waring.dsg_ltw_flag == 1)
				{
					//flag_tmp |= 0x10;
				}					
				//待补充 +w if(warings.waring.chg_ltw_flag == 1)
				{
					//flag_tmp |= 0x20;
				}
				Flash_Buf[60] = flag_tmp;
				Flash_Buf[60] <<= 8;
				flag_tmp = 0;				
				Flash_Buf[60] |= flag_tmp;
				Flash_Buf[61] = 0;
				Flash_Buf[62] = AFE.BALH;
				Flash_Buf[62] <<=8;
				Flash_Buf[62] |= AFE.BALL;
				Flash_Buf[63] = 0;
					
				//A240---电芯型号
					
				Flash_Buf[64] = 'I';
				Flash_Buf[64]<<=8;
				Flash_Buf[64] += 'F';
				Flash_Buf[65] = 'P';
				Flash_Buf[65]<<=8;
				Flash_Buf[65] += '2';
				Flash_Buf[66] = '2';
				Flash_Buf[66]<<=8;
				Flash_Buf[66] += '6';
				Flash_Buf[67] = '5';
				Flash_Buf[67]<<=8;
				Flash_Buf[67] += '1';
				Flash_Buf[68] = '4';
				Flash_Buf[68]<<=8;
				Flash_Buf[68] += '6';
				Flash_Buf[69] = '-';
				Flash_Buf[69]<<=8;
				Flash_Buf[69] += '2';
				Flash_Buf[70] = '3';
				Flash_Buf[70]<<=8;
				Flash_Buf[70] += 'A';

				
				Flash_Buf[71] = 0;
				//A248---RTC时钟
				Buff_Time_To_Struct_Time(&sd2058_val,&systmtime);
				tmp = mktimev(&systmtime);
				Flash_Buf[72] = tmp>>16;;
				Flash_Buf[73] = tmp&0xffff;
				//A24A---电池设计容量
				Flash_Buf[74] = DESIGN_CAP;
				//A24B---电池累积放电AH数
				Flash_Buf[75] = 0;
				Flash_Buf[76] = 0;
				//A24D---温度2
					
				s16_tmp =(s16)Info.afe.Temperature2 + 2731;
				Flash_Buf[77] = s16_tmp;
				//A24E---温度3

				s16_tmp =(s16)Info.afe.Temperature3 + 2731;
				Flash_Buf[78] = s16_tmp;
				
				//A24F---温度4
				s16_tmp =(s16)Info.afe.Temperature4 + 2731;
				Flash_Buf[79] = s16_tmp;
					
				//A250 MOS管温度
				s16_tmp =(s16)Info.g_NTC1_DSGFET_T + 2731;
				Flash_Buf[80] = s16_tmp;
				//A251 湿度
				
				Flash_Buf[81] = cht8305_val.Humidity;
				
				//A252 SOP
				Flash_Buf[82] = 0;
				Flash_Buf[83] = 0;
				Flash_Buf[84] = 0;
				Flash_Buf[85] = 0;
				
				//A256 FCC
				Flash_Buf[86] = 0;
				Flash_Buf[87] = 0;	
				
				//A258 累计充电能量
				Flash_Buf[88] = 0;	
				Flash_Buf[89] = 0;	
				Flash_Buf[90] = 0;	
				Flash_Buf[91] = 0;			
				
				//A25C	累计放电能量
				Flash_Buf[92] = 0;
				Flash_Buf[93] = 0;
				Flash_Buf[94] = 0;
				Flash_Buf[95] = 0;
				
				//A260 绝缘电阻
				Flash_Buf[96] = 0;
				Flash_Buf[97] = 0;
				
				//A262 是否允许充电
				Flash_Buf[98] = 0x100;
				
				//A263
				Flash_Buf[99] = 0;
				
				m = (rx_buffer[3]<<8) + rx_buffer[4]; //寄存器起始地址
				j = rx_buffer[5]*2; //寄存器读取数量
				tx_buffer[0] = 0x03; //源地址
				tx_buffer[1] = rx_buffer[0];
				tx_buffer[2] = cmd;
				tx_buffer[3] = j;
				for(i=0;i<j;i++)
				{
					if(i%2==0)
					{
						tx_buffer[4+i] = (Flash_Buf[m+i/2]>>8);
					}
					else tx_buffer[4+i] = (Flash_Buf[(m+i-(i+1)/2)]);
				}	
				crc = CRC16_MODBUS(tx_buffer,4+j,0);
				tx_buffer[4+j] = crc;
				tx_buffer[4+j+1] = crc>>8;
				crc =j+6;
			}
			else if(rx_buffer[3]==0xa0)
			{
				//A000---设备类型 
				Flash_Buf[0] = 0;
				Flash_Buf[1] = 0x03;
				//A002---版本号
				tmp_16 = sys_design.val.soft_version/100;
				Flash_Buf[2] = (u8)tmp_16;
				Flash_Buf[2] += 0x30;
				Flash_Buf[2] <<=8;
				tmp_16 = sys_design.val.soft_version%100;
				tmp_16 = tmp_16/10;
				Flash_Buf[2]  |=  tmp_16;
				Flash_Buf[2] += 0x30;
				tmp_16 = sys_design.val.soft_version%10;
				tmp_16 += 0x30;
				Flash_Buf[3] = tmp_16<<8;
							
				//A004---硬件版本
				tmp_16 = sys_design.val.hard_version/100;
				Flash_Buf[4] = (tmp_16+ 0x30);
				Flash_Buf[4] <<=8;
				tmp_16 = sys_design.val.hard_version%100;
				tmp_16 = tmp_16/10;
				Flash_Buf[4] |= (tmp_16+0x30);
				//A005---设备制造商信息
				Flash_Buf[5] = 'S';
				Flash_Buf[5]<<=8;
				Flash_Buf[5] += 'C';
				Flash_Buf[6] = 'U';
				Flash_Buf[6]<<=8;
				Flash_Buf[6] += 'D';
				Flash_Buf[7] = 0x0;
				Flash_Buf[8] = 0x0;
				Flash_Buf[9] = 0x0;
				Flash_Buf[10] = 0x0;
				Flash_Buf[11] = 0x0;
				Flash_Buf[12] = 0x0;
				//A00D---SN码
				flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_SN_ADDR, sn_buffer, 16);	
				Flash_Buf[13] = sn_buffer[0];
				Flash_Buf[13]<<=8;
				Flash_Buf[13] |= sn_buffer[1];
				Flash_Buf[14] = sn_buffer[2];
				Flash_Buf[14]<<=8;
				Flash_Buf[14] |=sn_buffer[3];
				Flash_Buf[15] = sn_buffer[4];
				Flash_Buf[15]<<=8;
				Flash_Buf[15] |= sn_buffer[5];
				Flash_Buf[16] = sn_buffer[6];
				Flash_Buf[16]<<=8;
				Flash_Buf[16] |=sn_buffer[7];
				Flash_Buf[17] = sn_buffer[8];
				Flash_Buf[17]<<=8;
				Flash_Buf[17] |= sn_buffer[9];
				Flash_Buf[18] = sn_buffer[10];
				Flash_Buf[18]<<=8;
				Flash_Buf[18] |=sn_buffer[11];
				Flash_Buf[19] = sn_buffer[12];
				Flash_Buf[19]<<=8;
				Flash_Buf[19] |= sn_buffer[13];
				Flash_Buf[20] = sn_buffer[14];
				Flash_Buf[20]<<=8;
				Flash_Buf[20] |= sn_buffer[15];
//							for(i=0;i<6;i++)
//							{
//								Flash_Buf[15+i] = '0';//sn_buffer[0+i*2];
//								Flash_Buf[15+i]<<=8;
//								Flash_Buf[15+i]+= '0';//sn_buffer[1+i*2];
//								Flash_Buf[16] = 0x3030;
//								Flash_Buf[17] = 0x3030;
//								Flash_Buf[18] = 0x3030;
//								Flash_Buf[19] = 0x3030;
//								Flash_Buf[20] = 0x3030;
//							}
				//20
				//A015---编号
				Flash_Buf[21] = 0x0010;
				//A016---从固件版本号 20
				Flash_Buf[22] = 'F';
				Flash_Buf[22]<<=8;
				Flash_Buf[22] += 'W';
				Flash_Buf[23] = 'B';
				Flash_Buf[23]<<=8;
				Flash_Buf[23] += 'A';
							
				Flash_Buf[24] = 'T';
				Flash_Buf[24]<<=8;
				Flash_Buf[24] += '-';
				Flash_Buf[25] = 'D';
				Flash_Buf[25]<<=8;
				Flash_Buf[25] += 'D';
							
				Flash_Buf[26] = '-';
				Flash_Buf[26]<<=8;
				Flash_Buf[26] += 'A';
				Flash_Buf[27] = '1';
				Flash_Buf[27]<<=8;
				Flash_Buf[27] += '0';
							
				Flash_Buf[28] = '-';
				Flash_Buf[28]<<=8;
				Flash_Buf[28] += 'V';
				Flash_Buf[29] = (u8)(Flash_Buf[2]>>8);
				Flash_Buf[29]<<=8;
				Flash_Buf[29] += (u8)(Flash_Buf[2]);
				Flash_Buf[30] = (u8)(Flash_Buf[3]>>8);
				Flash_Buf[30]<<=8;
				Flash_Buf[30] += 0;//'a';
				Flash_Buf[31] = 0x0;

				//A020---从硬件版本号
				Flash_Buf[32] = 'H';
				Flash_Buf[32]<<=8;
				Flash_Buf[32] += 'W';
				Flash_Buf[33] = 'B';
				Flash_Buf[33]<<=8;
				Flash_Buf[33] += 'A';
							
				Flash_Buf[34] = 'T';
				Flash_Buf[34]<<=8;
				Flash_Buf[34] += '-';
				Flash_Buf[35] = 'D';
				Flash_Buf[35]<<=8;
				Flash_Buf[35] += 'D';
							
				Flash_Buf[36] = '-';
				Flash_Buf[36]<<=8;
				Flash_Buf[36] += 'A';
				Flash_Buf[37] = '1';
				Flash_Buf[37]<<=8;
				Flash_Buf[37] += '0';
							
				Flash_Buf[38] = '-';
				Flash_Buf[38]<<=8;
				Flash_Buf[38] += 'V';
				
				tmp_16 = sys_design.val.hard_version/100;
				Flash_Buf[39] = (tmp_16+ 0x30);
				Flash_Buf[39] <<=8;
				tmp_16 = sys_design.val.hard_version%100;
				tmp_16 = tmp_16/10;
				Flash_Buf[39] |= (tmp_16+0x30);
				
				tmp_16 = sys_design.val.hard_version%10;
				Flash_Buf[40] = (tmp_16+ 0x30);
				Flash_Buf[40] <<=8;
				Flash_Buf[40] += 0;
				Flash_Buf[41] = 0x0;
				
				//当前状态
				Flash_Buf[42] = 1;
							
				
				m = (rx_buffer[3]<<8) + rx_buffer[4]; //寄存器起始地址
				j = rx_buffer[5]*2; //寄存器读取数量
				tx_buffer[0] = 0x03; //源地址
				tx_buffer[1] = rx_buffer[0];
				tx_buffer[2] = cmd;
				tx_buffer[3] = j;
				for(i=0;i<j;i++)
				{
					if(i%2==0)
					{
						tx_buffer[4+i] = (Flash_Buf[m+i/2]>>8);
					}
					else tx_buffer[4+i] = (Flash_Buf[(m+i-(i+1)/2)]);
				}
				crc = CRC16_MODBUS(tx_buffer,4+j,0);
				tx_buffer[4+j] = crc;
				tx_buffer[4+j+1] = crc>>8;
				crc =j+6;
				
			}
			else if(rx_buffer[3]==0xa3)//外部协议读取EEROMM
			{
				if (spi_flash_read_index != spi_flash_write_index)
				{
					SPI_Read_Cont((spi_flash_read_index << 7),ERROR_R_COUNT,(uint8_t *)Flash_Buf);
					for(i=0;i<ERROR_R_COUNT/2;i++)
					{
						tmp_16 = Flash_Buf[i];
						tmp_16 >>= 8;
						tmp_16 |= (Flash_Buf[i]<<8);
						Flash_Buf[i] = tmp_16;
					}
					
					if (spi_flash_write_index > spi_flash_read_index)
					{
						tmp_16 = spi_flash_write_index - spi_flash_read_index;
					}
					else
					{
						tmp_16 = spi_flash_write_index + MAX_ERROR_COUNT - spi_flash_read_index;
					}
					tmp_16 <<= 8;
					tmp_16 |= 37;
					Flash_Buf[0] = tmp_16;
					tmp_16 = 0xAAAA;
					SPI_Page_Program((spi_flash_read_index << 7)+127,1,(uint8_t *)&tmp_16);
					spi_flash_read_index++;
					spi_flash_read_index &= (MAX_ERROR_COUNT - 1 );
					SPI_FLASH_Write_Index();
				}
				else
				{
					memset(Flash_Buf,0,ERROR_R_COUNT);
					Flash_Buf[0] = 37;
				}
					
				m = (rx_buffer[3]<<8) + rx_buffer[4]; //寄存器起始地址
				j = rx_buffer[5]*2; //寄存器读取数量
				tx_buffer[0] = 0x03; //源地址
				tx_buffer[1] = rx_buffer[0];
				tx_buffer[2] = cmd;
				tx_buffer[3] = j;
				for(i=0;i<j;i++)
				{
					if(i%2==0)
					{
						tx_buffer[4+i] = (Flash_Buf[m+i/2]>>8);
					}
					else tx_buffer[4+i] = (Flash_Buf[(m+i-(i+1)/2)]);
				}
				crc = CRC16_MODBUS(tx_buffer,4+j,0);
				tx_buffer[4+j] = crc;
				tx_buffer[4+j+1] = crc>>8;
				crc =j+6;


			}
			break;
		case 0xE0://关机指令
				tmp_16 = rx_buffer[2];
				tmp_16<<=8;
				tmp_16 += rx_buffer[3];
				if(tmp_16 ==0xA200)
				{
					if(rx_buffer[8] == 0x00)
					{
						if(rx_buffer[7] == 0x01)
						{
						//待更新		close++;
						}
						if(rx_buffer[7] == 0x02)
						{
						//待更新			if(close > 0)
							{
						//待更新					close = 0;
								sys_states.state.sys_close = 1;
							}
						}
						tx_buffer[0] = 0x03;
						tx_buffer[1] = 0xE0;
						tx_buffer[2] = 0xA2;
						tx_buffer[3] = 0x00;
						tx_buffer[4] = 0x00;
						tx_buffer[5] = 0x01;
						crc = CRC16_MODBUS(tx_buffer,6,0);
						tx_buffer[6] = crc;
						tx_buffer[7] = crc>>8;
						crc = 8;
					}
					else
					{//强制休眠指令
						if(rx_buffer[8] == 0x01)
						{
						//待更新				sleep++;
						}
						if(rx_buffer[8] == 0x02)
						{
						//待更新			if(sleep > 0)
							{
						//待更新					sleep = 0;
								sys_uarts.flag.uart_sleep_flag = 1;	
							}
						}
						tx_buffer[0] = 0x03;
						tx_buffer[1] = 0xE0;
						tx_buffer[2] = 0xA2;
						tx_buffer[3] = 0x00;
						tx_buffer[4] = 0x00;
						tx_buffer[5] = 0x02;
						crc = CRC16_MODBUS(tx_buffer,6,0);
						tx_buffer[6] = crc;
						tx_buffer[7] = crc>>8;
						crc = 8;
							
					}
			}
			break;
		case 0xE3://191021 chh 测试生产使用，ID写入与读取 01 03 E3 00 10 data CRC CRC
				tmp_16 = rx_buffer[4];
				if(tmp_16 ==0x10)
				{
					if(rx_buffer[3] == 0x00)
					{
					//待更新	
						//_IAP_Erase(1,VER_SN_ADDR,VER_SN_ADDR+1024);
						SNwrite_buffer[0] = rx_buffer[5];
						SNwrite_buffer[1] = rx_buffer[6];
						SNwrite_buffer[2] = rx_buffer[7];
						SNwrite_buffer[3] = rx_buffer[8];
						SNwrite_buffer[4] = rx_buffer[9];
						SNwrite_buffer[5] = rx_buffer[10];
						SNwrite_buffer[6] = rx_buffer[11];
						SNwrite_buffer[7] = rx_buffer[12];
						SNwrite_buffer[8] = rx_buffer[13];
						SNwrite_buffer[9] = rx_buffer[14];
						SNwrite_buffer[10] = rx_buffer[15];
						SNwrite_buffer[11] = rx_buffer[16];
						SNwrite_buffer[12] = rx_buffer[17];
						SNwrite_buffer[13] = rx_buffer[18];
						SNwrite_buffer[14] = rx_buffer[19];
						SNwrite_buffer[15] = rx_buffer[20];
						//_IAP_Flash_WriteW(SNwrite_buffer,VER_SN_ADDR,SN_LENGTH);
						flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_SN_ADDR, SNwrite_buffer, 16);	
						flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_SN_ADDR, sn_buffer, 16);	
						flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_SN_ADDR, tx_buffer+5, 16);	
	
						//for(i=0;i<16;i++)
						//{
						//待更新			tx_buffer[i + 4] = _IAP_Flash_Read(VER_SN_ADDR+(i<<2));	//读取数据记录
						//待更新			sn_buffer[i] = _IAP_Flash_Read(VER_SN_ADDR+(i<<2));	//读取数据记录
						//}
						tx_buffer[0] = 0x03;
						tx_buffer[1] = 0x01;
						tx_buffer[2] = 0xE3;
						tx_buffer[3] = 0x01;
						tx_buffer[4] = 0x10;
						crc =CRC16_MODBUS(tx_buffer,21,0);
						tx_buffer[21] = crc;
						tx_buffer[22] = crc>>8;
						crc = 23;
					}
				}
			break;
		case 0xE4://191021 chh 读取EEROM数据最后内容拦截FLASH存储有问题保护板
				if(rx_buffer[2] == 0x01)
				{
			//待更新			tx_buffer[3] = (sys_states.state.flash_failure >> 8);
			//待更新			tx_buffer[4] = sys_states.state.flash_failure;
					tx_buffer[0] = 0x03;
					tx_buffer[1] = 0xE4;
					tx_buffer[2] = 0x02;
					crc = CRC16_MODBUS(tx_buffer,5,0);
					tx_buffer[5] = crc;
					tx_buffer[6] = crc>>8;
					crc = 7;
				}
			break;
		case 0xE1://检测外挂FALSH是否焊好  03 E1 01 00 CRC CRC
//					if(FLASH_W25QXX_Get_Manufacturer_Id() != 0x02)
//					{
//							tx_buffer[3] = 0x01;
//					}
//					else
//					{
//							tx_buffer[3] = 0x00;
//					}
//					tx_buffer[0] = 0x03;
//					tx_buffer[1] = 0xE1;
//					tx_buffer[2] = 0x01;
//					crc = CRC16_MODBUS(tx_buffer,4,0);
//					tx_buffer[4] = crc;
//					tx_buffer[5] = crc>>8;
//					crc = 6;
			break;
		case 0xE2://通信清除外挂EEROM指令（清除0和4096）
//					sys_w25.val.read_start = 1;
//					tmp_16 = rx_buffer[2];
//					tmp_16<<=8;
//					tmp_16 += rx_buffer[3];
//					if(tmp_16 ==0xA200)
//					{
//							if(rx_buffer[7] == 0x00)
//							{
//									if(rx_buffer[8] == 0x01)
//									{
//											erase_eerom++;
//									}
//									if(rx_buffer[8] == 0x02)
//									{
//											if(erase_eerom > 0)
//											{
//													erase_eerom = 0;
//													FLASH_W25QXX_Sector_Erase_4kb(0);
//													for(delay_i = 0;delay_i < 20;delay_i++)
//													{
//															delay_ms(5);
//															if(FLASH_W25QXX_Get_Busy_Status() == 0)
//															{
//																	break;
//															}
//													}
//													FLASH_W25QXX_Sector_Erase_4kb(4096);
//													for(delay_i = 0;delay_i < 20;delay_i++)
//													{
//															delay_ms(5);
//															if(FLASH_W25QXX_Get_Busy_Status() == 0)
//															{
//																	break;
//															}
//													}
//													FLASH_W25QXX_Sector_Erase_4kb(8192);
//													for(delay_i = 0;delay_i < 20;delay_i++)
//													{
//															delay_ms(5);
//															if(FLASH_W25QXX_Get_Busy_Status() == 0)
//															{
//																	break;
//															}
//													}
//													W25_Init();
//													//W25_Write_headData();
//													//W25_Write_Data_4K();
//													
//											}
//									}
//									tx_buffer[0] = 0x03;
//									tx_buffer[1] = 0xE2;
//									tx_buffer[2] = 0xA2;
//									tx_buffer[3] = 0x00;
//									tx_buffer[4] = 0x00;
//									tx_buffer[5] = 0x01;
//									crc = CRC16_MODBUS(tx_buffer,6,0);
//									tx_buffer[6] = crc;
//									tx_buffer[7] = crc>>8;
//									crc = 8;
//						}
//						else
//						{
								if(rx_buffer[7] == 0x01 && rx_buffer[8] == 0x00 )
								{
								//待更新			erase_eerom++;
								}
								if(rx_buffer[7] == 0x02 && rx_buffer[8] == 0x00)
								{
								//待更新			if(erase_eerom > 0)
										{
								//待更新					erase_eerom = 0;
													__disable_irq();
								//待更新				AP_Flash_Write(0,FLASH_ERR_ADDR);
													__enable_irq();
										}
										
										//待更新	
										
										//if(protects.protect.temp_err_flag ==1)
										//{
												//FALSH_error = FALSH_error & 0xFE;
												//FALSH_error_tmp = FALSH_error;
												//protects.protect.temp_err_flag = 0;
										//}
										//if(protects.protect.chg_mos_err_flag ==1)
										//{
												//FALSH_error = FALSH_error & 0xEF;
												//FALSH_error_tmp = FALSH_error;
												//protects.protect.chg_mos_err_flag = 0;
										//}
										//if(protects.protect.dch_mos_err_flag ==1)
										//{
												//FALSH_error = FALSH_error & 0xF7;
												//FALSH_error_tmp = FALSH_error;
												//protects.protect.dch_mos_err_flag = 0;
										//}
										//if(protects.protect.cell_dif_volt_flag ==1)
										//{
												//FALSH_error = FALSH_error & 0xFD;
												//FALSH_error_tmp = FALSH_error;
												//protects.protect.cell_dif_volt_flag = 0;
										//}
										//if(protects.protect.cell_low_1v5_flag ==1)
										//{
												//FALSH_error = FALSH_error & 0xFB;
												//FALSH_error_tmp = FALSH_error;
												//protects.protect.cell_low_1v5_flag = 0;
										//}
								}
								tx_buffer[0] = 0x03;
								tx_buffer[1] = 0xE2;
								tx_buffer[2] = 0xA2;
								tx_buffer[3] = 0x00;
								tx_buffer[4] = 0x01;
								tx_buffer[5] = 0x00;
								crc = CRC16_MODBUS(tx_buffer,6,0);
								tx_buffer[6] = crc;
								tx_buffer[7] = crc>>8;
								crc = 8;
//						}
//								
//				}
//				sys_w25.val.read_start = 0;
			break;
		case WRITE_R_FUNTION:	// 执行到这里 表示 写寄存器解析完成
			sys_uarts.flag.uart_update_ok_flag =0;
			tmp_16 = rx_buffer[3];
			tmp_16<<=8;
			tmp_16 += rx_buffer[4];
			
			if(tmp_16 ==0xA200)
			{
				if(rx_buffer[7] == 0x00) 
				{
						sys_flags.flag.sys_comm_open_dch_flag =0;
						sys_flags.flag.open_dch_30s_end_flag =1;
				}
				else
				{//191127 chh 增加30S通信强制开输出  无保护时通信开输出转入放电模式   保护时通信开输出转入30S预放电模式
					//if((sys_flags.flag.sys_id_connect_flag==1) && (protects.protect.forbidden_output == 0) && sys_flags.flag.open_dch_30s_end_flag == 0)
					
					//在老化模式 则退出老化模式
					flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
					if (tmp_16 == 0x0000)
					{
						tmp_16 = 0xffff;
						flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
					}
					//屏蔽了SOC保护 则开启SOC保护
					flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_NO_SOCLP_ADDR,(uint8_t *)&tmp_16, 2);
					if (tmp_16 == 0x0000)
					{
						tmp_16 = 0xffff;
						flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_NO_SOCLP_ADDR,(uint8_t *)&tmp_16, 2);
					}					
					
					if((sys_flags.flag.sys_id_connect_flag==1))
					{
						sys_flags.flag.sys_comm_open_dch_flag =1;
						sys_flags.flag.adapter_connect_flag =0;
						sys_states.state.sys_chg_ch_flag =0;
						sys_flags.flag.open_dch_30s_end_flag =1;
						if(protects.protect.dch_empty_flag ==1)
						{
							protects.protect.forbidden_output = 1;
							protects.protect.dch_empty_flag =0;
							sys_flags.flag.open_dch_30s_end_flag =0; 
							stick_30s_cnt =0;
							sys_flags.flag.sys_comm_open_dch_flag =0;
						}
						
						if ((protects2.protect.soc_lp_flag ==1))
						{
							protects2.protect.soc_lp_flag = 0;
							sys_flags.flag.open_dch_30s_end_flag =0;
							stick_30s_cnt =0;
							soc_lp_cnt = 0;
							sys_flags.flag.sys_comm_open_dch_flag =0;
						}
						
						if ((protects.protect.dch_lvp_flag == 1))
						{
							protects.protect.dch_lvp_flag = 0;
							sys_flags.flag.open_dch_30s_end_flag =0;
							stick_30s_cnt =0;
							bat_uvp_cnt = 0;
							sys_flags.flag.sys_comm_open_dch_flag =0;
						}						
											
						//191127 chh 增加30S强制开输出 过流类保护清除保护标志位维持30S放电  放电低温保护维持30S放电，不必清除，即使触发温度保护不执行关管保护动作
						//if(protects.protect.dch_ocp_flag ==1)
						//{
							//protects.protect.dch_ocp_flag = 0;
							//sys_flags.flag.open_dch_30s_end_flag =0; 
							//stick_30s_cnt =0;
							//sys_flags.flag.sys_comm_open_dch_flag =0;
						//}
						//if(protects.protect.chg_ocp_flag ==1)
						//{
							//protects.protect.chg_ocp_flag = 0;
							//sys_flags.flag.open_dch_30s_end_flag =0; 
							//stick_30s_cnt =0;
							//sys_flags.flag.sys_comm_open_dch_flag =0;
						//}
						//191205 chh  由于在中断中清除bq寄存器会导致失败，故在此置标志位在主循环中执行清除bq寄存器
						if(protects.protect.dch_scd_flag ==1)
						{
						//待更新			BQ_clear = 1;
													
						}
						if(protects.protect.dch_ocp2_flag ==1)
						{
						//待更新			BQ_clear = 2;
						}
										//191202 chh 增加30S强制开输出 过流类保护清除保护标志位维持30S放电  温度、压差保护维持30S放电，清除标志位，开管后重新置位，但即使触发温度保护不执行关管保护动作
						if((protects2.protect.dch_ltp2_flag == 1)||(protects2.protect.dch_ltp_flag == 1)||(protects2.protect.dch_otp_flag == 1)||
							(protects2.protect.dch_otp2_flag == 1)||(protects2.protect.chg_ltp_flag == 1)||(protects2.protect.chg_ltp2_flag == 1)|| 
							(protects2.protect.chg_otp_flag == 1)||(protects2.protect.chg_otp2_flag == 1)||(protects.protect.cell_dif_volt_flag == 1))
						{
							if(protects2.protect.dch_ltp_flag == 1)
							{
									//protects2.protect.dch_ltp_flag = 0;
							//待更新			dch_ltp_cnt = 0;
							}
							if(protects2.protect.dch_ltp2_flag == 1)
							{
							//待更新			dch_sec_ltp_cnt = 0;
									//protects2.protect.dch_ltp2_flag = 0;
							}
							if(protects2.protect.dch_otp_flag == 1)
							{
							//待更新			dch_otp_cnt = 0;
									//protects2.protect.dch_otp_flag = 0;
							}
							if(protects2.protect.dch_otp2_flag == 1)
							{
							//待更新			dch_sec_otp_cnt = 0;
									//protects2.protect.dch_otp2_flag = 0;
							}
							if(protects2.protect.chg_ltp_flag == 1)
							{
							//待更新			chg_ltp_cnt = 0;
									//protects2.protect.chg_ltp_flag = 0;
							}
							if(protects2.protect.chg_ltp2_flag == 1)
							{
							//待更新			chg_sec_ltp_cnt = 0;
									//protects2.protect.chg_ltp2_flag = 0;
							}
							if(protects2.protect.chg_otp_flag == 1)
							{
							//待更新			chg_otp_cnt = 0;
									//protects2.protect.chg_otp_flag = 0;
							}
							if(protects2.protect.chg_otp2_flag == 1)
							{
							//待更新			chg_sec_otp_cnt = 0;
									//protects2.protect.chg_ltp2_flag = 0;
							}
							if(protects.protect.cell_dif_volt_flag == 1)
							{
							//待更新			cell_sub_cnt = 0;
									//protects.protect.cell_dif_volt_flag = 0;
							}
								sys_flags.flag.open_dch_30s_end_flag =0; 
								stick_30s_cnt =0;
								sys_flags.flag.sys_comm_open_dch_flag =0;
								protects.protect.forbidden_output = 1;//Chh 20200214
						}
					}
				}
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xA2;
				tx_buffer[4] = 0x00;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
			}
			
			if(tmp_16 ==0xA235)
			{
				tmp = rx_buffer[6];
				tmp <<=8;
				tmp |= rx_buffer[7];
				//tmp = tmp*1000/1532;
				if(tmp >30000)
				{
			//待更新			vbat_empty_val = (u16)tmp;
			//待更新			vbat_empty_clr_val = vbat_empty_val+1000;
			//待更新			AP_Flash_Write(tmp,FLASH_EMPTY_ADDR);
					tx_buffer[0] = BMS_ADDRESS;
					tx_buffer[1] = 0x01;
					tx_buffer[2] = WRITE_R_FUNTION;
					tx_buffer[3] = 0xA2;
					tx_buffer[4] = 0x00;
					tx_buffer[5] = 0x01;
					crc = CRC16_MODBUS(tx_buffer,6,0);
					tx_buffer[6] = crc;
					tx_buffer[7] = crc>>8;
					crc = 8;
				}
			}
			
			if (tmp_16 == 0xA248)
			{
				tmp = *(rx_buffer+6) << 24 | *(rx_buffer+7) << 16 | *(rx_buffer+8) << 8 | *(rx_buffer+9) ;
				to_tm(tmp,&systmtime);
				Struct_Time_To_Buff_Time(&systmtime,&sd2058_val);
				SD2058_Rtc_Write_Time(&sd2058_val);
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xA2;
				tx_buffer[4] = 0x48;
				tx_buffer[5] = 0x02;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
			}
			
			if (tmp_16 ==0xAAAA) //shutdown
			{
				// 临时测试  正式程序删除
				sys_flags.flag.sys_close_flag = 1;
				sys_close_count = GC_DELAY_30S;
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xAA;
				tx_buffer[4] = 0xAA;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;			
			}
			
			if (tmp_16 ==0xAAAB) //电流校准
			{
				

				// 校准测试
				
				sys_flags.flag.com_current_cal = 1 ;	
				
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xAA;
				tx_buffer[4] = 0xAB;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
				
			}			
			
			if (tmp_16 ==0xAAAC) //老化模式
			{
				if(rx_buffer[6] == 0x00 && rx_buffer[7] == 0x00)
				{
					rx_buffer[6] = 0x00;
					rx_buffer[7] = 0x00;
					flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,rx_buffer+6, 2);
				}
				else if (rx_buffer[6] == 0xFF && rx_buffer[7] == 0xFF)
				{
					rx_buffer[6] = 0xff;
					rx_buffer[7] = 0xff;
					flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,rx_buffer+6, 2);
				}
				
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xAA;
				tx_buffer[4] = 0xAC;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
				
			}
			
			if (tmp_16 ==0xAAAD) // 均衡模式
			{
				AFE.BALH = 0xff;
				MTPWrite(MTP_BALANCEH, 1, &AFE.BALH);
				delay_ms(5);
				AFE.BALL = 0xff;
				MTPWrite(MTP_BALANCEL, 1, &AFE.BALL);

				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xAA;
				tx_buffer[4] = 0xAD;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
				
			}
			
			if (tmp_16 ==0xAAAE) //无SOC保护
			{
				if(rx_buffer[6] == 0x00 && rx_buffer[7] == 0x00)
				{
					rx_buffer[6] = 0x00;
					rx_buffer[7] = 0x00;
					flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_NO_SOCLP_ADDR,rx_buffer+6, 2);
				}
				else if (rx_buffer[6] == 0xFF && rx_buffer[7] == 0xFF)
				{
					rx_buffer[6] = 0xff;
					rx_buffer[7] = 0xff;
					flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_NO_SOCLP_ADDR,rx_buffer+6, 2);
				}
				
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xAA;
				tx_buffer[4] = 0xAE;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
				
			}		
			
			if (tmp_16 ==0xAAAF) //清故障
			{
				// 临时测试  正式程序删除
				
				Flash_error = 0;
				Flash_error_tmp = Flash_error;
				flash_save_data(VER_FLASH_ADDR+FLASH_ERROR_ADDR,Flash_error_write_index,Flash_error);
				Flash_error_write_index++;
				Flash_error_write_index &= 0x07;
				
				protects.protect.ntc_open_flag = 0;
				protects.protect.cell_dif_volt_flag = 0;
				protects.protect.cell_low_2v_flag = 0;
				protects.protect.dch_mos_err_flag = 0;
				protects.protect.chg_mos_err_flag = 0;
				protects.protect.ntc_short_flag = 0;


				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xAA;
				tx_buffer[4] = 0xAF;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
			}
			
			
			if (tmp_16 == 0xCCCC) //读取flash数据
			{
				tmp = rx_buffer[6]; //读取的条数 每条128字节
				tmp <<=8;
				tmp |= rx_buffer[7];

				spi_read_count = tmp;
				// 校准测试
				
				sys_flags.flag.read_spi_flash_flag = 1 ;
				
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = 0x01;
				tx_buffer[2] = WRITE_R_FUNTION;
				tx_buffer[3] = 0xCC;
				tx_buffer[4] = 0xCC;
				tx_buffer[5] = 0x01;
				crc = CRC16_MODBUS(tx_buffer,6,0);
				tx_buffer[6] = crc;
				tx_buffer[7] = crc>>8;
				crc = 8;
				
			}	
						
			break;
					
		case 0xF0: //读取EEROM数据		
//						tx_buffer[0] = 0x03;
//						tx_buffer[1] = 0xF0;
//						tx_buffer[2] = 0x00;

//						switch(rx_buffer[4])
//						{
//								case 0x00:
//										W25_Write_headData();
//										W25_Read_Data(0);
//										max_readeerom_len = read_eerom[2];
//										max_readeerom_len <<= 8;
//										max_readeerom_len |= read_eerom[3];
//										max_readeerom_len <<= 8;
//										max_readeerom_len |= read_eerom[4];
//										max_readeerom_len <<= 8;
//										max_readeerom_len |= read_eerom[5];
//										if(read_eerom[10] == 1)
//										{
//												ini_addr = read_eerom[6];
//												ini_addr <<= 8;
//												ini_addr |= read_eerom[7];
//												ini_addr <<= 8;
//												ini_addr |= read_eerom[8];
//												ini_addr <<= 8;
//												ini_addr |= read_eerom[9];
//												max_readeerom_len = (max_readeerom_len - (32 -((ini_addr%4096)/128)));//191015  chh 计算在数据写满时,有可能会出现
//																																															//没写满4K就读取的情况，此时要计算4K总未写的条数扣除
//												ini_addr = (((ini_addr/4096)+1)*4096);//191015 chh 数据写满后，读取的数据要从下一个4K读取	
//										}
//										else
//										{
//												ini_addr = 8192;
//										}
//										sys_w25.val.read_start = 1;
//										sys_w25.val.readpage_cnt = 0;
//										sys_w25.val.read_cnt = 0;	
//										sys_w25.val.tx_id = 0;
//									break;
//								case 0x01:
//									break;
//								case 0x02:
//									sys_w25.val.read_cnt++;
//									if(sys_w25.val.read_cnt >= 2)
//									{
//											sys_w25.val.read_cnt = 0;
//											sys_w25.val.readpage_cnt++;
//									}
//									break;
//								default:
//									return;
//									//break;
//						}
//						if((ini_addr + (sys_w25.val.read_cnt<<7) + (sys_w25.val.readpage_cnt<<8)) == MAX_ADDR )
//						{
//								ini_addr = 8192;
//								sys_w25.val.readpage_cnt = 0;
//								sys_w25.val.read_cnt = 0;	
//						}
//						for(i = 0;i < rx_buffer[5];i++)
//						{
//								W25_Read_Data(ini_addr + (sys_w25.val.read_cnt<<7) + (sys_w25.val.readpage_cnt<<8));
//								for(delay_readeerom = 0;delay_readeerom < 20;delay_readeerom++)
//								{
//										delay_ms(5);
//										if(FLASH_W25QXX_Get_Busy_Status() == 0)
//										{
//												break;
//										}
//								}
//								//数据CRC校验，若校验不过返回0
//								crc = TX_CheckSum(read_eerom,79,0);
//								if(((u8)(crc>>8) == read_eerom[79]) && ((u8)(crc) == read_eerom[80]))
//								{
//										for(j = 0;j < 79;j++)
//										{
//												tx_buffer[8 + j] = read_eerom[j + 2];
//										}
//								}
//								else
//								{
//										for(j = 0;j < 79;j++)
//										{
//												tx_buffer[4 + j] = 0;
//										}
//								}
//								sys_w25.val.tx_id += 1;
//								tx_buffer[4] = (sys_w25.val.tx_id>>24);
//								tx_buffer[5] = (sys_w25.val.tx_id>>16);
//								tx_buffer[6] = (sys_w25.val.tx_id>>8);
//								tx_buffer[7] = sys_w25.val.tx_id;
//						}
//						if(sys_w25.val.read_start == 1)
//						{
//								tx_buffer[2] = 0x00;
//						}
//						else
//						{
//								tx_buffer[2] = 0x01;
//						}
//						if(max_readeerom_len == sys_w25.val.tx_id)//191015 chh 读取EEROM增加02结束指令
//						{
//								tx_buffer[2] = 0x02;
//						}
//						tx_buffer[3] = 84;
//						crc =RX_CheckSum(tx_buffer,86);		
//						tx_buffer[86] = crc;
//						tx_buffer[87] = (crc >> 8);
//						crc = 88;
			break;
		case UPDATE_FUNTION: //到这里代表解析固件升级指令成功
			// 升级握手  2是类别 3,4是长度5开始才是数据 
			tx_buffer[0] = BMS_ADDRESS;
			tx_buffer[1] = rx_buffer[0];
			tx_buffer[2] = cmd;
			tx_buffer[3] = rx_buffer[3]+1;
			if(rx_buffer[3] == 0x01)//请求升级
			{
				j =2;
				tx_buffer[4] = 0;
				tx_buffer[5] = j;
				if(sys_cap.val.re_cap_rate > 10)
				{
					tx_buffer[6] = 0;
					tx_buffer[7] = 0;
					boot_state =1;  //进入OTA成功
					write_index =0;
					write_size =0;
					//待更新	sys_w25.val.read_start = 1;//191022 chh 增加在升级过程中将sys_w25.val.read_start标志位置1，升级不允许写EEROM，升级过程有通信标志位不会被清0
					sys_uarts.flag.uart_flash_earse_flag =0;
					sys_uarts.flag.cmd_reset_flag =0;
					sys_uarts.flag.uart_update_ok_flag =0;
					head_check = 0;
					write_crc =0;
				}
				else
				{
					tx_buffer[6] = 1;
					tx_buffer[7] = 1;
					boot_state =10;  //进入OTA失败 电量过低
				}
			}
			else if(rx_buffer[3] == 0x13)//尝试升级
			{
				if(rx_buffer[18] == 4)
				{
							
					crc_MODBUS = 0xFFFF;//计算CRC
					crc_i = 0;
									
					rev_version = rx_buffer[19]*100;
					rev_version += rx_buffer[20]*10;
					rev_version += rx_buffer[21];
								
					rev_packcount = rx_buffer[22];
					rev_packcount<<=8;
					rev_packcount |= rx_buffer[23];
					rev_packcount<<=8;
					rev_packcount |= rx_buffer[24];
					rev_packcount<<=8;
					rev_packcount |= rx_buffer[25];
					
								
					j =4;
					tx_buffer[4] = 0;
					tx_buffer[5] = j;
					if(sys_states.state.update_goto_reset_flag == 1)//191215 chh 修改升级过程：在数据传输完毕后接收到13指令后才能够执行跳转覆盖动作
					{		
						sys_states.state.update_goto_reset_flag = 0;
						sys_uarts.flag.cmd_reset_flag =1;
						tx_buffer[6] = 0x02;
					}
					else
					{	
						if(boot_state ==1)
						{
							if(rev_version != sys_design.val.soft_version)//191219 chh 应客户要求不同版本才可升级
							{
								tx_buffer[6] = 0x06;
							}
							else
							{
								tx_buffer[6] = 0x00;
							}
						}
						else
						{
							tx_buffer[6] = 0x08;
						}
					}
					tx_buffer[7] = 0x01; //64字节
									
					tx_buffer[8] = (u8)(write_index>>8);	
					tx_buffer[9] = (u8)(write_index);	
					if(tx_buffer[6] == 0x06 )
					{
						boot_state =2; // 固件需要升级
					}
					else
					{
						boot_state =1; //固件不需要升级  停留在bootloader
					}
				}
				else
				{
								
					j =4;
					tx_buffer[4] = 0;
					tx_buffer[5] = j;
					tx_buffer[6] = 0x04;
					tx_buffer[7] = 0x01; //64字节
									
					tx_buffer[8] = (u8)(write_index>>8);	
					tx_buffer[9] = (u8)(write_index);	
				}
			}
			else if(rx_buffer[3] == 0x15)//发包
			{
				//待更新	Update_prepare();//升级过程关闭定时器
				if(boot_state ==2)
				{
					tmp_16 = rx_buffer[10];
					tmp_16 <<=8;
					tmp_16 |= rx_buffer[11];
					if(tmp_16 == 0)
					{//191219 chh 应客户要求增加包头校验   包头：DD3616 + 软件3 + 硬件3 + bin文件长度4 + bin文件CRC2 + 包头CRC2 + 0xFF
						if((rx_buffer[16] == 'D')&& (rx_buffer[17] == 'D')&&(rx_buffer[18] == '4')&&(rx_buffer[19] == '8')&&
						(rx_buffer[20] == '2')&&(rx_buffer[21] == '3'))
						{
							head_datelen = rx_buffer[28];
							head_datelen<<=8;
							head_datelen |= rx_buffer[29];
							head_datelen<<=8;
							head_datelen |= rx_buffer[30];
							head_datelen<<=8;
							head_datelen |= rx_buffer[31];
							if((head_datelen + 64) == rev_packcount)
							{
								rev_crc16_firmware = rx_buffer[32];
								rev_crc16_firmware <<=8;
								rev_crc16_firmware |= rx_buffer[33];
								write_crc = CRC16_MODBUS(rx_buffer,18,16);
								if(((u8)(write_crc>>8) == rx_buffer[34])&&((u8)write_crc == rx_buffer[35]))
								{
									head_check = 1; //包头检查通过
									boot_state = 2;
									write_index = 1;
									rev_packcount -= 64;//接收长度要减去包头长度才能计算总CRC
								}
								else
								{
									boot_state = 13;//包头校验不过返回13
									head_check = 0;
								}
							}
							else
							{
								boot_state = 13;
								head_check = 0;
							}
						}
						else
						{
							boot_state = 13;
							head_check = 0;
						}
					}
					else
					{
						if(head_check == 1)
						{
							if(tmp_16 == write_index)
							{
								tmp = rx_buffer[4];
								tmp <<=8;
								tmp |= rx_buffer[5];
								if(tmp >10)
								{
									tmp = tmp-10;
									write_crc = CRC16_MODBUS(rx_buffer,(u16)tmp,16);
									if(((u8)(write_crc>>8) == rx_buffer[14])&&((u8)write_crc == rx_buffer[15]))
									{
										data_max_len = rx_buffer[12];
										data_max_len<<=8;
										data_max_len |= rx_buffer[13];
															
										if(sys_uarts.flag.uart_flash_earse_flag ==0)
										{
											sys_uarts.flag.uart_flash_earse_flag =1;
											sys_uarts.flag.update_begin_flag = 1;
											//待更新			_IAP_Erase(0,0x008C00,0x00F800);
											write_addr = 0x14000;
										}
										for(i=0;i<tmp;i++)
										{
											tx_buffer[i] = rx_buffer[i+16];
										}
										//_IAP_Flash_Write(tx_buffer,write_addr,tmp);
										flash_write(&FLASH_0, write_addr, tx_buffer, tmp);
															
										read_addr = write_addr;
										boot_state = 2;
															
										for( i=0;i<tmp;i++)
										{
											//flag_tmp =_IAP_Flash_Read(read_addr);
											flash_read(&FLASH_0, read_addr, &flag_tmp, 1);
											if(tx_buffer[i] == (u8)flag_tmp)
											{
												read_addr+=1;
												if(	crc_i < rev_packcount)
												{
													crc_buffer[0] = flag_tmp;
													crc_MODBUS = CRC16_MODBUS_ONE(crc_buffer,1,0,crc_i,crc_MODBUS);
													crc_i++;
												}
												continue;
											}
											else
											{
												boot_state = 12; //写flash校验出错
												break;
											}
										}
										if(	boot_state ==2)
										{
											write_addr+=tmp;
											write_index++;
											if(write_index == data_max_len)
											{//191011 chh 增加升级总数据校验，校验不过返回5
												if(rev_crc16_firmware == crc_MODBUS)
												{
													//sys_uarts.flag.update_goto_reset_flag = 1;
													sys_states.state.update_goto_reset_flag = 1;
													//烧写完成
												}
												else
												{
													boot_state = 5;   //升级总数据校验，校验不过返回5
												}
											}
										}
									}
									else
									{
										boot_state = 11; //数据块检验不过
									}
								}
								else
								{
									boot_state = 9; //数据长度不符合
								}
							}
							else
							{
								boot_state = 8;//包序号不对
							}
						}
						else
						{
							boot_state = 13; //包头检验不过
							head_check = 0;
						}
					}
				}
				else
				{
					boot_state = 7; //在不需要更新的状态下 接收到包
				}
				j = 19;
				tx_buffer[0] = BMS_ADDRESS;
				tx_buffer[1] = rx_buffer[0];
				tx_buffer[2] = cmd;
				tx_buffer[3] = rx_buffer[3]+1;
				tx_buffer[4] = 0;
				tx_buffer[5] = j;
				if(boot_state ==2)
				{
					tx_buffer[6] = 0;
				}
				else
				{
					tx_buffer[6] = boot_state;
				}
				boot_state =2;
				tx_buffer[7] = 0;
				tx_buffer[8] = 0;
				tx_buffer[9] = 0x04;
				tx_buffer[10] = (u8)(write_index>>8);
				tx_buffer[11] = (u8)write_index;
				tx_buffer[12] = 'S';
				tx_buffer[13] = 'C';
				tx_buffer[14] = 'U';
				tx_buffer[15] = 'D';
				for(i =0;i<6;i++)
				{
					tx_buffer[16+i] = 0x30;
				}
				tmp_16 = sys_design.val.soft_version/100;
				tx_buffer[22] = (u8)tmp_16;
				tmp_16 = sys_design.val.soft_version%100;
				tmp_16 = tmp_16/10;
				tx_buffer[23]  =  tmp_16;
				tmp_16 = sys_design.val.soft_version%10;
				tx_buffer[24] = tmp_16;
			}
			else
			{
				j =2;
				tx_buffer[4] = 0;
				tx_buffer[5] = j;
							
				tx_buffer[6] = 1;	
				tx_buffer[7] = 1;								
			}
			crc = CRC16_MODBUS(tx_buffer,j+6,0);
			tx_buffer[6+j] = crc;
			tx_buffer[7+j] = crc>>8;
			crc =j+8;
			break;
			
		case 0xF3://切换为工程模式
			tx_buffer[0] = 0x47;
			tx_buffer[1] = 0x16;
			tx_buffer[2] = rw;
			tx_buffer[3] = cmd;
			tx_buffer[4] = 4;
			tx_buffer[5] = 1;
			tx_buffer[6] = 0;
			tx_buffer[7] = 0;
			tx_buffer[8] = 1;
			for(i=0;i<9;i++)
			{
				check_sum+=tx_buffer[i];
			}
			tx_buffer[9] = check_sum;
			crc = 10;	
			sys_flags.flag.sys_recovery_flag =1;
			break;
	}

	if(crc >0)
	{
		CAN_Send(0x06,CAN_FMT_EXTID,tx_buffer,crc);		 //暂定发送扩展ID 
/*
			EN485_TX();
			DIS485_RX();
			delay_ms(5);
				
			for(i=0;i<crc;i++)
			{
					
				USART_SendData(HT_UART1, tx_buffer[i]);
				while (USART_GetFlagStatus(HT_UART1, USART_FLAG_TXC) == RESET);
			}
			delay_ms(5);
			EN485_RX();
			DIS485_TX();		
*/
	}	
	
	if(sys_uarts.flag.cmd_reset_flag ==1)
	{
		SPI_FLASH_errorstate();//复位记录
		sys_uarts.flag.cmd_reset_flag =0;
		delay_ms(10);
		tx_buffer[0] = COVER_APP;
		//写FLASH,复位
		//AP_Flash_Write(0x61,1);
		flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_BOOTSTATUS_ADDR, tx_buffer, 1);
		delay_ms(5);
		
		
		//存储SOC 充电 循环次数
	
		flash_save_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,chg_cap_cnt_write_index,sys_cap.val.chg_cap_cnt);
		chg_cap_cnt_write_index++;
		chg_cap_cnt_write_index &= 0x07;
		
		flash_save_data(VER_FLASH_ADDR+FLASH_CYCLES_ADDR,bat_cycle_cnt_write_index,sys_cap.val.bat_cycle_cnt);
		bat_cycle_cnt_write_index++;
		bat_cycle_cnt_write_index &= 0x07;
		
		flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
		re_cap_rate_write_index++;
		re_cap_rate_write_index &= 0x07;
		
		
		
		
		NVIC_SystemReset();
	}
	
}
