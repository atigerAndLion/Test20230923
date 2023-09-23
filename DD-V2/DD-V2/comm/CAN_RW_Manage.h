/*
 * CAN_RW_Manage.h
 *
 * Created: 2020/4/16 15:21:04
 *  Author: fjiangqing
 */


#ifndef CAN_RW_MANAGE_H_
#define CAN_RW_MANAGE_H_

#include "stdint.h"
#include "driver_init.h"
#include "CAN_Info.h"
#include "mydef.h"
#include "main.h"
#include "soft_soc.h"
#include "user_bms.h"

typedef union
{
	struct
	{
		uint32_t frame_num									 : 5;
		uint32_t pack_queue_num	                             : 3;
		uint32_t target_addr		                         : 6;
		uint32_t n2                       					 : 2;
		uint32_t protocol_num               		         : 2;
		uint32_t source_addr                      	         : 6;
		uint32_t n1				  			  	 			 : 2;
		uint32_t priority	    	 	       			     : 3;
		uint32_t n3			    	 	       			     : 3;
	}bit;
	uint32_t VAL;
}DD_CAN_EXTEN_ID;


#define CONF_CAN0_RX_STDID_ID_FILTER_NUM       1 /* Range: 1..64 */ //TODO:���ݽ��ձ�׼֡ID���޸�
#define CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM    1 /* Range: 1..64 */ //TODO:���ݽ�����չ֡ID���޸�
#define CAN_RX_STDID_CMD                       0x0001               //TODO:����Ĭ��ֻ�������ID
#define CAN_RX_EXTENDED_CMD                    0x00000300           //ֻ����Ŀ�ĵ�ַΪ3��ID
#define CAN_TX_STDID_CMD					   0x0001
#define CAN_TX_EXTENDED_CMD                    0x00000001

#define SEND_WAIT_TIMER                        1000                 //TODO:can ���Ͳ�������������ʹ�����1ms�ȴ�������ʵ���豸������

#define BMS_ADDRESS                            0x03
#define READ_R_FUNTION                         0x03
#define WRITE_R_FUNTION                        0x10
#define UPDATE_FUNTION                         0x45

#define MAX_DCH_CUR     30000
#define MAX_CHG_VOL		58400
#define MAX_CHG_CUR     13000



void CAN_RW_Init(void);
void CAN_tx_callback(struct can_async_descriptor *const descr);
void CAN_rx_callback(struct can_async_descriptor *const descr);
void CAN_error_callback(struct can_async_descriptor *const descr, enum can_async_interrupt_type type);

uint8_t CAN_Send(uint8_t priority, enum can_format format, uint8_t *send_data_addr, uint16_t length);
uint8_t CAN_Send_Wait(uint16_t timer);
extern uint8_t Send_Succeed ; //���ݷ��ͳɹ���־��

#define XMODEM_BUFLEN	512
void buff_init(void);
void write_byte(uint8_t byte);
void read_bytes(uint8_t * buffer, uint32_t byteCount);
uint8_t buff_empty(void);

void Comm_Proc(void);
void Uart_DIDI_Rx(void);
void Uart_DIDI_Tx(u8 rw,u8 cmd);

//CRC���㺯��
unsigned short CRC16_MODBUS(volatile unsigned char *puchMsg, unsigned int usDataLen,u8 index);
u8  InvertUint8(unsigned char dBuf,unsigned char srcBuf);
u16  InvertUint16(unsigned short dBuf,unsigned short srcBuf);
unsigned short RX_CheckSum(volatile unsigned char *buf, u8 len); //bufΪ���飬lenΪ���鳤��

#endif /* CAN_RW_MANAGE_H_ */
