/*
 * CAN_Info.h
 *
 * Created: 2020/3/3 10:38:25
 *  Author: fjiangqing
 */


#ifndef CAN_INFO_H_
#define CAN_INFO_H_

#include "stdint.h"
#include "driver_init.h"

#ifdef CAN_SYS
#define CAN_EXT
#else
#define CAN_EXT                         extern
#endif // CAN_SYS

#define CAN_RX_MASSAGE_BUFFER_LENGTH    20			//TODO:������ջ�������С������ͨѶЭ����С�
#define CAN_RX_DATE_LENGTH_BYTE         8			//TODO:CAN���յ�֡���ݳ��ȣ�Ĭ��Ϊ8byte��ATMEL C21����������64byte

CAN_EXT struct can_message CAN_Rx_Message_Buffer[CAN_RX_MASSAGE_BUFFER_LENGTH];
CAN_EXT uint8_t            CAN_Rx_Data_Buffer[CAN_RX_MASSAGE_BUFFER_LENGTH * CAN_RX_DATE_LENGTH_BYTE];

CAN_EXT uint16_t CAN_Rx_Buffer_Read_Index;
CAN_EXT uint16_t CAN_Rx_Buffer_Write_Index;
CAN_EXT uint16_t CAN_Tx_Error_Cnt;
#endif /* CAN_INFO_H_ */
