/*
 * W25Q32.h
 *
 * Created: 2020/5/27 9:38:06
 *  Author: 20200504602
 */ 


#ifndef W25Q32_H_
#define W25Q32_H_

#include "main.h"

#define CE_SetLow()		 gpio_set_pin_level(CS,false);
#define CE_SetHigh()     gpio_set_pin_level(CS,true);


void SPI_Init(void);
unsigned char SPI_Read_Status_Register();
unsigned char SPI_Read_Configuration_Register();
void SPI_WREN();
void SPI_WRDI();
void SPI_Jedec_ID_Read(char *Manufacturer_Id, char *Device_Type, char *Device_Id);
unsigned char SPI_Read(unsigned long Dst);
void SPI_Read_Cont(unsigned long Dst,unsigned long no_bytes, unsigned char *read_data);
unsigned char SPI_HighSpeed_Read(unsigned long Dst);
void SPI_HighSpeed_Read_Cont(unsigned long Dst,unsigned long no_bytes, unsigned int *read_data);;
void SPI_Page_Program(unsigned long Dst,unsigned long no_bytes, unsigned char *Prog_data);
void SPI_Chip_Erase();
void SPI_Sector_Erase(unsigned long Dst);
void SPI_Block_Erase(unsigned long Dst);
void SPI_NoOp();
void SPI_ResetEn();
void SPI_Reset();
void SPI_Write_Suspend();
void SPI_Write_Resume();
void SPI_Write_Status_Register(unsigned int data1, unsigned char datalen);
void SPI_ReadSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length);
void SPI_ProgSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length);
void SPI_LockSID();
void SPI_ReadBlockProtection(unsigned int *block_protection_data);
void SPI_WriteBlockProtection(unsigned int *block_protection_data);
void SPI_Global_Block_Protection_Unlock();
void SPI_LockBlockProtection();
void SPI_NonVolWriteLockProtection(unsigned int *block_protection_data);
void SPI_Wait_Busy();
unsigned char SPI_SFDP_Read(unsigned long Dst);



#endif /* W25Q32_H_ */