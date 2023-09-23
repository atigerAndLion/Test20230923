/*
 * error_save.h
 *
 * Created: 2020/7/24 15:06:57
 *  Author: 20200504602
 */ 


#ifndef ERROR_SAVE_H_
#define ERROR_SAVE_H_

#include "main.h"
#include "W25Q32.h"

#define  MAX_ERROR_COUNT 4096
#define  MAX_SPI_ADDR	(MAX_ERROR_COUNT*128)

#define  ERROR_INDEX_ADDR   (MAX_SPI_ADDR + 0x2000)  // MAX_SPI_ADDR  Å¼Êý£»MAX_SPI_ADDR 0x100 ÆæÊý£»

#define ERROR_R_COUNT (37*2)

extern uint8_t spi_flash_change_flag ;
extern uint32_t spi_flash_write_index ;
extern uint32_t spi_flash_read_index ;

extern uint32_t Flash_error ;
extern uint32_t Flash_error_tmp ;
extern uint8_t Flash_error_write_index ;

void SPI_FLASH_errorstate(void);
void SPI_FLASH_Get_writeindex(void);
void SPI_FLASH_Get_readindex(void);
void SPI_FLASH_Write_error_state(void);
void SPI_FLASH_Write_Index(void);
void SPI_FLASH_Get_Index(void);

#endif /* ERROR_SAVE_H_ */