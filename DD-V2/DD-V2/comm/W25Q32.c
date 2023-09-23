/*
 * W25Q32.c
 *
 * Created: 2020/5/27 9:39:09
 *  Author: 20200504602
 */ 

#include "W25Q32.h"

static struct io_descriptor *io;

void SPI_Init(void)
{
	
	spi_m_sync_get_io_descriptor(&SPI_0, &io);

	spi_m_sync_enable(&SPI_0);
	//io_write(io, example_SPI_0, 12);
}


/************************************************************************/
/* PROCEDURE: Read_Status_Register					*/
/*									*/
/* This procedure reads the status register and returns the byte.	*/
/************************************************************************/


unsigned char SPI_Read_Status_Register()
{
	unsigned char byte = 0;
	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x05);		/* send RDSR command */
	byte = 0x05;
	io_write(io, &byte, 1);
	//byte = SPI2_Exchange8bit(0xFF);		/* receive byte */
	io_read(io, &byte, 1);
	CE_SetHigh();			/* disable device */
	return byte;
}

/************************************************************************/
/* PROCEDURE: Read_Configuration_Register				*/
/*									*/
/* This procedure reads the configuration register and returns the byte.*/
/************************************************************************/
unsigned char SPI_Read_Configuration_Register()
{
	unsigned char byte = 0;
	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x35);		/* send RDSR command */
	byte = 0x35;
	io_write(io, &byte, 1);
	//byte = SPI2_Exchange8bit(0xFF);		/* receive byte */
	io_read(io, &byte, 1);
	CE_SetHigh();			/* disable device */
	return byte;
}



/************************************************************************/
/* PROCEDURE: WREN							*/
/*									*/
/* This procedure enables the Write Enable Latch.               	*/
/************************************************************************/


void SPI_WREN()
{
	unsigned char byte = 0;
	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x06);		/* send WREN command */
	byte = 0x06;
	io_write(io, &byte, 1);
	CE_SetHigh();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: WRDI							*/
/*									*/
/* This procedure disables the Write Enable Latch.			*/
/************************************************************************/

void SPI_WRDI()
{
	unsigned char byte = 0;
	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x04);		/* send WRDI command */
	byte = 0x04;
	io_write(io, &byte, 1);
	CE_SetHigh();			/* disable device */
}


/************************************************************************/
/* PROCEDURE: QuadJ_ID							*/
/*									*/
/* This procedure Reads the manufacturer's ID, device Type and device ID.  It will 	*/
/* use AFh as the command to read the ID.                               */
/* Returns:								*/
/*	ID1(Manufacture's ID = BFh, Device Type =26h , Device ID = 02h)	*/
/*									*/
/************************************************************************/

void SPI_Jedec_ID_Read(char *Manufacturer_Id, char *Device_Type, char *Device_Id)

{
	unsigned char byte = 0;

	CE_SetLow();			 /* enable device */
	//SPI2_Exchange8bit(0x9F);		 /* send JEDEC ID command (9Fh) */
	byte = 0x9F;
	io_write(io, &byte, 1);
	
	//*Manufacturer_Id = SPI2_Exchange8bit(0xFF);       /* receive byte */
	io_read(io, (uint8_t *)Manufacturer_Id, 1);
	//*Device_Type = SPI2_Exchange8bit(0xFF);           /* receive byte */
	io_read(io, (uint8_t *)Device_Type, 1);
	//*Device_Id = SPI2_Exchange8bit(0xFF);             /* receive byte */
	io_read(io,(uint8_t *)Device_Id, 1);
	CE_SetHigh();			 /* disable device */

}



/************************************************************************/
/* PROCEDURE:	Read							*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 7FFFFFH		*/
/************************************************************************/
unsigned char SPI_Read(unsigned long Dst)
{
	unsigned char byte = 0;
	unsigned char buff[4];
	SPI_Wait_Busy();
	CE_SetLow();                                /* enable device */
	//SPI2_Exchange8bit(0x03);                        /* read command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	buff[0] = 0x03;    // ????????????????????
	buff[1] = ((Dst & 0xFFFFFF) >> 16);  //  ????????????????????
	buff[2] = ((Dst & 0xFFFF) >> 8);  //  ????????????????????
	buff[3] = Dst & 0xFF;
	io_write(io, buff, 4);
	//byte = SPI2_Exchange8bit(0xFF);
	io_read(io, &byte, 1);
	CE_SetHigh();                              /* disable device */
	return byte;                            /* return one byte read */
}


/************************************************************************/
/* PROCEDURE:	Read_Cont						*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 256 byte buffer. Maximum number of bytes read is limited 256 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 256)	*/
/************************************************************************/
void SPI_Read_Cont(unsigned long Dst,unsigned long no_bytes, unsigned char *read_data)
{
	//unsigned long i = 0;
	unsigned char buff[4];
	SPI_Wait_Busy();
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x03); 			/* read command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	buff[0] = 0x03;
	buff[1] = ((Dst & 0xFFFFFF) >> 16);
	buff[2] = ((Dst & 0xFFFF) >> 8);
	buff[3] = Dst & 0xFF;
	io_write(io, buff, 4);
	//for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	//{
		//*read_data = SPI2_Exchange8bit(0xFF);	/* receive byte and store at address 80H - FFH */
		//read_data++;
	//}
	io_read(io, (uint8_t *)read_data, no_bytes);
	CE_SetHigh();				/* disable device */

}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read						*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 7FFFFFH		*/
/************************************************************************/
unsigned char SPI_HighSpeed_Read(unsigned long Dst)
{
	unsigned char byte = 0;

	CE_SetLow();                               /* enable device */
	//SPI2_Exchange8bit(0x0B);                        /* read command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	//SPI2_Exchange8bit(0xFF);                        /*dummy byte*/
	//byte = SPI2_Exchange8bit(0xFF);
	CE_SetHigh();                              /* disable device */
	return byte;                            /* return one byte read */
}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read_Cont					*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 256)	*/
/************************************************************************/
void SPI_HighSpeed_Read_Cont(unsigned long Dst,unsigned long no_bytes, unsigned int *read_data)
{
	//unsigned long i = 0;
	//CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x0B); 			/* read command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	//SPI2_Exchange8bit(0xFF);			/*dummy byte*/
	//for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	//{
		//*read_data = SPI2_Exchange8bit(0xFF);	/* receive byte and store at address 80H - FFH */
		//read_data++;
	//}
	//CE_SetHigh();				/* disable device */
}




/************************************************************************/
/* PROCEDURE:	Page_Program						*/
/*									*/
/* This procedure does page programming.  The destination               */
/* address should be provided.                                  	*/
/* The data array of 256 bytes contains the data to be programmed.      */
/* Since the size of the data array is 256 bytes rather than 256 bytes, this page program*/
/* procedure programs only 256 bytes                                    */
/* Assumption:  Address being programmed is already erased and is NOT	*/
/*		block protected.					*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*		data_256[256] containing 256 bytes of data will be programmed using this function */
/************************************************************************/

void SPI_Page_Program(unsigned long Dst,unsigned long no_bytes, unsigned char *Prog_data)
{
	//unsigned int i;
	uint8_t buff[4] ;
	//i=0;
	SPI_Wait_Busy();
	SPI_WREN();
	delay_ms(5);
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x02); 			/* send Byte Program command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	buff[0] = 0x02;
	buff[1] = ((Dst & 0xFFFFFF) >> 16);
	buff[2] = ((Dst & 0xFFFF) >> 8);
	buff[3] = Dst & 0xFF;
	io_write(io, buff, 4);
	
	//for (i=0;i<256;i++)
	//{	SPI2_Exchange8bit(*Prog_data);		/* send byte to be programmed */
		//Prog_data++;
	//}
	io_write(io, (uint8_t *)Prog_data, no_bytes);
	CE_SetHigh();				/* disable device */

}


/************************************************************************/
/* PROCEDURE: Chip_Erase						*/
/*									*/
/* This procedure erases the entire Chip.				*/
/************************************************************************/

void SPI_Chip_Erase()
{
	unsigned char byte = 0;
	SPI_Wait_Busy();
	SPI_WREN();
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0xC7);			/* send Chip Erase command (C7h) */
	byte = 0xc7;
	io_write(io, &byte, 1);
	CE_SetHigh();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Sector_Erase						*/
/*									*/
/* This procedure Sector Erases the Chip.				*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/************************************************************************/


void SPI_Sector_Erase(unsigned long Dst)
{
	uint8_t buff[4] ;
	SPI_Wait_Busy();
	SPI_WREN();
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x20);			/* send Sector Erase command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	buff[0] = 0x20;
	buff[1] = ((Dst & 0xFFFFFF) >> 16);
	buff[2] = ((Dst & 0xFFFF) >> 8);
	buff[3] = Dst & 0xFF;
	io_write(io, buff, 4);
	CE_SetHigh();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Block_Erase						*/
/*									*/
/* This procedure Block Erases 8Kbyte, 32 KByte or 64 KByte of the Chip.*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/************************************************************************/

void SPI_Block_Erase(unsigned long Dst)
{
	uint8_t buff[4];
	SPI_Wait_Busy();
	SPI_WREN();
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0xD8);			/* send Block Erase command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	buff[0] = 0xD8;
	buff[1] = ((Dst & 0xFFFFFF) >> 16);
	buff[2] = ((Dst & 0xFFFF) >> 8);
	buff[3] = Dst & 0xFF;
	io_write(io, buff, 4);
	CE_SetHigh();				/* disable device */
}


/************************************************************************/
/* PROCEDURE: NoOp                                              	*/
/*									*/
/* No operation is performed.                                           */
/************************************************************************/

void SPI_NoOp()
{
	unsigned char byte = 0;
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x00);
	byte = 0x00;
	io_write(io, &byte, 1);
	CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: ResetEn                                                   */
/*									*/
/* This procedure Enables acceptance of the RST (Reset) operation.	*/
/************************************************************************/

void SPI_ResetEn()
{
	unsigned char byte = 0;
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x66);
	byte = 0x66;
	io_write(io, &byte, 1);
	CE_SetHigh();				/* disable device */
}




/************************************************************************/
/* PROCEDURE: Reset                                     		*/
/*									*/
/* This procedure resets the device in to normal operating Ready mode.	*/
/*									*/
/************************************************************************/


void SPI_Reset()
{
	unsigned char byte = 0;
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x99);
	byte = 0x99;
	io_write(io, &byte, 1);
	CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Write_Suspend						*/
/*									*/
/* This procedure suspends Program/Erase operation.			*/
/************************************************************************/

void SPI_Write_Suspend()
{
	//unsigned char byte = 0;
	//CE_SetLow();				/* enable device */
	////SPI2_Exchange8bit(0xb0);
	//byte = 0xb0;
	//io_write(io, &byte, 1);
	//CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Write_Resume						*/
/*									*/
/* This procedure resumes Program/Erase operation.			*/
/************************************************************************/

void SPI_Write_Resume()
{
	//unsigned char byte = 0;
	//CE_SetLow();				/* enable device */
	////SPI2_Exchange8bit(0x30);
	//byte = 0x30;
	//io_write(io, &byte, 1);
	//CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Write_Status_Register					*/
/*									*/
/* This procedure resumes Program/Erase operation.			*/
/************************************************************************/

void SPI_Write_Status_Register(unsigned int data1, unsigned char datalen)
{	
	unsigned char byte = 0;  
	//For data1 - top 8 bits are status reg bits , lower 8 bits are configuration reg bits
	CE_SetLow();				/* enable device */
	//SPI2_Exchange8bit(0x01);
	byte = 0x01;
	io_write(io, &byte, 1);
	//SPI2_Exchange8bit((data1>>8)&0xff);
	byte = (data1>>8)&0xff;
	io_write(io, &byte, 1);
	if (datalen==2)
	{
		//SPI2_Exchange8bit((data1)&0xff);
		byte = ((data1)&0xff);
		io_write(io, &byte, 1);
	}

	CE_SetHigh();				/* disable device */
}


/************************************************************************/
/* PROCEDURE:	ReadSID	(Read Security ID)				*/
/*									*/
/* This procedure reads the security ID					*/
/************************************************************************/


void SPI_ReadSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length)
{

	//unsigned long i;
	//i=0;
	//if (security_length>2048)
	//{ security_length=2048;}
//
	//CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x88);
	//SPI2_Exchange8bit((Dst>>8) & 0xFF);
	//SPI2_Exchange8bit(Dst & 0xFF);
	//SPI2_Exchange8bit(Dst & 0xFF);  //dummy
//
	//for (i=0;i<security_length;i++)
	//{
		//*security_ID = SPI2_Exchange8bit(0xFF);
		//security_ID++;
	//}
	//CE_SetHigh();			/* disable device */
}



/************************************************************************/
/* PROCEDURE:	ProgSID	(Program Security ID)                           */
/*									*/
/* This procedure programs the security ID				*/
/*									*/
/************************************************************************/

void SPI_ProgSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length)
{
	//unsigned long i;
//
	//i=0;
//
	//if (security_length>256)
	//{ security_length=256;}
//
	//CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0xa5);
	//SPI2_Exchange8bit((Dst>>8) & 0xFF);
	//SPI2_Exchange8bit(Dst & 0xFF);
//
//
	//for (i=0;i<security_length;i++)
	//{
		//SPI2_Exchange8bit(*security_ID);
		//security_ID++;
	//}
//
//
	//CE_SetHigh();			/* disable device */
}




/************************************************************************/
/* PROCEDURE:	LockSID							*/
/*									*/
/* This procedure Locks the security ID setting				*/
/*									*/
/************************************************************************/

void SPI_LockSID()
{

	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x85);
	CE_SetHigh();			/* disable device */
}




/************************************************************************/
/* PROCEDURE:	ReadBlockProtection			  		*/
/*									*/
/* This procedure reads block protection register			*/
/*									*/
/************************************************************************/

void SPI_ReadBlockProtection(unsigned int *block_protection_data)
{
	
	//unsigned char i;
	//i=0;
//
	//CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x72);
//
	//for (i=10;i>0;i--)
	//{	*block_protection_data = SPI2_Exchange8bit(0xFF);
		//block_protection_data++;
	//}
	//CE_SetHigh();			/* disable device */
}



/************************************************************************/
/* PROCEDURE:	WriteBlockProtection					*/
/*									*/
/* This procedure writes to block protection register			*/
/*									*/
/************************************************************************/


void SPI_WriteBlockProtection(unsigned int *block_protection_data)
{

	//unsigned char i;
	//i=0;
//
	//CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x42); 		/* read command */
//
	//for (i=10;i>0;i--)
	//{
		//SPI2_Exchange8bit(*block_protection_data);
		//block_protection_data++;
	//}
	//CE_SetHigh();			/* disable device */
}
/************************************************************************/
/* PROCEDURE:	Global Block Protection Unlock				*/
/*									*/
/* This procedure clears all block protection				*/
/************************************************************************/
void SPI_Global_Block_Protection_Unlock()
{

	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x98); 		/* read command */
	CE_SetHigh();			/* disable device */
}


/************************************************************************/
/* PROCEDURE:	LockBlockProtection					*/
/*									*/
/* This procedure locks the block protection register			*/
/************************************************************************/


void SPI_LockBlockProtection()
{

	CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0x8d); 		/* read command */
	CE_SetHigh();			/* disable device */
}





/************************************************************************/
/* PROCEDURE:	Non Volatile Write Lock Protection			*/
/*									*/
/* This procedure writes to block protection register			*/
/*									*/
/************************************************************************/

void SPI_NonVolWriteLockProtection(unsigned int *block_protection_data)
{

	//unsigned char i;
	//i=0;
//
	//CE_SetLow();			/* enable device */
	//SPI2_Exchange8bit(0xE8); 		/* read command */
//
	//for (i=10;i>0;i--)
	//{
		//SPI2_Exchange8bit(*block_protection_data);
		//block_protection_data++;
	//}
	//CE_SetHigh();			/* disable device */
}


/************************************************************************/
/* PROCEDURE: Wait_Busy							*/
/*									*/
/* This procedure waits until device is no longer busy (can be used by	*/
/* Byte-Program, Page-Program, Sector-Erase, Block-Erase and Chip-Erase).*/
/************************************************************************/



void SPI_Wait_Busy()
{
	uint16_t i = 0;
	uint8_t buff[3];
	uint8_t byte = 0;
	while ((SPI_Read_Status_Register()& 0x01) == 0x01)	// waste time until not busy
	{
		delay_ms(1);
		i++;
		wdt_feed(&WDT_0);
		if (i > 400)
		{	
			//CE_SetLow();			/* enable device */
			////SPI2_Exchange8bit(0x04);		/* send WRDI command */
			//byte = 0x66;
			//io_write(io, &byte, 1);
			//CE_SetHigh();			/* disable device */
			//delay_ms(5);
			//CE_SetLow();			/* enable device */
			////SPI2_Exchange8bit(0x04);		/* send WRDI command */
			//byte = 0x99;
			//io_write(io, &byte, 1);
			//CE_SetHigh();			/* disable device */
			//delay_ms(5);
			SPI_0_init();
			SPI_Init();
			SPI_Jedec_ID_Read(buff,buff+1,buff+2);	
			__NOP();
			break;
		}
	}
	wdt_feed(&WDT_0);
}

/************************************************************************/
/* PROCEDURE:	SPI_SFDP_Read						*/
/*									*/
/* This procedure reads SFDP Table.					*/
/*									*/
/************************************************************************/
unsigned char SPI_SFDP_Read(unsigned long Dst)
{
	unsigned char byte = 0;
	uint8_t buff[5];

	CE_SetLow();                               /* enable device */
	//SPI2_Exchange8bit(0x5A);                        /* read command */
	//SPI2_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	//SPI2_Exchange8bit(((Dst & 0xFFFF) >> 8));
	//SPI2_Exchange8bit(Dst & 0xFF);
	//SPI2_Exchange8bit(0xFF);                	/*dummy byte*/
	buff[0] = 0x5A;
	buff[1] = ((Dst & 0xFFFFFF) >> 16);
	buff[2] = ((Dst & 0xFFFF) >> 8);
	buff[3] = Dst & 0xFF;
	buff[4] = 0xFF;
	io_write(io, buff, 5);
	//byte = SPI2_Exchange8bit(0xFF);
	io_read(io, &byte, 1);
	CE_SetHigh();                      	/* disable device */
	return byte;                    	/* return one byte read */
}


	