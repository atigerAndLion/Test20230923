/*
 * error_save.c
 *
 * Created: 2020/7/24 15:07:29
 *  Author: 20200504602
 */ 

#include "error_save.h"
#include "i2c_sd2058.h"
#include "i2c_sh367309.h"
#include "i2c_cht8305.h"
#include "soft_soc.h"
#include "user_bms.h"

uint8_t spi_flash_write_buff[128] = {0};
// ͳ��flash���ϴ���--����д�������ģ�
uint32_t spi_flash_write_index = 0;
uint32_t spi_flash_read_index = 0;
uint32_t spi_flash_serial_index = 0;
uint32_t error_states_new = 0;
uint32_t error_states_old = 0;
uint8_t spi_flash_change_flag = 0;

uint32_t Flash_error = 0;
uint32_t Flash_error_tmp = 0;
uint8_t Flash_error_write_index = 0;


void SPI_FLASH_Get_writeindex(void)
{
	uint32_t i = 0;
	for (i=0;i<MAX_ERROR_COUNT;i++)
	{
		if (SPI_Read((i<<7)) == 0xff)
		{
			spi_flash_write_index = i;
			break;
		}
	}
	if (i == MAX_ERROR_COUNT)
	{
		spi_flash_write_index = 0;
	}
}

void SPI_FLASH_Get_readindex(void)
{
	uint32_t i = 0;
	uint8_t tmp8 = 0;
	//���ж�β����û������
	if (SPI_Read(((MAX_ERROR_COUNT-1)<<7)) == 0xff)
	{
		//β��û���� ��ͷ��ʼ��
		for (i=0;i<MAX_ERROR_COUNT;i++)
		{
			if (SPI_Read((i<<7)+127) == 0xff)
			{
				spi_flash_read_index = i;
				break;
			}
		}
		if (i == MAX_ERROR_COUNT)
		{
			spi_flash_read_index = spi_flash_write_index;
		}
	}
	else
	{
		//β��������  ��β�Ϳ�ʼ��
		for (i=0;i<MAX_ERROR_COUNT;i++)
		{
			tmp8 = SPI_Read((MAX_ERROR_COUNT-1-i)<<7);
			if ( tmp8 == 0x00)
			{
				//�ҵ��Ѷ������һ��
				tmp8 = SPI_Read(((MAX_ERROR_COUNT-1-i)<<7)+127);
				if (tmp8 == 0xaa)
				{
					spi_flash_read_index = MAX_ERROR_COUNT-i;
					if (spi_flash_read_index == MAX_ERROR_COUNT)
					{
						spi_flash_read_index = 0;
					}
					break;
				}
			}
			else if (tmp8 == 0xff)
			{
				spi_flash_read_index = MAX_ERROR_COUNT-i;
				break;
			}
		}
		if (i == 0)
		{
			for (i=0;i<MAX_ERROR_COUNT;i++)
			{
				if (SPI_Read((i<<7)+127) == 0xff)
				{
					spi_flash_read_index = i;
					break;
				}
			}
		}

	}


}

void SPI_FLASH_Write_error_state(void)
{
	uint8_t i = 0;
	uint8_t tmp8 = 0;
	uint32_t check_sum = 0;
	
	if (spi_flash_change_flag  == 1)
	{
		spi_flash_change_flag = 0;
		//��־λ 
		spi_flash_write_buff[0] = 0;
		spi_flash_write_buff[1] = 0;
		//������ʱ����
		spi_flash_write_buff[2] = sd2058_val.years;
		spi_flash_write_buff[3] = sd2058_val.month;
		spi_flash_write_buff[4] = sd2058_val.date;
		spi_flash_write_buff[5] = sd2058_val.hours;
		spi_flash_write_buff[6] = sd2058_val.minutes;
		spi_flash_write_buff[7] = sd2058_val.seconds;
		//����ܵ�ѹ
		spi_flash_write_buff[8] = (Info.afe.Voltage >> 24)&0xff;
		spi_flash_write_buff[9] = (Info.afe.Voltage >> 16)&0xff;
		spi_flash_write_buff[10] = (Info.afe.Voltage >> 8)&0xff;
		spi_flash_write_buff[11] = (Info.afe.Voltage )&0xff;
		//����
		spi_flash_write_buff[12] = (Info.afe.CurCadc >> 24)&0xff;
		spi_flash_write_buff[13] = (Info.afe.CurCadc >> 16)&0xff;
		spi_flash_write_buff[14] = (Info.afe.CurCadc >> 8)&0xff;
		spi_flash_write_buff[15] = (Info.afe.CurCadc )&0xff;
		//������¶�1-5 û����ffff
		spi_flash_write_buff[16] = (Info.afe.Temperature1 + 2731 ) >> 8;
		spi_flash_write_buff[17] = (Info.afe.Temperature1 + 2731 ) & 0xff;
		spi_flash_write_buff[18] = (Info.afe.Temperature2 + 2731 ) >> 8;
		spi_flash_write_buff[19] = (Info.afe.Temperature2 + 2731 ) & 0xff;
		spi_flash_write_buff[20] = (Info.afe.Temperature3 + 2731 ) >> 8;
		spi_flash_write_buff[21] = (Info.afe.Temperature3 + 2731 ) & 0xff;
		spi_flash_write_buff[22] = (Info.afe.Temperature4 + 2731 ) >> 8;
		spi_flash_write_buff[23] = (Info.afe.Temperature4 + 2731 ) & 0xff;
		spi_flash_write_buff[24] = 0xff;
		spi_flash_write_buff[25] = 0xff;
		//ʪ��ADֵ
		spi_flash_write_buff[26] = cht8305_val.Humidity_AD >>8;
		spi_flash_write_buff[27] = cht8305_val.Humidity_AD & 0xff;
		//�����ѹ
		for (i = 0;i < 16;i++)
		{
			spi_flash_write_buff[28+i*2] = Info.afe.VCell[i] >> 8;
			spi_flash_write_buff[29+i*2] = Info.afe.VCell[i] & 0xff;
		}
			
		//��ʱʹ�ø�λ��־λ
		spi_flash_write_buff[60] = _get_reset_reason();
		spi_flash_write_buff[61] = 0;
		//����
		spi_flash_write_buff[62] = 0;
		spi_flash_write_buff[63] = 0;
		// MOS״̬  DET״̬
		if (AFE.BSTATUS3.Bit.CHG_FET == 1)
		{
			tmp8 |= 0x01;
		}
		if (AFE.BSTATUS3.Bit.DSG_FET == 1)
		{
			tmp8 |= 0x02;
		}		
		if (sys_flags.flag.sys_id_connect_flag == 1)
		{
			tmp8 |= 0x04;
		}		
		
		spi_flash_write_buff[64] = 0;
		spi_flash_write_buff[65] = tmp8;
		// soc
		spi_flash_write_buff[66] = sys_cap.val.re_cap_rate;
		spi_flash_write_buff[67] = 0;
		//�����־λ
		spi_flash_write_buff[68] = (error_states_new >> 24)&0xff;
		spi_flash_write_buff[69] = (error_states_new >> 16)&0xff;
		spi_flash_write_buff[70] = (error_states_new >> 8)&0xff;
		spi_flash_write_buff[71] = (error_states_new )&0xff;
		//����״̬��־λ
		spi_flash_write_buff[72] = AFE.BALH;
		spi_flash_write_buff[73] = AFE.BALL;
		
		for (i=0;i<73;i++)
		{
			check_sum += spi_flash_write_buff[i];
		}
		
		spi_flash_write_buff[74] = (check_sum >> 24) & 0xff;
		spi_flash_write_buff[75] = (check_sum >> 16) & 0xff;
		spi_flash_write_buff[76] = (check_sum >> 8) & 0xff;
		spi_flash_write_buff[77] = (check_sum ) & 0xff;
		
		//ȫ������ ��0xff
		for (i=78;i<128;i++)
		{
			spi_flash_write_buff[i] = 0xff;
		}
		
		//д��flash
		if (spi_flash_write_index % 32 == 0) //�ж��Ƿ���Ҫ����
		{
			SPI_Sector_Erase(spi_flash_write_index << 7);
		}
		SPI_Page_Program(spi_flash_write_index << 7,128,spi_flash_write_buff);
		spi_flash_write_index++;
		spi_flash_write_index &= (MAX_ERROR_COUNT-1);
		
	}
}

void SPI_FLASH_errorstate(void)
{
	uint32_t tmp32 = 0;
	//bit0 �ŵ�һ������
	if (protects.protect.dch_ocp_flag == 1)
	{
		error_states_new |= (1 << 0);
	}
	else
	{
		if ((error_states_new & (1 << 0)) == 1)
		{
			error_states_new &= ~(1 << 0);
		}
	} 
	
	//bit1 �ŵ��������
	if (protects.protect.dch_ocp2_flag == 1)
	{
		error_states_new |= (1 << 1);
	}
	else
	{
		if ((error_states_new & (1 << 1)) == 1)
		{
			error_states_new &= ~(1 << 1);
		}
	}

	//bit2 �ŵ����
	if (protects2.protect.dch_otp_flag == 1)
	{
		error_states_new |= (1 << 2);
	}
	else
	{
		if ((error_states_new & (1 << 2)) == 1)
		{
			error_states_new &= ~(1 << 2);
		}
	}

	//bit3 �ŵ����
	if (protects2.protect.dch_ltp_flag == 1)
	{
		error_states_new |= (1 << 3);
	}
	else
	{
		if ((error_states_new & (1 << 3)) == 1)
		{
			error_states_new &= ~(1 << 3);
		}
	}

	//bit4 ��·����
	if (protects.protect.dch_scd_flag == 1)
	{
		error_states_new |= (1 << 4);
	}
	else
	{
		if ((error_states_new & (1 << 4)) == 1)
		{
			error_states_new &= ~(1 << 4);
		}
	}	
	
	//bit5 Ƿѹ���� SOC
	if (protects2.protect.soc_lp_flag == 1)
	{
		error_states_new |= (1 << 5);
	}
	else
	{
		if ((error_states_new & (1 << 5)) == 1)
		{
			error_states_new &= ~(1 << 5);
		}
	}
	
	//bit6 ����Ƿѹ���� ����Ƿѹ
	if (protects.protect.dch_lvp2_flag == 1)
	{
		error_states_new |= (1 << 6);
	}
	else
	{
		if ((error_states_new & (1 << 6)) == 1)
		{
			error_states_new &= ~(1 << 6);
		}
	}

	//bit7 ������
	if (protects.protect.chg_ocp_flag == 1)
	{
		error_states_new |= (1 << 7);
	}
	else
	{
		if ((error_states_new & (1 << 7)) == 1)
		{
			error_states_new &= ~(1 << 7);
		}
	}

	//bit8 ������
	if (protects2.protect.chg_otp_flag == 1)
	{
		error_states_new |= (1 << 8);
	}
	else
	{
		if ((error_states_new & (1 << 8)) == 1)
		{
			error_states_new &= ~(1 << 8);
		}
	}
	
	//bit9 ������
	if (protects2.protect.chg_ltp_flag == 1)
	{
		error_states_new |= (1 << 9);
	}
	else
	{
		if ((error_states_new & (1 << 9)) == 1)
		{
			error_states_new &= ~(1 << 9);
		}
	}
	
	//bit10 ��о��ѹһ������
	if (protects.protect.chg_ovp_flag == 1)
	{
		error_states_new |= (1 << 10);
	}
	else
	{
		if ((error_states_new & (1 << 10)) == 1)
		{
			error_states_new &= ~(1 << 10);
		}
	}
	//bit11 ��о��ѹ��������

	//bit12 ��о���ع�ѹ����

	//bit13 ���MOS����
	if (protects.protect.chg_mos_err_flag == 1)
	{
		error_states_new |= (1 << 13);
	}
	else
	{
		if ((error_states_new & (1 << 13)) == 1)
		{
			error_states_new &= ~(1 << 13);
		}
	}
	
	//bit14 �ŵ�MOS����
	if (protects.protect.dch_mos_err_flag == 1)
	{
		error_states_new |= (1 << 14);
	}
	else
	{
		if ((error_states_new & (1 << 14)) == 1)
		{
			error_states_new &= ~(1 << 14);
		}
	}
	
	//bit15 ��о����
	if (protects.protect.cell_low_2v_flag == 1)
	{
		error_states_new |= (1 << 15);
	}
	else
	{
		if ((error_states_new & (1 << 15)) == 1)
		{
			error_states_new &= ~(1 << 15);
		}
	}
	
	//bit16 ѹ�����澯
	
	//bit17 ѹ��������
	if (protects.protect.cell_dif_volt_flag == 1)
	{
		error_states_new |= (1 << 17);
	}
	else
	{
		if ((error_states_new & (1 << 17)) == 1)
		{
			error_states_new &= ~(1 << 17);
		}
	}
	
	//bit18 ��籣��
	
	//bit19 �ŵ籣��
	
	//bit20 �Ƿ����
	
	//bit21 ���ѹ��������
	
	//bit22 �²�������
	if (protects2.protect.temp_diff_flag == 1)
	{
		error_states_new |= (1 << 22);
	}
	else
	{
		if ((error_states_new & (1 << 22)) == 1)
		{
			error_states_new &= ~(1 << 22);
		}
	}
	
	//bit23 ��о����
	if (AFE.BSTATUS1.Bit.PF == 1)
	{
		error_states_new |= (1 << 23);
	}
	else
	{
		if ((error_states_new & (1 << 23)) == 1)
		{
			error_states_new &= ~(1 << 23);
		}
	}
	//bit24 MOS����
	if (protects2.protect.mosfet_temp_err_flag == 1)
	{
		error_states_new |= (1 << 24);
	}
	else
	{
		if ((error_states_new & (1 << 24)) == 1)
		{
			error_states_new &= ~(1 << 24);
		}
	}
	
	//bit25 ϵͳ��λ
	if (sys_uarts.flag.cmd_reset_flag == 1)
	{
		error_states_new |= (1 << 25);
	}
	else
	{
		if ((error_states_new & (1 << 25)) == 1)
		{
			error_states_new &= ~(1 << 25);
		}
	}
	
	//bit26 ϵͳ����
	if (power_first_flag == 1)
	{
		error_states_new |= (1 << 26);
	}
	else
	{
		if ((error_states_new & (1 <<26)) == 1)
		{
			error_states_new &= ~(1 << 26);
		}
	}
	
	//bit27 ϵͳ�ػ�
	if (sys_flags.flag.sys_close_flag == 1)
	{
		error_states_new |= (1 << 27);
	}
	else
	{
		if ((error_states_new & (1 << 27)) == 1)
		{
			error_states_new &= ~(1 << 27);
		}
	}

	if (error_states_new != error_states_old)
	{
		tmp32 = error_states_new ^ error_states_old;  
		tmp32 = tmp32 & error_states_new;
		if (tmp32 != 0)
		{
			spi_flash_change_flag = 1;
			SPI_FLASH_Write_error_state();
			error_states_old = error_states_new;
			SPI_FLASH_Write_Index();
		}		
	}
	
	//���ɻָ����ϼ�¼MCU �ڲ�flash
	if(protects.protect.ntc_open_flag ==1)
	{
		Flash_error |= 0x01;
	}
	if(protects.protect.cell_dif_volt_flag ==1)
	{
		Flash_error |= 0x02;
	}
	if(protects.protect.cell_low_2v_flag == 1)
	{
		Flash_error |= 0x04;
	}
	if(protects.protect.dch_mos_err_flag ==1)
	{
		Flash_error |= 0x08;
	}
	if(protects.protect.chg_mos_err_flag ==1)
	{
		Flash_error |= 0x10;
	}
	
	if (protects.protect.ntc_short_flag ==1 )
	{
		Flash_error |= 0x20;
	}
	
	//�����ϼ�¼����ֵ����ϼ�¼�����Ĳ�ͬ˵���������ϣ�����ǰ����ֵд��
	//EEROM�����±���,�����Ĵ�����Ϊ�˴ﵽ��������д��һ�ε�Ŀ�ģ���ֹ���д��
	if(Flash_error_tmp != Flash_error)
	{
		Flash_error_tmp = Flash_error;
		flash_save_data(VER_FLASH_ADDR+FLASH_ERROR_ADDR,Flash_error_write_index,Flash_error);
		Flash_error_write_index++;
		Flash_error_write_index &= 0x07;
	}

}

void SPI_FLASH_Write_Index(void)
{
	uint32_t check_sum = 0;
	uint8_t i = 0;
	
	*((uint32_t *)(spi_flash_write_buff))  = spi_flash_write_index;
	*((uint32_t *)(spi_flash_write_buff + 4))  = spi_flash_read_index;
	*((uint32_t *)(spi_flash_write_buff + 8))  = spi_flash_serial_index;
	
	for (i=0;i<12;i++)
	{
		check_sum += spi_flash_write_buff[i];
	}
	
	*((uint32_t *)(spi_flash_write_buff + 12))  = check_sum;
	
	if (spi_flash_serial_index % 2 == 0)
	{
		SPI_Sector_Erase(ERROR_INDEX_ADDR);
		SPI_Page_Program(ERROR_INDEX_ADDR,16,spi_flash_write_buff);
	}
	else
	{
		SPI_Sector_Erase(ERROR_INDEX_ADDR + 0x1000);
		SPI_Page_Program(ERROR_INDEX_ADDR + 0x1000,16,spi_flash_write_buff);
	}
	spi_flash_serial_index++;
}

void SPI_FLASH_Get_Index(void)
{
	uint32_t write_index1 = 0;
	uint32_t write_index2 = 0;
	uint32_t read_index1 = 0;
	uint32_t read_index2 = 0;
	uint32_t serial_index1 = 0;
	uint32_t serial_index2 = 0;	
	uint32_t checksum1 = 0;
	uint32_t checksum2 = 0;	
	uint8_t checksum1_true_flag = 0;
	uint8_t checksum2_true_flag = 0;	
	uint8_t read_buff[16];
	SPI_Read_Cont(ERROR_INDEX_ADDR,16,read_buff);
	write_index1 = *(uint32_t *)(read_buff);
	read_index1 = *(uint32_t *)(read_buff+4);
	serial_index1 = *(uint32_t *)(read_buff+8);
	checksum1 = *(uint32_t *)(read_buff+12);
	
	SPI_Read_Cont(ERROR_INDEX_ADDR + 0x1000,16,read_buff);
	write_index2 = *(uint32_t *)(read_buff);
	read_index2 = *(uint32_t *)(read_buff+4);
	serial_index2 = *(uint32_t *)(read_buff+8);
	checksum2 = *(uint32_t *)(read_buff+12);	
	
	// ��ȡ�������൱�� У��һ�£�
	// ȡwrite_index1��write_index2�� ��ֵ(���� ������ ż��)  ����
	if ( write_index1 + read_index1 + serial_index1 == checksum1  )
	{
		checksum1_true_flag = 1;
	}
	
	if ( write_index2 + read_index2 + serial_index2 == checksum2  )
	{
		checksum2_true_flag = 1;
	}
	
	if (checksum1_true_flag == 1 && checksum2_true_flag == 1)
	{
		if (serial_index2 > serial_index1)
		{
			spi_flash_write_index = write_index2;
			spi_flash_read_index = read_index2;
			spi_flash_serial_index = serial_index2;
		}
		else
		{
			spi_flash_write_index = write_index1;
			spi_flash_read_index = read_index1;
			spi_flash_serial_index = serial_index1;
		}
	}
	else if (checksum1_true_flag == 1)
	{
		spi_flash_write_index = write_index1;
		spi_flash_read_index = read_index1;
		spi_flash_serial_index = serial_index1;
	}
	else if (checksum2_true_flag == 1)
	{
		spi_flash_write_index = write_index2;
		spi_flash_read_index = read_index2;
		spi_flash_serial_index = serial_index2;
	}
	else
	{
		spi_flash_write_index = 0;
		spi_flash_read_index = 0;
		spi_flash_serial_index = 0;
	}
		
}