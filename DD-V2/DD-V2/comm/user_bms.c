/*
* user_bms.c
*
* Created: 2020/5/15 14:41:48
*  Author: chenjiawei
*/ 
#include "i2c_sh367309.h"
#include "user_bms.h"
#include "soft_soc.h"
#include "stdlib.h"
#include "comm/CAN_RW_Manage.h"
#include "comm/W25Q32.h"


uint16_t vin_val;

uint8_t chg_retry_cnt = 0;
uint8_t dsg_retry_cnt = 0;
uint8_t dsg2_retry_cnt = 0;
uint8_t dsg_sc_retry_cnt = 0;
uint16_t stick_30s_cnt = 0;//预放电30S 计数

//balance 
uint8_t blance_stick_1s_cnt = 0;
uint8_t blance_cnt = 0;
uint16_t cell_state = 0;
int16_t cell_dif = 0;
uint16_t blance_cell_state = 0;
uint8_t blance_3_hour = 0;
uint8_t blance_cell_cnt = 0;
uint16_t blance_1_minute = 0;
uint8_t balance_start = 0;

//电流校准
uint8_t count = 0;
uint16_t cadc_temp[32] = {0};
uint32_t cadc_sum = 0;
uint32_t cadc_max = 0;
uint32_t cadc_min = 0xffff;

//spi read count
uint32_t spi_read_count = 0;

//全局计数变量
static u8 cur_chg_start_cnt = 0;

u8 bat_uvp_cnt = 0;
u8 soc_lp_cnt = 0;

u8 id_connect_cnt = 0;

/*
* 103AT volt
* 3.3V--10K--103AT---GND
* REF 3.3V
* ADC 10bit
*/
const uint16_t TEMP_VOLT_TABLE[MCU_TEMP_DEGREE_RANGE] =  
{
	3909,3899,3888,3877,3865,3853,3840,3826,3812,3798,
	3783,3767,3750,3733,3716,3698,3679,3659,3639,3618,
	3597,3578,3558,3537,3516,3494,3465,3436,3405,3374,
	3343,3314,3284,3253,3222,3191,3156,3121,3085,3049,
	3013,2977,2941,2905,2868,2830,2794,2757,2719,2681,
	2643,2594,2546,2499,2454,2409,2379,2347,2314,2279,
	2244,2204,2165,2126,2087,2048,2010,1972,1934,1897,
	1860,1823,1787,1750,1714,1679,1644,1609,1575,1542,
	1509,1476,1444,1412,1380,1349,1319,1289,1260,1231,
	1203,1176,1149,1123,1098,1073,1048,1023,999,976,
	953,930,908,887,866,845,825,805,786,767,
	749,732,714,698,682,666,649,634,618,603,
	589,574,560,546,532,519,507,496,485,474,
	464,453,442,432,422,412,402,393,384,375,
	367,359,351,343,336,328,321,314,307,300,
	293,287,281,275,269,263,257,251,246,240,
	235,230,225,220,215,210
};

/****************************************************************************
FUNCTION		: AFE_Update_Function
DESCRIPTION		: 从AFE更新数据
INPUT			: None
OUTPUT			: None
*****************************************************************************/

void AFE_Update_Function()
{
	UpdateDataFromSH367309();
}
 
/****************************************************************************
FUNCTION		: State_Change_Function
DESCRIPTION	: 更改系统状态函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/

void State_Change_Function()
{
	Adapter_Function();
	Voltage_Protect_Function();
	Current_Protect_Function();
	Temperature_Protect_Function();
	Protect_Retry_Function();
	Balance_Function();
	Current_Cal_Function();
	Can_send_spi_flash();
	Can_Check_Init();
}
 
/****************************************************************************
FUNCTION		: ADC_Function
DESCRIPTION	: MCU 自带ADC 采集电压处理函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
uint16_t ADC_Filter(u8 ch)
{
	unsigned char	i=0;
	unsigned int	ad_data_total =0,ad_val=0,ad_max=0x0000,ad_min=0xffff;
	uint8_t buffer[2]={0};

	for(i=0;i<6;i++)
	{
		adc_sync_set_inputs(&ADC_0,ch,0x18,0);
		adc_sync_read_channel(&ADC_0, ch, buffer, 2);
		ad_val = buffer[1] << 8 | buffer[0];
		//ad_val = ADC_GetConvert(ch);
		ad_data_total+=ad_val;
		if(ad_val<ad_min)
		ad_min=ad_val;
		if(ad_val>ad_max)
		ad_max=ad_val;
	}
	ad_data_total=ad_data_total-ad_max;
	ad_data_total=ad_data_total-ad_min;
	ad_val = ad_data_total>>2;
	return ad_val;
}


 
void ADC_Function()
{
	uint8_t buffer[2]={0};
	uint16_t ad_val=0;
	 
	 
	 
	 
	
	// GET WT1 WT2
	adc_sync_enable_channel(&ADC_0, 0);
	adc_sync_enable_channel(&ADC_1, 0);
	 
	adc_sync_set_inputs(&ADC_0,AD_WT2_ADC0_CHANNEL,0x18,0);
	adc_sync_read_channel(&ADC_0, AD_WT2_ADC0_CHANNEL, buffer, 2);

	adc_sync_set_inputs(&ADC_1,AD_WT1_ADC1_CHANNEL,0x18,0);
	adc_sync_read_channel(&ADC_1, AD_WT1_ADC1_CHANNEL, buffer, 2);

	// GET AD BAT	
	adc_sync_set_inputs(&ADC_0,AD_BAT_ADC0_CHANNEL,0x18,0);
	adc_sync_read_channel(&ADC_0, AD_BAT_ADC0_CHANNEL, buffer, 2);
	ad_val = buffer[1] << 8 | buffer[0];
	ad_val = ad_val*3300 /4096 *61; //电阻分压1/31
	 
	// GET AD RT4 RT5
	//adc_sync_set_inputs(&ADC_0,AD_RT4_ADC0_CHANNEL,0x18,0);
	//adc_sync_read_channel(&ADC_0, AD_RT4_ADC0_CHANNEL, buffer, 2);
	//ad_val = buffer[1] << 8 | buffer[0];
	ad_val = ADC_Filter(AD_RT4_ADC0_CHANNEL);
	Info.afe.Temperature4 = Find_MCU_Temp(ad_val) - MCU_TEMP_OFFSET_VAL;
	 
	//adc_sync_set_inputs(&ADC_0,AD_RT5_ADC0_CHANNEL,0x18,0);
	//adc_sync_read_channel(&ADC_0, AD_RT5_ADC0_CHANNEL, buffer, 2);
	//ad_val = buffer[1] << 8 | buffer[0];
	ad_val = ADC_Filter(AD_RT5_ADC0_CHANNEL);
	Info.g_NTC1_DSGFET_T = Find_MCU_Temp(ad_val) - MCU_TEMP_OFFSET_VAL;
	Info.g_NTC2_CHGFET_T = Info.g_NTC1_DSGFET_T;
	 
	RT_ON_OFF();
		  
	// GET AD P+
	//adc_sync_set_inputs(&ADC_0,AD_PP_ADC0_CHANNEL,0x18,0);
	//adc_sync_read_channel(&ADC_0, AD_PP_ADC0_CHANNEL, buffer, 2);
	//ad_val = buffer[1] << 8 | buffer[0];
	ad_val = ADC_Filter(AD_PP_ADC0_CHANNEL);
	vin_val = ad_val*3300 /4096 *31;//电阻分压1/31
	
	PMOS_EN_OFF();
	 
	// GET AD 5V
	VCC_EN_ON();
	adc_sync_set_inputs(&ADC_1,AD_5V_ADC1_CHANNEL,0x18,0);
	adc_sync_read_channel(&ADC_1, AD_5V_ADC1_CHANNEL, buffer, 2);
	ad_val = buffer[1] << 8 | buffer[0];
	ad_val = ad_val*3300 /4096 *3; //电阻分压1/3
}
 
/****************************************************************************
FUNCTION		: Adapter_Function
DESCRIPTION	: 适配器接入 处理函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
 
void Adapter_Function()
{
	static u8 adapter_connect_cnt = 0;
	
	uint16_t tmp_16 = 0xffff;
	
	// ID 在位检测
	if (DET_READ())
	{
		//ID 未接入电平
		if (sys_flags.flag.sys_id_connect_flag == 1)
		{
			id_connect_cnt--;
			if (id_connect_cnt == 0)
			{
				//ID拔出
				if (sys_flags.flag.dch_state_flag == 1) //如果此时电流大于300mA 则为异常拔出状态
				{
					sys_flags.flag.sys_id_error_flag = 1;
				}
				sys_flags.flag.sys_id_connect_flag = 0;
				// ID移除 恢复放电过流一级 二级保护 充电过流保护
				protects.protect.dch_ocp_flag = 0;
				protects.protect.dch_ocp2_flag = 0;
				protects.protect.chg_ocp_flag = 0;
				protects.protect.dch_scd_flag = 0;
				sys_flags.flag.adapter_connect_flag = 0;
				sys_flags.flag.sys_comm_open_dch_flag = 0;
				sys_flags.flag.open_dch_30s_end_flag =1; //清空预防电完成标志位
				chg_retry_cnt = 0; //重置充电过流次数 放电一级过流次数 放电二级过流次数 AFE清电流错误
				dsg_retry_cnt = 0;
				dsg2_retry_cnt = 0;
				dsg_sc_retry_cnt = 0;
					//UpdateStates();
					//if (AFE.BSTATUS1.Bit.OCD1 == 1 ||
						//AFE.BSTATUS1.Bit.OCD2 == 1 ||
						//AFE.BSTATUS1.Bit.OCC == 1 ||
						//AFE.BSTATUS1.Bit.SC == 1
					//) //如果没清除成功  不清除标志位
					//{
						//sys_flags.flag.sys_id_connect_flag = 1;
						//id_connect_cnt++; //此时为0  下个循环-- 为0 继续清除过流
					//}
						
			}
		} 
		else
		{
			id_connect_cnt = 0;
			
			if (
				AFE.BSTATUS1.Bit.OCD1 == 1 ||
				AFE.BSTATUS1.Bit.OCD2 == 1 ||
				AFE.BSTATUS1.Bit.OCC == 1 ||
				AFE.BSTATUS1.Bit.SC == 1 ||
				protects.protect.dch_ocp_flag == 1 ||
				protects.protect.chg_ocp_flag == 1 ||
				protects.protect.dch_scd_flag == 1 ||
				protects.protect.dch_ocp2_flag == 1
			)
			{
				protects.protect.dch_ocp_flag = 0;
				protects.protect.chg_ocp_flag = 0;
				protects.protect.dch_scd_flag = 0;
				protects.protect.dch_ocp2_flag = 0;
				__disable_irq();
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 1;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(5);
				__enable_irq();
			}
		}
	} 
	else
	{
		//ID 接入电平
		id_connect_cnt++;
		if (id_connect_cnt > GC_DELAY_1S)
		{
			id_connect_cnt = GC_DELAY_5S;
			if (sys_flags.flag.sys_id_connect_flag == 0)
			{
				sys_flags.flag.sys_id_connect_flag = 1; //置位ID
				sys_flags.flag.open_dch_30s_end_flag =0; //清空预防电完成标志位
				stick_30s_cnt = 0;
				
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
				
				
				//test
				soc_count_start_flag = 1;
				soc_count = 0;
				
				
			}
			
			sys_flags.flag.sys_id_error_flag = 0 ; //ID接入 清除ID异常状态
			//ID接入
		}
	}
	
	//ID在位 预放电开始
	if (sys_flags.flag.sys_id_connect_flag == 1 && sys_flags.flag.open_dch_30s_end_flag == 0)
	{
		stick_30s_cnt++; // 暂放这里  考虑定时器增加计数
		if(stick_30s_cnt > GC_DELAY_30S)
		{
			stick_30s_cnt = GC_DELAY_30S;
			
			flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
			if (tmp_16 == 0x0000 && sys_flags.flag.sys_id_connect_flag == 1)
			{
				sys_flags.flag.sys_comm_open_dch_flag =1;
			}

			sys_flags.flag.open_dch_30s_end_flag = 1; // 预放电结束
			if(protects.protect.forbidden_output == 1)
			{
				protects.protect.forbidden_output = 0;
			}
		}
	} 
	else
	{   
		//预放电没结束  ID移除  直接置位预放电
		if (sys_flags.flag.sys_id_connect_flag == 0 && sys_flags.flag.open_dch_30s_end_flag == 0)
		{
			sys_flags.flag.open_dch_30s_end_flag = 1;
			if(protects.protect.forbidden_output == 1)
			{
				protects.protect.forbidden_output = 0;
			}
		}
	}
	
	
	//if(sys_flags.flag.adapter_int_flag == 1)
	//{
		//sys_flags.flag.adapter_connect_flag =1;
		//sys_flags.flag.adapter_int_flag =0;
		//adapter_connect_cnt = GC_DELAY_1S;
	//}
	if (sys_flags.flag.adapter_connect_flag == 0)
	{
		if((sys_flags.flag.sys_comm_open_dch_flag == 0)&&(sys_flags.flag.open_dch_30s_end_flag == 1))
		{
			if(vin_val > VPACK_CONNECT && sys_flags.flag.sys_id_connect_flag == 1)
			{
				adapter_connect_cnt++;
				if(adapter_connect_cnt > GC_DELAY_2S)
				{
					adapter_connect_cnt = GC_DELAY_4S;
					sys_flags.flag.adapter_connect_flag =1;
					cur_chg_start_cnt = GC_DELAY_2S;
					sys_flags.flag.chg_state_flag =1;   
				}
			}
			else
			{
				adapter_connect_cnt = 0;
			}
		}
		else
		{
			adapter_connect_cnt = 0;
		}
	}
	else
	{
		if (sys_flags.flag.chg_state_flag ==0)
		{
			if ((protects2.protect.chg_otp_flag ==0)&&(protects2.protect.chg_ltp_flag ==0))
			{
				if(vin_val < VPACK_NCONT)
				{
					if(adapter_connect_cnt>0)
					{
						adapter_connect_cnt--;
					}
					else
					{
						sys_flags.flag.adapter_connect_flag =0;
					}
				}
				else
				{
					if((protects.protect.chg_ocp_flag == 0) && (protects.protect.chg_ovp_flag == 0))
					{
						if (Info.afe.Voltage < VBAT_FULL )
						{
							if(adapter_connect_cnt>0)
							{
								adapter_connect_cnt--;
							}
							else
							{
								sys_flags.flag.adapter_connect_flag =0;
							}
						}
						
					}
					else
					{
						adapter_connect_cnt = GC_DELAY_4S;
					}
				}
			}
			else
			{
				adapter_connect_cnt = GC_DELAY_4S;
			}
		}

		

	}

	

}
 
/****************************************************************************
FUNCTION		: Voltage_Protect_Function
DESCRIPTION	: 过压 欠压等电压相关 处理函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
 
void Voltage_Protect_Function()
{
	static u8 cell_ovp_cnt = 0;
	static u8 cell_uvp_cnt = 0;
	static u8 bat_ovp_cnt = 0;
	
	static u16 cell_err_cnt = 0;
	static u16 cell_dif_cnt = 0;

	uint16_t tmp_16=0;
	//Hard OVP
	if (AFE.BSTATUS1.Bit.OV == 1)
	{
		protects.protect.chg_ovp_flag = 1;
		protects.protect.chg_full_flag = 1;
		cell_ovp_cnt = GC_DELAY_2S;
		
			//过充保护  重置ID
			flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
			if (tmp_16 == 0x0000)
			{
				id_connect_cnt = 0;
				sys_flags.flag.sys_id_connect_flag = 0;
			}
	}
		
	//OVP
	if(Info.vcell_max > VCELL_OVP)
	{
		cell_ovp_cnt++;
		if(cell_ovp_cnt > GC_DELAY_2S)
		{
			cell_ovp_cnt = GC_DELAY_2S;
			protects.protect.chg_ovp_flag = 1;
			protects.protect.chg_full_flag = 1;
			
			//过充保护  重置ID
			flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
			if (tmp_16 == 0x0000)
			{
				id_connect_cnt = 0;
				sys_flags.flag.sys_id_connect_flag = 0;
			}
		}
	}
	else
	{
		if(protects.protect.chg_ovp_flag  == 1)
		{
			if(Info.vcell_max < VCELL_OVP_CLEAR)
			{
				if(cell_ovp_cnt > 0)
				{
					cell_ovp_cnt--;
				}
				else
				{
					protects.protect.chg_ovp_flag =0;
					protects.protect.chg_full_flag = 0;
				}
			}
			else
			{
				cell_ovp_cnt =GC_DELAY_2S;
			}
		}
		else
		{
			cell_ovp_cnt = 0;
		}
	}
	//Hard UVP
	if (AFE.BSTATUS1.Bit.UV == 1)
	{
		cell_uvp_cnt = GC_DELAY_2S;
		protects.protect.dch_lvp2_flag = 1;
		//sys_flags.flag.sys_close_flag = 1;
	}
	
	//UVP cell
	if(Info.vcell_min < VCELL_UVP)
	{
		cell_uvp_cnt++;
		if(cell_uvp_cnt > GC_DELAY_2S)
		{
			cell_uvp_cnt = GC_DELAY_2S;
			protects.protect.dch_lvp2_flag = 1;
			//sys_flags.flag.sys_close_flag = 1;
		}
	}
	else
	{
		if(protects.protect.dch_lvp2_flag  == 1)
		{
			if(Info.vcell_min > VCELL_UVP_CLEAR)
			{
				if(cell_uvp_cnt > 0)
				{
					cell_uvp_cnt--;
				}
				else
				{
					protects.protect.dch_lvp2_flag = 0;
					//sys_flags.flag.sys_close_flag = 0;
				}
			}
			else
			{
				cell_uvp_cnt = GC_DELAY_2S;
			}
		}
		else
		{
			cell_uvp_cnt = 0;
		}
	}
	
	//UVP bat
	if(Info.afe.Voltage < VBAT_UVP)
	{
		bat_uvp_cnt++;
		if(bat_uvp_cnt > GC_DELAY_2S)
		{
			bat_uvp_cnt = GC_DELAY_2S;
			protects.protect.dch_lvp_flag = 1;
		}
	}
	else
	{
		if(protects.protect.dch_lvp_flag  == 1)
		{
			if(Info.afe.Voltage > VBAT_UVP_CLEAR)
			{
				if(bat_uvp_cnt > 0)
				{
					bat_uvp_cnt--;
				}
				else
				{
					protects.protect.dch_lvp_flag = 0;
				}
			}
			else
			{
				bat_uvp_cnt = GC_DELAY_2S;
			}
		}
		else
		{
			bat_uvp_cnt = 0;
		}
	}	
	
		//OVP BAT
	
	if((Info.afe.Voltage > VBAT_FULL)&&(sys_flags.flag.adapter_connect_flag == 1)&&(sys_flags.flag.chg_state_flag == 0))
	{		
		bat_ovp_cnt++;
		if(bat_ovp_cnt > GC_DELAY_4S)
		{
			bat_ovp_cnt = GC_DELAY_4S;
			protects.protect.chg_full_flag = 1;
		}
	}
	else
	{
		if(protects.protect.chg_full_flag == 1)
		{
			if((Info.afe.Voltage < VBAT_FULL_CLEAR)&&(protects.protect.chg_ovp_flag == 0))
			{
				if(bat_ovp_cnt>0)
				{
					bat_ovp_cnt--;
				}
				else
				{
					protects.protect.chg_full_flag =0;
				}
			}
			else
			{
				bat_ovp_cnt = GC_DELAY_1S;
			}
		}
		else
		{
			bat_ovp_cnt =0;
		}
	}

	

	if(Info.vcell_min<VCELL_ERR)
	{
		cell_err_cnt++;
		if(cell_err_cnt > GC_DELAY_10S)
		{
			protects.protect.cell_low_2v_flag = 1;
			sys_flags.flag.sys_close_flag = 1;
		}
	}
	else
	{
		cell_err_cnt =0;
	}
	
	if((Info.vcell_max - Info.vcell_min) > VCELL_DEF && (Info.vcell_max - Info.vcell_min) < 1500)
	{
		cell_dif_cnt++;
		if(cell_dif_cnt > GC_DELAY_10S)
		{
			protects.protect.cell_dif_volt_flag = 1;
			sys_flags.flag.sys_comm_open_dch_flag = 0;
			sys_flags.flag.adapter_connect_flag = 0 ;
			//sys_flags.flag.sys_close_flag = 1;
		}
	}
	else
	{
		cell_dif_cnt = 0;
	}
	
	//SOC under 4%
	if(sys_cap.val.re_cap_rate <= SOC_LP)
	{
		soc_lp_cnt++;
		if(soc_lp_cnt > GC_DELAY_2S)
		{
			soc_lp_cnt = GC_DELAY_2S;
			flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_NO_SOCLP_ADDR,(uint8_t *)&tmp_16, 2);
			if(tmp_16 == 0xffff)
			{
				protects2.protect.soc_lp_flag = 1;
			}
			
		}
	}
	else
	{
		if(protects2.protect.soc_lp_flag  == 1)
		{
			if(sys_cap.val.re_cap_rate >= SOC_LP_CLEAR)
			{
				if(soc_lp_cnt > 0)
				{
					soc_lp_cnt--;
				}
				else
				{
					protects2.protect.soc_lp_flag = 0;
				}
			}
			else
			{
				soc_lp_cnt = GC_DELAY_2S;
			}
		}
		else
		{
			soc_lp_cnt = 0;
		}
	}
	
	
}
 
/****************************************************************************
FUNCTION		: Current_Protect_Function
DESCRIPTION	: 过流 短路等电流相关 处理函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
 
void Current_Protect_Function()
{
	static u8 cur_dsg_ocp_cnt = 0;
	static u8 cur_dsg_start_cnt = 0;
	static u8 cur_dsg_ocp2_cnt = 0;
	static u8 cur_chg_ocp_cnt = 0;

	static u8 cur_dsg_mos_err_cnt = 0;
	static u8 cur_chg_mos_err_cnt = 0;
	static uint16_t dsg_to_chg_cnt = 0;
	uint16_t tmp_16 = 0xffff;
		
	if(Info.afe.CurCadc > ICUR_CHG_STATE)
	{
		cur_chg_start_cnt++;
		if(cur_chg_start_cnt > GC_DELAY_2S)
		{
			cur_chg_start_cnt = GC_DELAY_2S;
			sys_flags.flag.chg_state_flag = 1;
			if (sys_flags.flag.sys_comm_open_dch_flag ==0) //有充电电流 通信未打开输出
			{
				sys_flags.flag.adapter_connect_flag = 1;
				sys_flags.flag.open_dch_30s_end_flag = 1;
				if(protects.protect.forbidden_output == 1)
				{
					protects.protect.forbidden_output = 0;
				}
			}
		}
		
		if (sys_flags.flag.chg_state_flag == 1 && sys_flags.flag.adapter_connect_flag == 0)
		{
			dsg_to_chg_cnt++;
			if (dsg_to_chg_cnt >= 15000)
			{
				sys_flags.flag.adapter_connect_flag = 1;
				sys_flags.flag.sys_comm_open_dch_flag = 0;
				if(protects.protect.forbidden_output == 1)
				{
					protects.protect.forbidden_output = 0;
				}
				dsg_to_chg_cnt = 0;
			}
		}
		else
		{
			dsg_to_chg_cnt = 0;
		}
		
		
		if(Info.afe.CurCadc > ICUR_CHG_OCP)
		{
			cur_chg_ocp_cnt++;
			if(cur_chg_ocp_cnt > GC_DELAY_4S)
			{
				cur_chg_ocp_cnt = 0;
				protects.protect.chg_ocp_flag = 1;
				cur_chg_start_cnt = 0 ;
				sys_flags.flag.chg_state_flag = 0;
				sys_flags.flag.open_dch_30s_end_flag = 1;
				if(protects.protect.forbidden_output == 1)
				{
					protects.protect.forbidden_output = 0;
				}
				sys_flags.flag.sys_comm_open_dch_flag = 0;
			}
		}
		else
		{
			cur_chg_ocp_cnt = 0;
		}
	}
	else
	{
		cur_chg_ocp_cnt = 0;
		if(sys_flags.flag.chg_state_flag == 1)
		{
			if(cur_chg_start_cnt > 0)
			{
				cur_chg_start_cnt--;
			}
			else
			{
				sys_flags.flag.chg_state_flag = 0;
			}
		}
		else
		{
			cur_chg_start_cnt = 0;
		}
	}

	if(Info.afe.CurCadc < ICUR_DCH_STATE)
	{
		cur_dsg_start_cnt++;
		if(cur_dsg_start_cnt > GC_DELAY_2S)
		{
			cur_dsg_start_cnt = GC_DELAY_2S;
		
			flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
			if (tmp_16 == 0 && sys_flags.flag.sys_id_connect_flag == 1)
			{
				sys_flags.flag.sys_comm_open_dch_flag =1;
				sys_flags.flag.adapter_connect_flag =0;
				sys_states.state.sys_chg_ch_flag =0;
				sys_flags.flag.open_dch_30s_end_flag =1;
				sys_flags.flag.dch_state_flag = 1;
			}
			else
			{
				sys_flags.flag.dch_state_flag = 1;
			}
			
		}
				
		if(Info.afe.CurCadc < ICUR_DCH_OCP)
		{
			cur_dsg_ocp_cnt++;
			if(cur_dsg_ocp_cnt > GC_DELAY_4S)
			{
				cur_dsg_ocp_cnt = 0;
				protects.protect.dch_ocp_flag = 1;
				cur_dsg_start_cnt = 0;
				sys_flags.flag.dch_state_flag = 0;
			}
		}
		else
		{
			cur_dsg_ocp_cnt = 0;
		}
		
		if(Info.afe.CurCadc < ICUR_DCH_OCP2)
		{
			cur_dsg_ocp2_cnt++;
			if(cur_dsg_ocp2_cnt >GC_DELAY_02S)
			{
				cur_dsg_ocp2_cnt = 0;
				protects.protect.dch_ocp2_flag =1;
				cur_dsg_start_cnt = 0;
				sys_flags.flag.dch_state_flag = 0;
			}
		}
		else
		{
			cur_dsg_ocp2_cnt = 0;
		}
	}
	else
	{
		cur_dsg_ocp2_cnt =0;
		cur_dsg_ocp_cnt = 0;
		if(sys_flags.flag.dch_state_flag ==1)
		{
			if(cur_dsg_start_cnt > 0)
			{
				cur_dsg_start_cnt--;
			}
			else
			{
				sys_flags.flag.dch_state_flag =0;
			}
		}
		else
		{
			cur_dsg_start_cnt =0;
		}
	}	
	
	if (sys_flags.flag.dch_en_flag == 0) // 放电MOS管关闭  检测到电流
	{
		if(Info.afe.CurCadc < ICUR_DCH_MOS_ERR)
		{
			cur_dsg_mos_err_cnt++;
			if(cur_dsg_mos_err_cnt > GC_DELAY_8S)
			{
				cur_dsg_mos_err_cnt = 0;
				protects.protect.dch_mos_err_flag =1;
			}
		}
		else
		{
			cur_dsg_mos_err_cnt = 0;
		}
	}
	
	if (sys_flags.flag.chg_en_flag == 0) // 充电MOS管关闭  检测到电流
	{
		if(Info.afe.CurCadc > ICUR_CHG_MOS_ERR)
		{
			cur_chg_mos_err_cnt++;
			if(cur_chg_mos_err_cnt > GC_DELAY_8S)
			{
				cur_chg_mos_err_cnt = 0;
				protects.protect.chg_mos_err_flag =1;
			}
		}
		else
		{
			cur_chg_mos_err_cnt = 0;
		}
	}
	
	if (AFE.BSTATUS1.Bit.SC	== 1)
	{
		protects.protect.dch_scd_flag = 1;
		cur_dsg_start_cnt = 0;
		sys_flags.flag.dch_state_flag = 0;
		//sys_flags.flag.sys_comm_open_dch_flag =0;
	}
	
	if (AFE.BSTATUS1.Bit.OCD1 == 1)
	{
		cur_dsg_ocp2_cnt = 0;
		protects.protect.dch_ocp2_flag = 1;
		cur_dsg_start_cnt = 0;
		sys_flags.flag.dch_state_flag = 0;
	}
	
	if (AFE.BSTATUS1.Bit.OCD2 == 1)
	{
		cur_dsg_start_cnt = 0;
		sys_flags.flag.dch_state_flag = 0;
	}
	
	if (AFE.BSTATUS1.Bit.OCC == 1)
	{
		cur_chg_ocp_cnt = 0;
		protects.protect.chg_ocp_flag = 1;
		cur_chg_start_cnt = 0 ;
		sys_flags.flag.chg_state_flag = 0;
	}
	
}

/****************************************************************************
FUNCTION		: Temperature_Protect_Function
DESCRIPTION	: 高温 低温等温度相关 处理函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
 
void Temperature_Protect_Function()
{
	static u16 chg_otp_cnt =0;
	static u16 chg_ltp_cnt =0;
	static u16 dsg_otp_cnt =0;
	static u16 dsg_ltp_cnt =0;
	static u16 tp_open_cnt =0;
	static u16 tp_short_cnt =0;
	static u16 tp_dif_cnt =0;
	static u16 tp_mosfet_cnt =0;
	
	if(sys_flags.flag.adapter_connect_flag == 1)
	{
		if(Info.max_temp > NTC_CHG_OTP)
		{
			chg_otp_cnt++;
			if(chg_otp_cnt > GC_DELAY_5S)
			{
				chg_otp_cnt = GC_DELAY_5S;
				protects2.protect.chg_otp_flag = 1;
			}
		}
		else
		{
			if(protects2.protect.chg_otp_flag ==1)
			{
				if(Info.max_temp < NTC_CHG_OTP_CLEAR)
				{
					if(chg_otp_cnt > 0)
					{
						chg_otp_cnt--;
					}
					else
					{
						protects2.protect.chg_otp_flag = 0;
					}
				}
				else
				{
					chg_otp_cnt = GC_DELAY_5S;
				}
			}
			else
			{
				chg_otp_cnt =0;
			}
		}
		
		if(Info.min_temp < NTC_CHG_LTP)
		{
			chg_ltp_cnt++;
			if(chg_ltp_cnt > GC_DELAY_5S)
			{
				chg_ltp_cnt = GC_DELAY_5S;
				protects2.protect.chg_ltp_flag =1;
			}
		}
		else
		{
			if(protects2.protect.chg_ltp_flag ==1)
			{
				if(Info.min_temp > NTC_CHG_LTP_CLEAR)
				{
					if(chg_ltp_cnt > 0)
					{
						chg_ltp_cnt--;
					}
					else
					{
						protects2.protect.chg_ltp_flag = 0;
					}
				}
				else
				{
					chg_ltp_cnt = GC_DELAY_5S;
				}
			}
			else
			{
				chg_ltp_cnt =0;
			}
		}
	}
	else
	{
		protects2.protect.chg_otp_flag =0;
		chg_otp_cnt =0;
		protects2.protect.chg_ltp_flag =0;
		chg_ltp_cnt =0;
	}
	
	if (1)//sys_flags.flag.sys_comm_open_dch_flag == 1)
	{
		if(Info.max_temp > NTC_DCH_OTP)
		{
			dsg_otp_cnt++;
			if(dsg_otp_cnt > GC_DELAY_5S)
			{
				dsg_otp_cnt = GC_DELAY_5S;
				protects2.protect.dch_otp_flag = 1;
			}
		}
		else
		{
			if(protects2.protect.dch_otp_flag == 1)
			{
				if(Info.max_temp < NTC_DCH_OTP_CLEAR)
				{
					if(dsg_otp_cnt > 0)
					{
						dsg_otp_cnt--;
					}
					else
					{
						protects2.protect.dch_otp_flag = 0;
					}
				}
				else
				{
					dsg_otp_cnt = GC_DELAY_5S;
				}
			}
			else
			{
				dsg_otp_cnt =0;
			}
		}
		if(Info.min_temp < NTC_DCH_LTP)
		{
			dsg_ltp_cnt++;
			if(dsg_ltp_cnt > GC_DELAY_5S)
			{
				dsg_ltp_cnt = GC_DELAY_5S;
				protects2.protect.dch_ltp_flag = 1;
			}
		}
		else
		{
			if(protects2.protect.dch_ltp_flag == 1)
			{
				if(Info.min_temp > NTC_DCH_LTP_CLEAR)
				{
					if(dsg_ltp_cnt > 0)
					{
						dsg_ltp_cnt--;
					}
					else
					{
						protects2.protect.dch_ltp_flag = 0;
					}
				}
				else
				{
					dsg_ltp_cnt = GC_DELAY_5S;
				}
			}
			else
			{
				dsg_ltp_cnt = 0;
			}
		}
	}
	else
	{
		protects2.protect.dch_otp_flag =0;
		dsg_otp_cnt =0;
		protects2.protect.dch_ltp_flag =0;
		dsg_ltp_cnt =0;
	}
	

	
	if(Info.min_temp < NTC_OPEN)
	{
		tp_open_cnt++;
		if(tp_open_cnt > GC_DELAY_8S)
		{
			tp_open_cnt = GC_DELAY_8S;
			protects.protect.ntc_open_flag = 1;
		}
	}
	else
	{
		tp_open_cnt = 0;
	}
	
	if(Info.max_temp > NTC_SHORT)
	{
		tp_short_cnt++;
		if(tp_short_cnt > GC_DELAY_8S)
		{
			tp_short_cnt = GC_DELAY_8S;
			protects.protect.ntc_short_flag = 1;
		}
	}
	else
	{
		tp_short_cnt = 0;
	}	
	
	if(Info.max_temp - Info.min_temp > NTC_DIFF)
	{
		tp_dif_cnt++;
		if(tp_dif_cnt > GC_DELAY_8S)
		{
			tp_dif_cnt = GC_DELAY_8S;
			protects2.protect.temp_diff_flag = 1;
		}
	}
	else
	{
		if(protects2.protect.temp_diff_flag == 1)
		{
			if(Info.max_temp - Info.min_temp < NTC_DIFF_CLEAR)
			{
				if(tp_dif_cnt > 0)
				{
					tp_dif_cnt--;
				}
				else
				{
					protects2.protect.temp_diff_flag = 0;
				}
			}
			else
			{
				tp_dif_cnt = GC_DELAY_8S;
			}
		}
		else
		{
			tp_dif_cnt =0;
		}
	}

	if(Info.g_NTC1_DSGFET_T > NTC_MOSFET_ERR)
	{
		tp_mosfet_cnt++;
		if(tp_mosfet_cnt > GC_DELAY_8S)
		{
			tp_mosfet_cnt = GC_DELAY_8S;
			protects2.protect.mosfet_temp_err_flag = 1;
		}
	}
	else
	{
		if(protects2.protect.mosfet_temp_err_flag == 1)
		{
			if(Info.g_NTC1_DSGFET_T  < NTC_MOSFET_CLEAR)
			{
				if(tp_mosfet_cnt > 0)
				{
					tp_mosfet_cnt--;
				}
				else
				{
					protects2.protect.mosfet_temp_err_flag = 0;
				}
			}
			else
			{
				tp_mosfet_cnt = GC_DELAY_8S;
			}
		}
		else
		{
			tp_mosfet_cnt =0;
		}
	}

}
 
/****************************************************************************
FUNCTION		: MOS_Control_Function
DESCRIPTION	: 充电 放电 MOS 控制函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
 
void MOS_Control_Function()
{
	//更新MOS管状态
	if (AFE.CONF.Bit.CHGMOS == 1)
	{
		sys_flags.flag.chg_en_flag = 1;
	}
	else
	{
		sys_flags.flag.chg_en_flag = 0;
	}
	
	if (AFE.CONF.Bit.DSGMOS == 1)
	{
		sys_flags.flag.dch_en_flag = 1;
	}
	else
	{
		sys_flags.flag.dch_en_flag = 0;
	}
	
	if(sys_flags.flag.dch_state_flag == 1)
	{
		protects.protect.chg_full_flag =0;
		protects2.protect.chg_ltp_flag =0;
		protects.protect.chg_ocp_flag =0;
		protects2.protect.chg_otp_flag =0;
		protects.protect.chg_ovp_flag =0;
		chg_retry_cnt = 0;
	}
	
	if(sys_flags.flag.chg_state_flag == 1)
	{
		//sys_flags.flag.sys_comm_open_dch_flag =0;
		protects.protect.dch_lvp_flag =0;
		if (protects.protect.dch_lvp2_flag  == 1)
		{
			//sys_flags.flag.sys_close_flag = 0;	
		}
		protects.protect.dch_lvp2_flag =0;
		protects.protect.dch_empty_flag =0;
		protects2.protect.soc_lp_flag = 0;
		protects.protect.dch_scd_flag =0;
		protects.protect.dch_ocp_flag =0;
		protects.protect.dch_ocp2_flag =0;
		dsg_retry_cnt = 0;
		dsg2_retry_cnt = 0;
		dsg_sc_retry_cnt = 0;
	}
	 
	if(sys_flags.flag.chg_en_flag == 0)
	{
		if(
			(protects.protect.chg_full_flag ==0)&&
			(protects.protect.chg_ocp_flag ==0)&&
			(protects.protect.chg_ovp_flag ==0)&&
			(protects.protect.cell_dif_volt_flag ==0)&&
			(protects.protect.cell_low_2v_flag ==0)&&
			(protects2.protect.chg_ltp_flag ==0)&&
			(protects2.protect.chg_ltp2_flag ==0)&&
			(protects2.protect.chg_otp_flag ==0)&&
			(protects2.protect.chg_otp2_flag ==0)&&
			(protects2.protect.dch_ltp_flag ==0)&&
			(protects2.protect.dch_ltp2_flag ==0)&&
			(protects2.protect.dch_otp_flag ==0)&&
			(protects2.protect.dch_otp2_flag ==0)&&	
			(protects2.protect.temp_diff_flag == 0)&&
			(protects2.protect.mosfet_temp_err_flag == 0)&&
			//(protects.protect.dch_ocp_flag ==0)&&	
			//(protects.protect.dch_ocp2_flag ==0)&&	
			//(protects.protect.dch_scd_flag ==0)&&
			(protects.protect.ntc_open_flag == 0)&&
			(protects.protect.ntc_short_flag == 0)			
			)
		{   
			if((sys_flags.flag.sys_id_connect_flag ==1)&&(sys_flags.flag.open_dch_30s_end_flag ==0))//30S强制开放电
			{
				Chg_Control(OPEN_MOS);
			}
			
			if((sys_flags.flag.sys_id_connect_flag ==1)&&(sys_flags.flag.sys_comm_open_dch_flag ==1))//通信开放电
			{
				Chg_Control(OPEN_MOS);
			}
			
			if((sys_flags.flag.sys_id_connect_flag ==1)&&(sys_flags.flag.adapter_connect_flag == 1))//适配器在位开充电 测不到
			{
				Chg_Control(OPEN_MOS);
			}
/*
			if (sys_flags.flag.adapter_connect_flag == 1)
			{
				//CHG_EN_ON();
				//sys_flags.flag.chg_en_flag =1;
				if  (sys_flags.flag.adapter_connect_flag == 1 && sys_flags.flag.sys_id_connect_flag ==1 )
				{
					// 开充电管 和 放电管
					Dsg_Control(OPEN_MOS);
					Chg_Control(OPEN_MOS);
				}
				else
				{

					if((sys_flags.flag.sys_id_connect_flag ==1)&&(sys_flags.flag.open_dch_30s_end_flag ==0)&&(sys_flags.flag.dch_en_flag == 1))//30S强制开放电
					{
						Chg_Control(OPEN_MOS);
					}
				}
			}
			else
			{
				if (sys_flags.flag.dch_en_flag ==1 && sys_flags.flag.sys_id_connect_flag ==1 ) //此时在放电状态下 开充电
				{
					Chg_Control(OPEN_MOS);
				}
				
				//if( AFE.BSTATUS1.Bit.OV == 1 && sys_flags.flag.sys_id_connect_flag ==1 ) //AFE解除OV保护情况下 
				//{
					//Chg_Control(OPEN_MOS);
				//}
				
				if((sys_flags.flag.sys_comm_open_dch_flag ==1)&&(sys_flags.flag.dch_en_flag ==1))//通信开放电
				{
					Chg_Control(OPEN_MOS);
				}
			}
*/
		}
	}
	else
	{
		if(
			(protects.protect.chg_full_flag ==1)||
			(protects.protect.chg_ocp_flag ==1)||		
			(protects.protect.chg_ovp_flag ==1)||
			(protects.protect.cell_dif_volt_flag ==1)||
			(protects.protect.cell_low_2v_flag ==1)||		
			(sys_flags.flag.adapter_connect_flag == 0)||
			(sys_flags.flag.sys_id_connect_flag == 0)||
			(protects2.protect.chg_otp_flag ==1)||
			(protects2.protect.chg_otp2_flag ==1)||
			(protects2.protect.chg_ltp_flag ==1)||
			(protects2.protect.chg_ltp2_flag ==1)||
			(protects2.protect.dch_otp_flag ==1)||
			(protects2.protect.dch_otp2_flag ==1)||
			(protects2.protect.dch_ltp_flag ==1)||
			(protects2.protect.dch_ltp2_flag ==1)||
			(protects2.protect.temp_diff_flag == 1)||
			(protects2.protect.mosfet_temp_err_flag == 1)||
			//(protects.protect.dch_ocp_flag ==1)||
			//(protects.protect.dch_ocp2_flag ==1)||
			//(protects.protect.dch_scd_flag ==1)||
			(protects.protect.ntc_open_flag == 1)||
			(protects.protect.ntc_short_flag == 1)
			)
		{
				//CHG_EN_OFF();
			//sys_flags.flag.chg_en_flag =0;
			if (sys_flags.flag.sys_id_connect_flag == 1)
			{
				if (sys_flags.flag.adapter_connect_flag == 1)// 充电阶段
				{
					if(
						(protects2.protect.chg_otp_flag == 1)  ||
						(protects2.protect.chg_ltp_flag == 1) ||	
						(protects.protect.chg_full_flag ==1)||
						(protects.protect.chg_ocp_flag == 1)||
						(protects2.protect.temp_diff_flag == 1)||
						(protects2.protect.mosfet_temp_err_flag == 1)
						//(protects.protect.dch_ocp_flag ==1)||
						//(protects.protect.dch_ocp2_flag ==1)||
						//(protects.protect.dch_scd_flag ==1)
					)
					{
						Chg_Control(CLOSE_MOS);
					}
					if ( protects.protect.chg_ovp_flag == 1 && sys_flags.flag.dch_state_flag == 0)
					{
						Chg_Control(CLOSE_MOS);
					}
						
				}
				else //非充电  在放电 
				{
					if (sys_flags.flag.sys_comm_open_dch_flag == 0 && sys_flags.flag.open_dch_30s_end_flag == 1) //预放电结束没在充电 也没在放电
					{
						Chg_Control(CLOSE_MOS);
					}
															
				}
				
				if (	
						(protects2.protect.dch_otp2_flag == 1) ||
						(protects2.protect.dch_otp_flag == 1)  ||
						(protects2.protect.dch_ltp2_flag == 1) ||
						(protects2.protect.dch_ltp_flag == 1) ||
						(protects.protect.ntc_open_flag == 1)||
						(protects.protect.ntc_short_flag == 1)||
						(protects.protect.cell_dif_volt_flag ==1)||
						(protects.protect.cell_low_2v_flag ==1) ||
						(protects2.protect.temp_diff_flag == 1) ||
						(protects2.protect.mosfet_temp_err_flag == 1)
				)
				{
					Chg_Control(CLOSE_MOS);
				}
							
			} 
			else
			{
				Chg_Control(CLOSE_MOS);//关闭充电管
			}
		}
	}

	if(sys_flags.flag.dch_en_flag ==0)
	{
		if(
			//(protects.protect.dch_empty_flag ==0)&&
			(protects.protect.dch_scd_flag ==0)&&
			(protects.protect.dch_ocp_flag ==0)&&
			(protects.protect.dch_ocp2_flag ==0)&&
			(protects.protect.dch_lvp_flag ==0)&&
			(protects.protect.dch_lvp2_flag ==0)&&
			(protects2.protect.soc_lp_flag ==0)&&
			(protects.protect.cell_low_2v_flag ==0)&&
			(protects.protect.cell_dif_volt_flag ==0)&&
			(protects2.protect.dch_otp_flag ==0)&&
			(protects2.protect.dch_otp2_flag ==0)&&
			(protects2.protect.dch_ltp_flag ==0)&&
			(protects2.protect.dch_ltp2_flag ==0)&&		
			(protects2.protect.chg_ltp_flag ==0)&&
			(protects2.protect.chg_ltp2_flag ==0)&&
			(protects2.protect.chg_otp_flag ==0)&&
			(protects2.protect.chg_otp2_flag ==0)&&
			(protects2.protect.temp_diff_flag == 0)&&
			(protects2.protect.mosfet_temp_err_flag == 0)&&
			//(protects.protect.chg_ocp_flag ==0)	&&
			(protects.protect.ntc_open_flag == 0)&&
			(protects.protect.ntc_short_flag == 0)
			)
		{
				//DSG_EN_ON();
			if(sys_flags.flag.sys_comm_open_dch_flag ==0 )
			{
				if ((sys_flags.flag.sys_id_connect_flag ==1)&&(sys_flags.flag.open_dch_30s_end_flag ==0)) //id连接 && 30秒预放电未结束
				{
					if (protects.protect.chg_ovp_flag  == 0 )
					{
						Chg_Control(OPEN_MOS);
					}
					Dsg_Control(OPEN_MOS);
					protects.protect.chg_ocp_flag =0;
	
				}
			}
			else
			{
				if (sys_flags.flag.sys_id_connect_flag == 1)
				{
					Chg_Control(OPEN_MOS);
					Dsg_Control(OPEN_MOS);
					//打开充电
					//打开放电
				}
			}
			//sys_flags.flag.dch_en_flag =1;

			
			if (sys_flags.flag.chg_state_flag == 1 && sys_flags.flag.adapter_connect_flag == 1)
			{
				if (sys_flags.flag.sys_id_connect_flag == 1)
				{
					Dsg_Control(OPEN_MOS);
					//此时在充电 打开MOS管子
				}
			}
		}
	}
	else
	{
		if(
			(sys_flags.flag.sys_id_connect_flag ==0)||
			(sys_flags.flag.sys_comm_open_dch_flag == 0) ||
			//(protects.protect.dch_empty_flag)||
			(protects.protect.dch_scd_flag ==1)||
			(protects.protect.dch_lvp_flag ==1)||
			(protects.protect.dch_lvp2_flag ==1)||
			(protects2.protect.soc_lp_flag ==1)||
			(protects.protect.dch_ocp_flag ==1)||
			(protects.protect.cell_dif_volt_flag ==1)||
			(protects.protect.cell_low_2v_flag ==1)||
			(protects.protect.dch_ocp2_flag ==1)||
			(protects2.protect.dch_ltp2_flag ==1)||
			(protects2.protect.dch_otp2_flag ==1)||			
			(protects2.protect.dch_ltp_flag ==1)||
			(protects2.protect.dch_otp_flag ==1)||
			(protects2.protect.chg_otp_flag ==1)||
			(protects2.protect.chg_otp2_flag ==1)||
			(protects2.protect.chg_ltp_flag ==1)||
			(protects2.protect.chg_ltp2_flag ==1) ||
			(protects2.protect.temp_diff_flag == 1)||
			(protects2.protect.mosfet_temp_err_flag == 1)||
			//(protects.protect.chg_ocp_flag ==1)	||
			(protects.protect.ntc_open_flag == 1)||
			(protects.protect.ntc_short_flag == 1)
			)
		{
				//DSG_EN_OFF();
			//sys_flags.flag.dch_en_flag =0;
			if ((sys_flags.flag.sys_id_connect_flag ==1)&&(sys_flags.flag.open_dch_30s_end_flag ==0))// ID连接 && 预放电30S未结束
			{
				//30S强制开输出  发生温度 压差时不保护
				if(
				(protects.protect.dch_empty_flag == 1)||
				(protects.protect.dch_ocp2_flag == 1)||
				(protects.protect.dch_scd_flag == 1)||
				(protects.protect.cell_dif_volt_flag ==1)||
				(protects.protect.cell_low_2v_flag ==1)||
				//(protects.protect.dch_lvp_flag == 1)||
				(protects.protect.dch_lvp2_flag ==1) ||
				//(protects2.protect.soc_lp_flag ==1)||
				(protects.protect.dch_ocp_flag == 1)||
				(protects2.protect.temp_diff_flag == 1)||
				(protects2.protect.mosfet_temp_err_flag == 1)||
				//(protects.protect.chg_ocp_flag ==1)	||
				(protects.protect.ntc_open_flag == 1)||
				(protects.protect.ntc_short_flag == 1)
				)
				{
					Dsg_Control(CLOSE_MOS);
				}
						
			} 
			else
			{
				
				if (sys_flags.flag.open_dch_30s_end_flag == 1)//30秒强制放电结束
				{
					if (sys_flags.flag.sys_comm_open_dch_flag == 0 && sys_flags.flag.adapter_connect_flag == 0) //没在充电 也没在放电
					{
						Dsg_Control(CLOSE_MOS);
					}
					else
					{
						if(
							(protects.protect.dch_scd_flag ==1)||
							(protects.protect.dch_lvp_flag ==1)||
							(protects.protect.dch_lvp2_flag ==1)||
							(protects2.protect.soc_lp_flag ==1)||
							(protects.protect.dch_ocp_flag ==1)||
							(protects.protect.cell_dif_volt_flag ==1)||
							(protects.protect.cell_low_2v_flag ==1)||
							(protects.protect.dch_ocp2_flag ==1)||
							(protects2.protect.dch_ltp2_flag ==1)||
							(protects2.protect.dch_otp2_flag ==1)||
							(protects2.protect.dch_ltp_flag ==1)||
							(protects2.protect.dch_otp_flag ==1)||
							(protects2.protect.chg_otp_flag ==1)||
							(protects2.protect.chg_otp2_flag ==1)||
							(protects2.protect.chg_ltp_flag ==1)||
							(protects2.protect.chg_ltp2_flag ==1)||
							(protects2.protect.temp_diff_flag == 1)||
							(protects2.protect.mosfet_temp_err_flag == 1)||
							//(protects.protect.chg_ocp_flag ==1)	||
							(protects.protect.ntc_open_flag == 1)||
							(protects.protect.ntc_short_flag == 1)
						)
						{
							Dsg_Control(CLOSE_MOS);
						}
					}	
				}
		
												
				if(sys_flags.flag.sys_id_connect_flag == 0) 
				{
					Dsg_Control(CLOSE_MOS);
					//关闭放电
				}
								
			}
		}
	}
}
 
 
/****************************************************************************
FUNCTION		: Protect_Retry_Function
DESCRIPTION	: 充电过流三次恢复 等多次恢复的函数
INPUT			: None
OUTPUT			: None
*****************************************************************************/
 
void Protect_Retry_Function(void)
{
	static uint16_t chg_ocp_cnt = 0;
	static uint16_t dsg_ocp_cnt = 0;
	static uint16_t dsg2_ocp_cnt =0;
	static uint16_t dsg_scp_cnt =0;
			
	if (protects.protect.chg_ocp_flag == 1 && chg_retry_cnt <3)
	{
		chg_ocp_cnt++;
		if (chg_ocp_cnt > GC_DELAY_5S) //定时有待更新
		{
			if (chg_retry_cnt < 3)
			{
				protects.protect.chg_ocp_flag = 0;
				//sys_flags.flag.adapter_connect_flag = 0;
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 1;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				//UpdateStates();
				//if (AFE.BSTATUS1.Bit.OCC == 0)
				{
					chg_ocp_cnt = 0;
					chg_retry_cnt++;
					protects.protect.chg_ocp_flag =0;
					if (sys_flags.flag.adapter_connect_flag == 0)
					{
						sys_flags.flag.sys_id_connect_flag = 0;
						id_connect_cnt = 0;
					}
				}
				
			}

		}
	} 
	else
	{
		chg_ocp_cnt = 0;
	}
	 
	if((protects.protect.dch_ocp_flag == 1) && (dsg_retry_cnt <3))
	{
		dsg_ocp_cnt++;
		if(dsg_ocp_cnt > GC_DELAY_10S)//5S //定时有待更新
		{
			if(dsg_retry_cnt <3)
			{
				//Dsg_Control(CLOSE_MOS);
				delay_us(100);
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 1;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				//if(sys_flags.flag.open_dch_30s_end_flag == 1)
				//{
					//sys_flags.flag.sys_comm_open_dch_flag =1;
				//}
				//UpdateStates();
				
				//if (AFE.BSTATUS1.Bit.OCD1 == 0)
				{
					dsg_ocp_cnt = 0;
					dsg_retry_cnt++;
					protects.protect.dch_ocp_flag =0;
				}
			}
		}
	}
	else
	{
		dsg_ocp_cnt =0;
	}
	 
	if((protects.protect.dch_ocp2_flag == 1) && (dsg2_retry_cnt <3))
	{
		dsg2_ocp_cnt++;
		if(dsg2_ocp_cnt > GC_DELAY_10S)//5S
		{
			
			if(dsg2_retry_cnt <3)
			{
				//Dsg_Control(CLOSE_MOS);
				delay_us(100);
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 1;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				AFE.CONF.Bit.OCRC = 0;
				MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
				delay_ms(2);
				//if(sys_flags.flag.open_dch_30s_end_flag == 1)
				//{
					//sys_flags.flag.sys_comm_open_dch_flag =1;
				//}
				//UpdateStates();
				//if (AFE.BSTATUS1.Bit.OCD2 == 0)
				{
					dsg2_ocp_cnt = 0;
					dsg2_retry_cnt++;
					protects.protect.dch_ocp2_flag =0;
				}		
			}
		}
	}
	else
	{
		dsg2_ocp_cnt =0;
	}

	if((protects.protect.dch_scd_flag == 1) && (dsg_sc_retry_cnt <3))
	{
		dsg_scp_cnt++;
		if (dsg_sc_retry_cnt == 0)
		{
			if(dsg_scp_cnt > GC_DELAY_5S)//5S
			{
				if(dsg_sc_retry_cnt <3)
				{
					//Dsg_Control(CLOSE_MOS);
					delay_us(100);
					AFE.CONF.Bit.OCRC = 0;
					MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
					delay_ms(2);
					AFE.CONF.Bit.OCRC = 1;
					MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
					delay_ms(2);
					AFE.CONF.Bit.OCRC = 0;
					MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
					delay_ms(2);
					//if(sys_flags.flag.open_dch_30s_end_flag == 1)
					//{
					//sys_flags.flag.sys_comm_open_dch_flag =1;
					//}
					dsg_scp_cnt = 0;
					dsg_sc_retry_cnt++;
					protects.protect.dch_scd_flag =0;
				}
			}
		}
		else
		{
			if(dsg_scp_cnt > GC_DELAY_20S)//5S
			{
				if(dsg_sc_retry_cnt <3)
				{
					//Dsg_Control(CLOSE_MOS);
					delay_us(100);
					AFE.CONF.Bit.OCRC = 0;
					MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
					delay_ms(2);
					AFE.CONF.Bit.OCRC = 1;
					MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
					delay_ms(2);
					AFE.CONF.Bit.OCRC = 0;
					MTPWrite(MTP_CONF, 1, &AFE.CONF.Byte);
					delay_ms(2);
					//if(sys_flags.flag.open_dch_30s_end_flag == 1)
					//{
					//sys_flags.flag.sys_comm_open_dch_flag =1;
					//}
					dsg_scp_cnt = 0;
					dsg_sc_retry_cnt++;
					protects.protect.dch_scd_flag =0;
				}
			}
		}
		
	}
	else
	{
		dsg_scp_cnt =0;
	}
	 
}
 

uint16_t Find_MCU_Temp(uint16_t temp_val)
{
// 折半查表法，最多需要8次找到温度值
	uint16_t low = 0;
	uint16_t high = MCU_TEMP_DEGREE_RANGE - 1;
	uint16_t mid = 0 ;
	uint16_t mid_temp = 0;

	while(low < high)
	{
		mid = (low + high) >>1;

		if(temp_val == TEMP_VOLT_TABLE[mid])
		{break;}                               // 索引到刚好相等的值，则马上返回
		if(1 == (high - low))
		{
			mid = low;                 // 由于不是精确查找，若在小区间内，取小值
			break;
		}
		if(temp_val < TEMP_VOLT_TABLE[mid])
		{low = mid;}
		else
		{high = mid;}
	}
	
	if (mid == 0 || mid == 165)
	{
		mid = mid*10;
	}
	else
	{
		if (temp_val == TEMP_VOLT_TABLE[mid])
		{
			mid = mid*10;
		}
		else
		{
			mid_temp = mid;
			mid_temp = (TEMP_VOLT_TABLE[mid] - temp_val)*10 / (TEMP_VOLT_TABLE[mid] - TEMP_VOLT_TABLE[mid+1]);
			mid = mid*10 + mid_temp;
		}
	}
	

	return mid;
}

void Balance_Function(void)
{
	uint8_t i=0;
	blance_stick_1s_cnt++;
	if(blance_stick_1s_cnt > GC_DELAY_1S)
	{
		blance_stick_1s_cnt = 0;
		if((sys_cap.val.re_cap_rate >15)&&(sys_cap.val.re_cap_rate <20)&&(Info.max_temp <600))//SOC ?éóú15~20?ò???èD?óú60?? ?aê??????12?
		{
			if(sys_uarts.flag.blance_state_flag ==0)
			{
				if((Info.vcell_max - Info.vcell_min) >20)//?12?′óóú20mV
				{
					blance_cnt++;
					if(blance_cnt >10)
					{
						blance_cnt =0;
						for(i=0;i<16;i++)
						{
							cell_dif = Info.afe.VCell[i];
							if(cell_dif - Info.vcell_min >20)
							{
								cell_state |= (0x0001<<i);
							}
						}
						if(cell_state >0)
						{
							sys_uarts.flag.blance_state_flag =1;//?ùoa?a??±ê????????
						}
					}
				}
				else
				{
					blance_cnt =0;
				}
			}
			else
			{
				blance_cnt =0;
			}
		}
		else
		{
			blance_cnt =0;
		}
		 
		if(sys_cap.val.re_cap_rate >25)
		{
			if(sys_uarts.flag.blance_state_flag ==1)
			{
				balance_start = 1;
				if((u8)Info.max_temp >600)//???è?D??
				{
					sys_uarts.flag.blance_state_flag =0;
					cell_state =0;
					blance_cell_state =0;
					AFE.BALH = blance_cell_state>>8;
					MTPWrite(MTP_BALANCEH, 1, &AFE.BALH);
					delay_ms(5);
					AFE.BALL = blance_cell_state;
					MTPWrite(MTP_BALANCEL, 1, &AFE.BALL);
					blance_3_hour =0;
					return;
				}
			
				if(blance_cell_cnt ==0)//?a??êy?ú
				{
					blance_cell_state = (cell_state & 0xAAAA);
					if(blance_1_minute ==0)
					{
						AFE.BALH = blance_cell_state>>8;
						MTPWrite(MTP_BALANCEH, 1, &AFE.BALH);
						delay_ms(5);
						AFE.BALL = blance_cell_state;
						MTPWrite(MTP_BALANCEL, 1, &AFE.BALL);
					}
					blance_1_minute++;
					if(blance_1_minute >60)//1·??ó
					{
						blance_cell_cnt =1;
						blance_1_minute =0;
						blance_3_hour++;
					}
				}
				if(blance_cell_cnt ==1)//?a??êy?ú
				{
					blance_cell_state = (cell_state & 0x5555);
					if(blance_1_minute ==0)
					{
						AFE.BALH = blance_cell_state>>8;
						MTPWrite(MTP_BALANCEH, 1, &AFE.BALH);
						delay_ms(5);
						AFE.BALL = blance_cell_state;
						MTPWrite(MTP_BALANCEL, 1, &AFE.BALL);
					}
					blance_1_minute++;
					if(blance_1_minute >60)//1·??ó
					{
						blance_cell_cnt =0;
						blance_1_minute =0;
						blance_3_hour++;
					}
				}
				if(blance_3_hour >180)//3D?ê±
				{
					sys_uarts.flag.blance_state_flag =0;
					cell_state =0;
					blance_cell_state =0;
					AFE.BALH = blance_cell_state>>8;
					MTPWrite(MTP_BALANCEH, 1, &AFE.BALH);
					delay_ms(5);
					AFE.BALL = blance_cell_state;
					MTPWrite(MTP_BALANCEL, 1, &AFE.BALL);
					blance_3_hour =0;
					return;
				}
			}
		}
		else if(sys_cap.val.re_cap_rate <20 && balance_start == 1)
		{
			if(sys_uarts.flag.blance_state_flag ==1)
			{
				sys_uarts.flag.blance_state_flag =0;
				cell_state =0;
				blance_cell_state =0;
				AFE.BALH = blance_cell_state>>8;
				MTPWrite(MTP_BALANCEH, 1, &AFE.BALH);
				delay_ms(5);
				AFE.BALL = blance_cell_state;
				MTPWrite(MTP_BALANCEL, 1, &AFE.BALL);
				blance_3_hour =0;
				return;
			}
		}
		 
		 
	}
}

void Current_Cal_Function(void)
{
	static int16_t offset_value=0;
	static int16_t DSG_1A_value=0;
	int16_t adc_value_temp = 0;
	static uint16_t cal_count = 0;
	
	if (sys_flags.flag.com_current_cal == 1)
	{
		sys_flags.flag.com_current_cal = 0;		
		cadc_sum = 0;
		cadc_min = 0xffff;
		cadc_max = 0;	
		//校准测试
		//开充放电
		Dsg_Control(CLOSE_MOS);
		Chg_Control(CLOSE_MOS);
		delay_ms(250);
		count = 10;
		while ( count != 0)
		{
			AFE_Update_Function();
			wdt_feed(&WDT_0);
			if (AFE.BFLAG2.Bit.CADC_FLG == 1)
			{
				AFE.BFLAG2.Bit.CADC_FLG = 0;
				//cadc_temp[count] =(uint16_t)AFE.Cadc;
				cadc_sum += (uint16_t)AFE.Cadc;
				if (cadc_max < (uint16_t)AFE.Cadc)
				{
					cadc_max = (uint16_t)AFE.Cadc;
				}
								
				if (cadc_min >(uint16_t) AFE.Cadc)
				{
					cadc_min = (uint16_t)AFE.Cadc;
				}
				count--;
			}
		}
		cadc_sum = cadc_sum - cadc_min - cadc_max;
		cadc_sum >>= 3;
		offset_value = cadc_sum;
		//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_0A_ADC_ADDR,(uint8_t *)&cadc_sum, 2);
						
		cadc_sum = 0;
		cadc_min = 0xffff;
		cadc_max = 0;
		//关充放电
		Dsg_Control(OPEN_MOS);
		Chg_Control(OPEN_MOS);
		delay_ms(1000);
		wdt_feed(&WDT_0);
		delay_ms(1000);
		wdt_feed(&WDT_0);
		delay_ms(1000);
		wdt_feed(&WDT_0);
		delay_ms(1000);
		wdt_feed(&WDT_0);
		delay_ms(1000);
		wdt_feed(&WDT_0);
		delay_ms(1000);
		wdt_feed(&WDT_0);
		count = 10;
		while ( count != 0)
		{
			AFE_Update_Function();
			wdt_feed(&WDT_0);
			if (AFE.BFLAG2.Bit.CADC_FLG == 1)
			{
				AFE.BFLAG2.Bit.CADC_FLG = 0;
				//cadc_temp[count+16] = (uint16_t)AFE.Cadc;
				cadc_sum += (uint16_t)AFE.Cadc;
				if (cadc_max <(uint16_t) AFE.Cadc)
				{
					cadc_max =(uint16_t) AFE.Cadc;
				}
								
				if (cadc_min > (uint16_t)AFE.Cadc)
				{
					cadc_min =(uint16_t) AFE.Cadc;
				}
				count--;
			}
		}
		cadc_sum = cadc_sum - cadc_min - cadc_max;
		cadc_sum >>= 3;
		DSG_1A_value = cadc_sum;

		//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_1A_ADC_ADDR,(uint8_t *)&cadc_sum, 2);
		
		//flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_0A_ADC_ADDR, (uint8_t *)&offset_value, 2);
		//flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_1A_ADC_ADDR, (uint8_t *)&DSG_1A_value, 2);
		adc_value_temp = DSG_1A_value - offset_value;
		if (adc_value_temp < 0)
		{
			adc_value_temp = 0 - adc_value_temp;
		}
		if (adc_value_temp < 65  && adc_value_temp > 45)  //正常数值 55
		{
			sys_flags.flag.current_calibration = 1 ;
			flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_1A_ADC_ADDR,(uint8_t *)&DSG_1A_value, 2);
			flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_0A_ADC_ADDR,(uint8_t *)&offset_value, 2);
		}		
		
		cal_count = 200;
		while(cal_count)
		{
			cal_count--;
			wdt_feed(&WDT_0);
			AFE_Update_Function();
			Comm_Proc();
		}
		
		
		Dsg_Control(CLOSE_MOS);
		Chg_Control(CLOSE_MOS);

	}
}


void flash_save_data(uint32_t address,uint8_t index,uint32_t data)
{
	uint8_t write_buff[64] = {0};
	if (index > 7)
	{
		return;
	}
	// 判断是否需要擦除
	if (index == 0 || index == 4)
	{
		//擦除指令
		flash_erase(&FLASH_0,address+index*64,4);
	}
	
	*((uint32_t *)write_buff) = data;
	*((uint32_t *)(write_buff+4)) = data;
	//check sum  这里偷懒 只有一个数值  
	*((uint32_t *)(write_buff+56)) = *((uint32_t *)write_buff) + *((uint32_t *)(write_buff+4)) ;
	*((uint32_t *)(write_buff+60)) = 0xAAAAAAAA;
	flash_write(&FLASH_0, address+index*64,write_buff, 64);
}
void flash_read_data(uint32_t address,uint8_t * index,uint32_t * data)
{
	uint8_t read_buff[64] = {0};
	uint8_t i;
	for (i=0;i<8;i++)
	{
		flash_read(&FLASH_0, address+i*64+60,read_buff, 4);
		if (*((uint32_t *)read_buff)  == 0xffffffff)
		{
			*index = i;
			break;
		}
	}
	if (i == 8) //上面数据全满 没找到0xaaaaaaaa
	{
		*index = 0;
		i = 0;
	}
	if (i == 0)
	{
		flash_read(&FLASH_0, address+7*64,read_buff, 64);
		if (*((uint32_t *)(read_buff+56)) == (*((uint32_t *)read_buff) + *((uint32_t *)(read_buff+4))))
		{
			*data = *((uint32_t *)(read_buff));
		}
		else
		{
			//check没有过 读上一个块区的最后一帧
			flash_read(&FLASH_0, address+3*64,read_buff, 64);
			if (*((uint32_t *)(read_buff+56)) == (*((uint32_t *)read_buff) + *((uint32_t *)(read_buff+4))))
			{
				*data = *((uint32_t *)(read_buff));
			}
			else
			{
				*data = 0xffffffff;
			}
		}
	}
	else
	{
		flash_read(&FLASH_0, address+(i-1)*64,read_buff, 64);
		if (*((uint32_t *)(read_buff+56)) == (*((uint32_t *)read_buff) + *((uint32_t *)(read_buff+4))))
		{
			*data = *((uint32_t *)(read_buff));
		}
		else
		{
			//check没有过 读上一个块区的最后一帧
			if (i<4)
			{
				flash_read(&FLASH_0, address+7*64,read_buff, 64);
			}
			else
			{
				flash_read(&FLASH_0, address+3*64,read_buff, 64);
			}
			
			if (*((uint32_t *)(read_buff+56)) == (*((uint32_t *)read_buff) + *((uint32_t *)(read_buff+4))))
			{
				*data = *((uint32_t *)(read_buff));
			}
			else
			{
				*data = 0xffffffff;
			}
		}
	}
	
}


void Can_send_spi_flash(void)
{
	uint32_t i=0;
	uint32_t j=0;
	uint8_t send_buff[128];
	struct  can_message msg;
	
	if (sys_flags.flag.read_spi_flash_flag == 1 )
	{
		sys_flags.flag.read_spi_flash_flag = 0 ;
		
		for (i=0;i<spi_read_count;i++)
		{
			SPI_Read_Cont(0+i*128,128,send_buff);
			for (j=0;j<16;j++)
			{
				msg.id = 0xAAAAAA;
				msg.type = CAN_TYPE_DATA;
				msg.data = send_buff+j*8;
				msg.len  = 8;
				msg.fmt  = CAN_FMT_EXTID;
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
				}
			}
			wdt_feed(&WDT_0);
			
		}
		__NOP();
		
	}
}
 
 

void Can_Check_Init(void)
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
}