/*
 * File:   UART.c
 * Author: AID
 *
 * 16Q UART ������
 * 
 * Created on 2016��9��22��, ����5:46
 */
/****************************************************************************
 include files
*****************************************************************************/
#include "soft_soc.h"
#include "i2c_sh367309.h"
#include "stdlib.h"
#include "user_bms.h"
/****************************************************************************
Private variables 
*****************************************************************************/
extern uint16_t cc;
extern uint16_t vbat;
volatile SystemCap  sys_cap;
u8 first_soc_cnt=0;
u8 deta_cap_cnt=0;
uint16_t re_cap_tmp;
s16 ov_cap_val =0;
uint16_t bat_cycle_tmp = 3000;
u8  power_first_flag =0;
uint16_t update_delay =0;
//extern u16 min_temp;
//extern s16 min_temp2;

extern uint16_t max_temp;
extern uint16_t min_temp;
#define  TEMP_10C  10      //10���¶�
#define  CELL_FULL  3340//99%

uint32_t sleep_cnt =0;
uint16_t run_cap = 0;
u8 deta_cap;
u8 save_cap_tmp;//5^����һ��

uint8_t re_cap_rate_write_index = 0;
uint8_t chg_cap_cnt_write_index = 0;
uint8_t bat_cycle_cnt_write_index = 0;

  const uint16_t  Cell_volt_20c[36]=
 {
	/*0*/   2838,2948,3032,3095,3139,3178,3184,3188,3191,3195,
	/*10*/  3199,3203,3207,3211,3215,3219,3223,3227,3231,3235,
	/*20*/  3240,3244,3248,3251,3255,3259,3262,3266,3269,3272,
	/*30~35*/  3275,3278,3280,3283,3285,3289
 };
 
 const uint16_t  Cell_volt_0c[36]=
 {
	/*0*/   2787,2897,2992,3060,3108,3150,3160,3166,3172,3178,
	/*10*/  3184,3190,3195,3200,3205,3210,3215,3219,3223,3227,
	/*20*/  3231,3235,3239,3243,3246,3249,3253,3256,3259,3261,
	/*30~35*/  3264,3267,3269,3272,3274,3277
 };
 
 const uint16_t  low_temp_fcc[6]=
 {
	/*-20*/   65,
	/*-10*/   85,
	/*0*/   	90,
	/*10*/  	95,
	/*25*/  	100,
	/*25+*/  	100
 };
 
 //�����SOC��ȱ��һ�������Ժĵ����ֵ����Ҫ�������ߺ�����
 
/****************************************************************************
FUNCTION		: SOC
DESCRIPTION		: ????(???)
INPUT			: None
OUTPUT			: None
NOTICE			: TMIER????
DATE			: 2016/06/24
*****************************************************************************/
void Soc_AddSub(void)  //chh 20190909 ���Ӵ������²�������
{
	 //static s8 temp_cap_buf =0;
	 int tmp_cap=0;
	 static int temp_cap_buf = 0;
		uint32_t cur;
	 uint16_t tmp_16;
	 
	 if(Info.afe.CurCadc >0x8000)
	 {
			tmp_16 = 0-Info.afe.CurCadc;
			cur = (u32)tmp_16;
			sys_cap.val.cap_cnt-=cur;
	 }
	 else
	 {
			cur = (u32)Info.afe.CurCadc;
			sys_cap.val.cap_cnt+=cur;
	 }
	 
	 tmp_cap=(int)(sys_cap.val.cap_cnt/CAP_CNT_VAL);  
	 sys_cap.val.cap_cnt=sys_cap.val.cap_cnt-(long)(tmp_cap)*CAP_CNT_VAL;
			
		
	 if(tmp_cap>-30)
	 {
		 
		if(Info.min_temp < NTC_DSG_REV)
		{
			if(tmp_cap<1)
			{
				temp_cap_buf+= tmp_cap;
			}
			else
			{
				temp_cap_buf =0;
			}
			if(Info.min_temp < NTC_CHG_LTP_CLEAR)
			{
				if(Info.min_temp < NTC_DCH_LTP_CLEAR)
				{
					if(temp_cap_buf < -5)
					{
						tmp_cap = tmp_cap-3;
						temp_cap_buf =0;
					}
				}
				else
				{
					if(temp_cap_buf < -6)
					{
						tmp_cap = tmp_cap-2;
						temp_cap_buf =0;
					}
				}
			}
			else
			{
				if(temp_cap_buf < -12)
				{
					tmp_cap = tmp_cap-1;
					temp_cap_buf =0;
				}
			}
		}

		 
		 
		if(sys_cap.val.bat_cap >10)//ʣ������
		{
				sys_cap.val.bat_cap+=tmp_cap; 
		}
		else
		{
				if(tmp_cap>0)
				{
					sys_cap.val.bat_cap+=tmp_cap;
				}
		}
	 }
	 //��������
	 if(sys_flags.flag.adapter_connect_flag == 0)//�ŵ��������� ֻ������
	 {
		 if(sys_uarts.flag.cap_update_sub_flag == 1)
		 {
				 tmp_16 = 20*deta_cap;
				 if(sys_cap.val.bat_cap >tmp_16)
				 {
						sys_cap.val.bat_cap-= tmp_16;
				 }
				 sys_uarts.flag.cap_update_sub_flag =0;
		 }
		 else 
		 {
				deta_cap =0;//���ڲ�����ʱ��,��֤��ֵΪ0
		 }
	 }
	 else//����������� ֻ������
	 {
		 if(sys_uarts.flag.cap_update_add_flag == 1)
		 {
				 tmp_16 = 20*deta_cap;
				 if(sys_cap.val.bat_cap <(sys_cap.val.full_cap - tmp_16))
				 sys_cap.val.bat_cap+=tmp_16;
				 sys_uarts.flag.cap_update_add_flag =0;
		 }
		 else 
		 {
				deta_cap =0;//���ڲ�����ʱ��,��֤��ֵΪ0
		 }
	 }
	 
	 
	 if((protects.protect.chg_ovp_flag == 0)&&(protects.protect.chg_full_flag ==0))
	 {
			 if(sys_cap.val.bat_cap >= (sys_cap.val.full_cap - 5))
			 {
					sys_cap.val.bat_cap-=1;
					ov_cap_val += 1;
			 }
	 }
		
}

u8 VbatToSoc(u16 bat_val)
{
		uint8_t i;
		if(Info.min_temp >TEMP_10C)
		{
			for(i=0;i<31;i++)
			{
					if(bat_val < Cell_volt_20c[i])
					{
							return i;
					}
			}
		}
		else
		{
			for(i=0;i<31;i++)
			{
				if(bat_val < Cell_volt_0c[i])
				{
						return i;
				}
			}
		}
		return 31;
}
/****************************************************************************
FUNCTION		: TempCompensationSOC
DESCRIPTION		: �¶Ȳ���
INPUT			: None
OUTPUT			: None
NOTICE			:  �����¶�Ӱ����������������,�ٸ�����ʱ��ʾ��ʣ��������
�ڲ���Ȼʹ��ʣ���������ⲿ��ʾΪʣ���������
DATE			: 2020/02/17
*****************************************************************************/
void TempCompensationSOC(void)
{
		static uint16_t low_temp_full_cap =0;
		uint8_t i;
		for(i=0;i<5;i++)
		{
				if(min_temp <low_temp_fcc[i])
				{
						break;
				}
		}
		low_temp_full_cap = (u32)sys_cap.val.full_cap*low_temp_fcc[i]/100;
		sys_cap.val.low_temp_bat_cap = (u32)low_temp_full_cap*sys_cap.val.re_cap_rate/100;
		sys_cap.val.re_cap_rate = (u32)sys_cap.val.low_temp_bat_cap*100/ low_temp_full_cap;
		
}

/****************************************************************************
FUNCTION		: NormalCapacityProc
DESCRIPTION		: ���������Լ���ѹ����
INPUT			: None
OUTPUT			: None
NOTICE			:
DATE			: 2018/01/31
*****************************************************************************/
void NormalCapacityProc(void)      //����
{
		uint32_t capacity_volt;        //��·��ѹ
		static uint32_t full_cap_tmp = 0;
		static uint8_t full_cap_add_flag = 0;
		static uint16_t full_cap_count = 0;
		uint32_t re_cap_rate_temp = 0;
		static uint8_t powerup_cap_add_flag = 0;
		static uint32_t powerup_cap_tmp = 0;
		
//		if(sys_design.val.shutdown_soc >0)//Sean Add 20200327
//		{
//			power_first_flag =1;
//			sys_cap.val.re_cap_rate = sys_design.val.shutdown_soc;
//		}
		if(power_first_flag==0)
		{
				//capacity_volt = vbat;
				capacity_volt = Info.vcell_min;//capacity_volt/12;//*383/2500;
				if(Info.min_temp >TEMP_10C)
				{
						if(capacity_volt <Cell_volt_20c[30])
						{
								re_cap_rate_temp = VbatToSoc((u16)capacity_volt);//SOC
						}
						else
						{
								if(capacity_volt<CELL_FULL)
								{
										re_cap_rate_temp = 60;//ȡ�м�ֵ,�ȴ�У׼
								}
								else
								{
										re_cap_rate_temp = 99;
								}
						}
				}
				else
				{
						if(capacity_volt <Cell_volt_0c[30])
						{
								re_cap_rate_temp = VbatToSoc((u16)capacity_volt);
						}
						else
						{
								if(capacity_volt<CELL_FULL)
								{
										re_cap_rate_temp = 60;//ȡ�м�ֵ,�ȴ�У׼
								}
								else
								{
										re_cap_rate_temp = 99;
								}
						}
				}
				if(first_soc_cnt ==0)
				{
						re_cap_tmp = re_cap_rate_temp;
				}
				else
				{
						re_cap_tmp += re_cap_rate_temp;
				}
				first_soc_cnt++;
				if(first_soc_cnt >3)
				{
						re_cap_tmp >>=2;
						//re_cap_tmp �Ǵ˴��ϵ��OCV���ֵ
						if ( sys_cap.val.re_cap_rate <= 100) //����SOCֵ
						{
							//�Ա�������С
							if ( re_cap_tmp > sys_cap.val.re_cap_rate && re_cap_tmp < 30 ) //С��30%
							{
								powerup_cap_tmp = re_cap_tmp - sys_cap.val.re_cap_rate;
								powerup_cap_tmp = (u16)powerup_cap_tmp * (sys_cap.val.full_cap/100); //������ֵ
								powerup_cap_add_flag = 1;
							}
						}
						else //��һ���ϵ�  flashҲûSOCֵ
						{
							sys_cap.val.re_cap_rate = re_cap_tmp;
						}
						
				
						sys_cap.val.bat_cap = (u16)sys_cap.val.re_cap_rate * (sys_cap.val.full_cap/100); //aid 20200217 �޸�Ϊ��ǰ��������
						power_first_flag=1;
						first_soc_cnt =0;
						deta_cap_cnt = sys_cap.val.re_cap_rate;
						save_cap_tmp = sys_cap.val.re_cap_rate;//ˢ�±�������
				}
		 }
		 else
		 {			
				//AID  20180710  ���Ӳ���
				
				//�������
				if (AFE.BFLAG2.Bit.CADC_FLG == 1)
				{
					AFE.BFLAG2.Bit.CADC_FLG = 0;
					Soc_AddSub();
					Cap_Update_Check();//����Ƿ�������������͸���
					//���׾��Ȳ���
					//if (soc_count_start_flag == 1)
					//{
					//soc_count++;
					//if (soc_count >= 4)
					//{
					//soc_count_start_flag = 2;
					//soc_count = 0;
					//}
					//}
					//
					//if (soc_count_start_flag == 2)
					//{
					//soc_count++;
					//Soc_AddSub();
					//if (soc_count == (4*60*6))
					//{
					//soc_count = 0;
					//}
					//}
					
					//�������� ������ܼ� 12 + 3 mA   �������3mA
					if (Info.afe.CurCadc == 0)
					{
						if (AFE.BSTATUS3.Bit.DSG_FET == 1)
						{
							run_cap += 15;
						}
						else
						{
							run_cap += 3;
						}
						
						if (run_cap > CAP_CNT_VAL)
						{
							run_cap = run_cap - CAP_CNT_VAL;
							if(sys_cap.val.bat_cap > 0)
							{
								sys_cap.val.bat_cap -=1;
							}
						}
					}
					
					//���䲹��
					if ( (sys_flags.flag.chg_state_flag == 1)&&(Info.afe.CurCadc < 7000) )
					{
						if (Info.vcell_max > 3500 && full_cap_add_flag == 0)
						{
							full_cap_count++;
							if (full_cap_count >= 40 ) //10s
							{
								full_cap_add_flag = 1;
								full_cap_tmp = sys_cap.val.full_cap - sys_cap.val.bat_cap;
							}
						}
						else
						{
							full_cap_count = 0;
						}
			
						if (full_cap_add_flag == 1)
						{
							if (full_cap_tmp > 0)
							{
								full_cap_tmp -= 1;
								if (sys_cap.val.bat_cap < (sys_cap.val.full_cap-20))
								{
									sys_cap.val.bat_cap  += 1;
								}
							}
							else
							{
								full_cap_add_flag = 0;
							}
						}
					}
					else
					{
						full_cap_count = 0;
						full_cap_add_flag =0;
					}
		
					//�ϵ�OCVУ׼����
					if (powerup_cap_add_flag == 1)
					{
						if (sys_flags.flag.chg_state_flag == 1)
						{
							if (powerup_cap_tmp > 0)
							{
								powerup_cap_tmp -= 1;
								if (sys_cap.val.bat_cap < (sys_cap.val.full_cap-20))
								{
									sys_cap.val.bat_cap  += 1;
								}
							}
							else
							{
								powerup_cap_add_flag =0;
					
							}
						}
						else if (sys_flags.flag.dch_state_flag == 1)
						{
							//�ŵ�̬  ���� ������
							powerup_cap_add_flag = 0;
						}
					}
					
					
				}
								
				//FullCap_Update();//����Ƿ����������������
				

				if(protects.protect.dch_lvp_flag==1)
				{
						sys_cap.val.bat_cap = 0;
				}
				if((protects.protect.chg_ovp_flag == 1)||(protects.protect.chg_full_flag ==1))
				{
						if(ov_cap_val == 0)
						{
								ov_cap_val = sys_cap.val.bat_cap - sys_cap.val.full_cap;
						}
						sys_cap.val.bat_cap = sys_cap.val.full_cap;
				}
				capacity_volt = sys_cap.val.bat_cap*100;
				sys_cap.val.re_cap_rate = (capacity_volt/ sys_cap.val.full_cap);
				BatCycleProc();
				
		}
			
		
		
}

/****************************************************************************
FUNCTION		: Cap_Update_Check
DESCRIPTION		: �����������³���
INPUT			: None
OUTPUT			: None
NOTICE			: ���þ��ó���10����,������������
DATE			: 2017/06/24
*****************************************************************************/

void Cap_Update_Check(void)
{
		uint32_t capacity_volt;
		static uint8_t sleep_soc_hour =0;
		uint16_t cur_tmp =0;
		if(Info.afe.CurCadc >0x8000)
		{
				cur_tmp = Info.afe.CurCadc;
				cur_tmp = 0-cur_tmp;
		}
		else
		{
				cur_tmp = Info.afe.CurCadc;
		}
		if((cur_tmp <CUR_DSG_050mA)&&(sys_flags.flag.adapter_connect_flag ==0))//&&(sys_flags.flag.sys_comm_open_dch_flag ==1))
		{
				update_delay++;
				if(update_delay >2400) //���Խ����ӳ�    250ms����һ��   2400*250 = 10min  
				{
						update_delay = 0;
						//capacity_volt = vbat;
						capacity_volt = Info.vcell_min;//capacity_volt/12;//*383/2500;
						if(Info.min_temp >TEMP_10C)
						{
								if(capacity_volt <Cell_volt_20c[30])
								{
										sys_cap.val.re_cap_rate_volt = VbatToSoc((u16)capacity_volt);
								}	
								else
								{
										if(capacity_volt>CELL_FULL)
										{
												sys_cap.val.re_cap_rate_volt = 99;
										}
										else
										{
												sys_cap.val.re_cap_rate_volt = sys_cap.val.re_cap_rate;//35~98������
										}
								}
						}
						else
						{
								if(capacity_volt <Cell_volt_0c[30])
								{
										sys_cap.val.re_cap_rate_volt = VbatToSoc((u16)capacity_volt);
								}
								else
								{
										if(capacity_volt>CELL_FULL)
										{
												sys_cap.val.re_cap_rate_volt = 99;
										}
										else
										{
												sys_cap.val.re_cap_rate_volt = sys_cap.val.re_cap_rate;//35~98������
										}
								}
						}
						if(sys_cap.val.re_cap_rate_volt > sys_cap.val.re_cap_rate)
						{
								sys_uarts.flag.cap_update_sub_flag =0;
//								deta_cap = sys_cap.val.re_cap_rate_volt - sys_cap.val.re_cap_rate;
//								if(deta_cap >3)
//								{
//										sys_uarts.flag.cap_update_add_flag =1;
//								}
						}
						else
						{
								sys_uarts.flag.cap_update_add_flag =0;
								deta_cap = sys_cap.val.re_cap_rate - sys_cap.val.re_cap_rate_volt;		
								if(deta_cap >2)
								{
										sys_uarts.flag.cap_update_sub_flag =1;
								}
						}
				}
		}
		else
		{
				update_delay =0;
				sys_cap.val.re_cap_rate_volt = sys_cap.val.re_cap_rate;
		}
		//��ʱ���߻���У׼
		if(sleep_cnt >3600)//��ʱ����1S 3600�� =3600S = 1Сʱ  ƽ�����߹��� = 200uA������1Сʱ��ʧ0.2mAH
		{
				sleep_cnt = sleep_cnt -3600;
				sleep_soc_hour +=1;
			  if(sleep_soc_hour == 5)
				{
						sleep_soc_hour =0;
						if(sys_cap.val.bat_cap >0)
						{
								sys_cap.val.bat_cap -=1;
						}
				}
		}		
}
void FullCap_Update(void)
{
		static uint8_t fcc_delay_cnt =0;
		uint32_t fcc_tmp;
		static uint16_t wait_save_cnt =0;
	  static uint16_t time_cnt = 0;
	  static uint8_t fcc_save = 0;			//6.11AID
		uint16_t cur_tmp = 0;
		sys_states.state.fcc_power_on_flag =1;
	  sys_cap.val.ocv_print = ov_cap_val;
		if(sys_states.state.fcc_power_on_flag ==1)
		{
				if(sys_cap.val.re_cap_rate <30) //6.11�޸�     //((power_first_flag ==1)&&(sys_cap.val.re_cap_rate <30))  //30
				{
						//if((max_temp <35)&&(min_temp >10))// 10~35��
						//{
								if((sys_flags.flag.chg_en_flag ==0)&&(sys_flags.flag.dch_en_flag == 0))
								{
										if((sys_states.state.soc_fcc_save_flag ==0)&&(fcc_save == 0))
										{
												wait_save_cnt++;
												if(wait_save_cnt>20)
												{
													wait_save_cnt =0;
													sys_states.state.soc_fcc_save_flag = 1;
													sys_cap.val.bat_cap_fcc = sys_cap.val.bat_cap;
													sys_cap.val.re_cap_rate_fcc = sys_cap.val.re_cap_rate;
												}
										}
								}
								else
								{
										wait_save_cnt =0;
								}
						//}
						//else
						//{
							//	wait_save_cnt =0;
					//	}

				}
				else
				{
					wait_save_cnt =0;
				}
		}
		else
		{
				wait_save_cnt =0;
		}
		if(sys_states.state.soc_fcc_save_flag ==1)//&&(sys_flags.flag.sys_close_flag ==1))
		{
				 
			   if(Info.afe.CurCadc>0x8000)
				 {
					 cur_tmp = 0-Info.afe.CurCadc;
				 }
			   if((Info.vcell_min>3340)&&(cur_tmp<CUR_CHG_500mA))      //��ӱ�־λ
				 {
					  sys_states.state.soc_fcc_1_flag = 0;  //6.9AID
					  time_cnt =0;
						fcc_delay_cnt++;
						if(fcc_delay_cnt >200)  //10S
						{
								fcc_delay_cnt =0;
								if((sys_cap.val.bat_cap - sys_cap.val.bat_cap_fcc) > 5000)  //5AH
								{
									  sys_states.state.soc_fcc_2_flag = 0;  //6.9AID									
									  fcc_save = 1; 		//6.11AID�������Ƹ��º��־λ����
										fcc_tmp = sys_cap.val.bat_cap - sys_cap.val.bat_cap_fcc +ov_cap_val;
										fcc_tmp = fcc_tmp*100;
										fcc_tmp = fcc_tmp/(98 - sys_cap.val.re_cap_rate_fcc);
										//sys_cap.val.fcc_tmp_print = fcc_tmp;
										if(fcc_tmp >  sys_cap.val.full_cap)
										{
											  sys_states.state.soc_fcc_3_flag = 0;  //6.9AID
												if((fcc_tmp - sys_cap.val.full_cap) < 2000)//2000mAH
												{
														//  +w AP_Flash_Write(fcc_tmp,(FLASH_CAP_ADDR+1 + sys_cap.val.fcc_index)); //sys_cap.val.fcc_index
														sys_cap.val.fcc_index ++;
														if(sys_cap.val.fcc_index >3)
														{
																sys_cap.val.fcc_index =0;
														}
														// +w AP_Flash_Write(sys_cap.val.fcc_index,FLASH_CAP_ADDR);
														sys_states.state.soc_fcc_4_flag= 1;  //6.9AID
														//�����ۼӺͣ��������FCC��ֱ�Ӹ���
														sys_cap.val.full_cap = fcc_tmp;
														sys_states.state.soc_fcc_save_flag =0;
												}
												else
												{
														//protects2.protect.sys_fcc_err_flag =1;
												}
										}
										else
										{
											  sys_states.state.soc_fcc_3_flag = 1;  //6.9AID
												if((sys_cap.val.full_cap - fcc_tmp) < 2000)//1000mAH
												{
														// +w AP_Flash_Write(fcc_tmp,(FLASH_CAP_ADDR+1 + sys_cap.val.fcc_index));//sys_cap.val.fcc_index
														sys_cap.val.fcc_index ++;
														if(sys_cap.val.fcc_index >3)
														{
																sys_cap.val.fcc_index =0;
														}
														// +w AP_Flash_Write(sys_cap.val.fcc_index,FLASH_CAP_ADDR);
														//�����ۼӺͣ��������FCC��ֱ�Ӹ���
														//sys_cap.val.full_cap = ......
														sys_cap.val.full_cap = fcc_tmp;
												}
												else
												{
														//protects2.protect.sys_fcc_err_flag =1;
												}
										}
								}
								else
								{
									  sys_states.state.soc_fcc_2_flag = 1;  //6.9AID
										sys_states.state.soc_fcc_save_flag =0;
										//protects2.protect.sys_fcc_err_flag =1;
								}
						}
				}
				 else
				 {
					sys_states.state.soc_fcc_1_flag = 1;  //6.9AID
					fcc_delay_cnt =0;
					if((cur_tmp<CUR_CHG_500mA)&&(sys_flags.flag.chg_state_flag ==0))			//������С���Ҳ����ڳ��״̬�򲻽��и��º����ݵĴ洢���رմ洢��־λ
					{
							time_cnt++;
							if(fcc_save == 1)             //��ʱ�رձ�־λ����ֹ��ֹ״̬�±�־λ����
							{
									time_cnt =0;
									sys_states.state.soc_fcc_save_flag = 0;
							}
					}
					else
					{
							time_cnt =0;
					}
				}
				
		}
		else
		{
				fcc_delay_cnt =0;
		}			
		
}
void BatCycleProc(void)
{
		if(sys_flags.flag.chg_state_flag ==1)
		{
				if(sys_cap.val.re_cap_rate > deta_cap_cnt)
				{
						if(sys_cap.val.re_cap_rate - deta_cap_cnt <10)
						{
								sys_cap.val.chg_cap_cnt += sys_cap.val.re_cap_rate - deta_cap_cnt;
						}
						deta_cap_cnt = sys_cap.val.re_cap_rate;
						if(sys_cap.val.chg_cap_cnt >80)
						{
								sys_cap.val.chg_cap_cnt =0;
								__disable_irq();
								// +w AP_Flash_Write(0,FLASH_CHGCAP_ADDR);
								//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_CHGCAP_ADDR, (uint8_t *)&sys_cap.val.chg_cap_cnt, 1);	
								flash_save_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,chg_cap_cnt_write_index,sys_cap.val.chg_cap_cnt);
								chg_cap_cnt_write_index++;
								chg_cap_cnt_write_index &= 0x07;
								__enable_irq();
								sys_cap.val.bat_cycle_cnt++;
							srand(sys_cap.val.bat_cycle_cnt);
							sys_cap.val.full_cap = DESIGN_CAP - (sys_cap.val.bat_cycle_cnt<<1) - (rand()%3);//191225 chh ��ȥ�����
						}
				}
		}
		else
		{
				deta_cap_cnt = sys_cap.val.re_cap_rate;
		}
		if(bat_cycle_tmp != sys_cap.val.bat_cycle_cnt)
		{
				__disable_irq();
				// +w AP_Flash_Write(sys_cap.val.bat_cycle_cnt,FLASH_CYCLES_ADDR);
				//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_CYCLES_ADDR, (uint8_t *)&sys_cap.val.bat_cycle_cnt, 4);	
				flash_save_data(VER_FLASH_ADDR+FLASH_CYCLES_ADDR,bat_cycle_cnt_write_index,sys_cap.val.bat_cycle_cnt);
				bat_cycle_cnt_write_index++;
				bat_cycle_cnt_write_index &= 0x07;
				__enable_irq();
				bat_cycle_tmp = sys_cap.val.bat_cycle_cnt;

		}		
		//ÿ5%����һ��������EEPROM
		if((save_cap_tmp >sys_cap.val.re_cap_rate)&&(save_cap_tmp - sys_cap.val.re_cap_rate >5))
		{
				__disable_irq();
				//AP_Flash_Write(sys_cap.val.re_cap_rate,FLASH_SOC_ADDR);
				//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_SOC_ADDR, (uint8_t *)&sys_cap.val.re_cap_rate, 4);	
				flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
				re_cap_rate_write_index++;
				re_cap_rate_write_index &= 0x07;
				__enable_irq();
				save_cap_tmp = sys_cap.val.re_cap_rate;
			
		}
		if((sys_cap.val.re_cap_rate >save_cap_tmp)&&(sys_cap.val.re_cap_rate - save_cap_tmp >5))
		{
				__disable_irq();
				//AP_Flash_Write(sys_cap.val.re_cap_rate,FLASH_SOC_ADDR);
				//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_SOC_ADDR, (uint8_t *)&sys_cap.val.re_cap_rate, 4);
				flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
				re_cap_rate_write_index++;
				re_cap_rate_write_index &= 0x07;
				__enable_irq();
				save_cap_tmp = sys_cap.val.re_cap_rate;
		}
		
}

void Power_On_SOC(void)
{
		//��ȡд��EEPROM�����soc
		//......
		save_cap_tmp = sys_cap.val.re_cap_rate;//ˢ�±�����Ҫ��SOC��ǰֵ
}





