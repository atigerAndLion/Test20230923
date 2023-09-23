

/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __SOFT_SOC_H
#define __SOFT_SOC_H

#include "main.h"
#define CAP_CNT_VAL 		    14400//17460//1.2//14550准确   //14400   //1mah积分常数
#define CUR_DSG_050mA   		50			// mA
#define CUR_CHG_500mA   		62			// mA

typedef union
{
	struct{
		long	cap_cnt;
				
		u16  	 full_cap;
		u16    bat_cap;//剩余容量
	  u16    bat_cycle_cnt;
		u16    ocv_print;       //ocv打印6.11AID
		u32  	 re_cap_rate;//SOC
		u8  	 re_cap_rate_volt;
		u32		 chg_cap_cnt;
		u8		 fcc_index;
		u8     soh;
		u16    low_temp_bat_cap;
		u16    low_temp_re_cap_rate;
		u16 	 bat_cap_fcc;
		u16 	 re_cap_rate_fcc;
		//u32    fcc_tmp_print;  		//fcc_tmp打印
	}val;
}SystemCap;

extern volatile SystemCap  sys_cap;
extern uint32_t sleep_cnt;

extern uint8_t re_cap_rate_write_index ;
extern uint8_t chg_cap_cnt_write_index ;
extern uint8_t bat_cycle_cnt_write_index ;
extern u8 power_first_flag;
extern uint16_t update_delay;

/*
typedef union        //林增辉AID,用于验证是否满电容量更新
{
	struct{
		long	cap_cnt_1;
				
		u16  	 fcc_power_off;     
		u16    re_cap_more;
	  u16    no_work;
		u16  	 soc_fcc_un;
		u8  	 cap_less;
		
		u8		 chg_cap_cnt;
		u8		 fcc_index;
		u8     soh;
		u16    low_temp_bat_cap;
		u16    low_temp_re_cap_rate;
		u16 	 bat_cap_fcc;
		u16 	 re_cap_rate_fcc;
		
	}sta;
}FccSave;

extern volatile FccSave  fcc_save;      //林增辉AID
*/

void Soc_AddSub(void);
void NormalCapacityProc(void);
u8 VbatToSoc(u16 bat_val);
void Cap_Update_Check(void);
void FullCap_Update(void);
void BatCycleProc(void);
void Power_On_SOC(void);

#endif /* __SPI_FLASH_AUTO_H -------------------------------------------------------------------------------*/
