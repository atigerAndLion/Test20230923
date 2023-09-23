/*
 * user_bms.h
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 
 #ifndef __USER_BMS_H
#define __USER_BMS_H
#include "main.h"

#define VCELL_Dsg_DEF_ERR     500
#define VCELL_Chg_DEF_ERR     200 //191105 
#define VCELL_Dsg_DEF_WAR     400
#define VCELL_Chg_DEF_WAR     150 //191105

//V = NTC10K+10K  3.3V   AD = (RT/RT+10000)*3300*4096/3300
//VTSX = AD*382 μV
//RTS = 10000*VTSX/(3.3-VTSX)
//VTSX = 3.3RTS/(10000+RTS)
//AD = 3.3RTS/(10000+RTS)/382
#define NTC_CHG_OTP				550//3.54K
#define NTC_CHG_OTP_CLEAR		500//4.15K
#define NTC_DCH_OTP 			600				//3K
#define NTC_DCH_OTP_CLEAR		550				//3.54k

#define NTC_CHG_LTP				00			//27.8K
#define NTC_CHG_LTP_CLEAR		50			//22.3K
#define NTC_DCH_LTP				-200			//72K	
#define NTC_DCH_LTP_CLEAR		-150			//58K

#define NTC_DSG_REV             100  

#define NTC_SHORT				1240
#define NTC_OPEN				-390
#define NTC_DIFF                100
#define NTC_DIFF_CLEAR          50  
#define NTC_MOSFET_ERR          1000
#define NTC_MOSFET_CLEAR        800

#define NTC_ERR_SHORT							8200
#define NTC_ERR_OPEN							660			//569K

#define VBAT_FULL					      	55200			//31v
#define VBAT_FULL_CLEAR      				53600   		//33V

// AD = V*3/53*4096/3300 =
#define VPACK_OVP								2978//42V        // 3191   // 45V
#define VPACK_CONNECT						55000 // 40.05V   //2837   // 40V
#define VPACK_NCONT							50000   // 39V


//放电为负 充电为正

#define ICUR_DCH_STATE     	 		-300
#define ICUR_DCH_OCP     	 		-30000//-30000	 
#define	ICUR_DCH_OCP2				-40000
#define ICUR_DCH_OCP_30S     	 	2962	  

#define ICUR_CHG_STATE      	 	200	  
#define ICUR_CHG_OCP      	 		15000	  

#define ICUR_DCH_MOS_ERR            -300
#define ICUR_CHG_MOS_ERR            300

#define VCELL_ERR								1500
#define VCELL_DEF								700

#define VCELL_OVP								3700
#define VCELL_OVP_CLEAR							3450

#define VCELL_UVP								2200
#define VCELL_UVP_CLEAR							2800

#define VBAT_UVP								39000 //34V
#define VBAT_UVP_CLEAR							43200 //33V

#define SOC_LP          4
#define SOC_LP_CLEAR    9

#define GC_DELAY_02S						5
#define GC_DELAY_05S						12
#define GC_DELAY_1S							25
#define GC_DELAY_2S							50
#define GC_DELAY_4S							100
#define GC_DELAY_5S							125
#define GC_DELAY_6S							150
#define GC_DELAY_8S							200
#define GC_DELAY_10S						250
#define GC_DELAY_20S						500
#define GC_DELAY_30S						750


#define AD_WT2_ADC0_CHANNEL    0
#define AD_BAT_ADC0_CHANNEL    1
#define AD_RT4_ADC0_CHANNEL    4
#define AD_RT5_ADC0_CHANNEL    5
#define AD_PP_ADC0_CHANNEL     11
#define AD_5V_ADC1_CHANNEL     2
#define AD_WT1_ADC1_CHANNEL    3

#define MCU_TEMP_DEGREE_RANGE 	166LL
#define MCU_TEMP_OFFSET_VAL 	400LL

#define DET_READ()        gpio_get_pin_level(INT_DET)

extern uint16_t stick_30s_cnt;
extern uint32_t spi_read_count;

extern u8 bat_uvp_cnt ;
extern u8 soc_lp_cnt ;

void State_Change_Function(void);
void Voltage_Protect_Function(void);
void Current_Protect_Function(void);
void Temperature_Protect_Function(void);
void MOS_Control_Function(void);
void Adapter_Function(void);
void AFE_Update_Function(void);
void ADC_Function(void);
void Protect_Retry_Function(void);
void Balance_Function(void);
void Current_Cal_Function(void);

uint16_t Find_MCU_Temp(uint16_t temp_val);
void flash_save_data(uint32_t address,uint8_t index,uint32_t data);
void flash_read_data(uint32_t address,uint8_t * index,uint32_t * data);

void Can_send_spi_flash(void);
void Can_Check_Init(void);

#endif
