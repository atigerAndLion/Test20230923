/*
 * main.h
 *
 * Created: 2020/5/15 14:41:48
 *  Author: chenjiawei
 */ 

#ifndef __MAIN_H
#define	__MAIN_H 
#include <atmel_start.h>
#include "mydef.h"
#include "driver_init.h"
/****************************************************************************
 define macro
*****************************************************************************/
#define UART_PRINTF

extern uint32_t soc_count ;
extern uint32_t soc_count_start_flag ;

extern uint16_t sys_close_count ;

#define VER_FLASH_ADDR 				0x3c000
#define EEPROM_LENGTH				21

#define FLASH_BOOTSTATUS_ADDR       4





#define FLASH_CUR_CAL_ADDR			0x100
#define FLASH_0A_ADC_ADDR			FLASH_CUR_CAL_ADDR
#define FLASH_1A_ADC_ADDR			FLASH_CUR_CAL_ADDR+2
#define FLASH_DCH_NO_COM_ADDR		0x200
#define FLASH_NO_SOCLP_ADDR			FLASH_DCH_NO_COM_ADDR+4
#define FLASH_SN_ADDR				0x300
#define FLASH_SOC_ADDR				0x400 
#define FLASH_CHGCAP_ADDR			0x600
#define FLASH_CYCLES_ADDR			0x800
#define FLASH_ERROR_ADDR            0xa00

#define COVER_APP 0x00
#define JUMP_APP  0x01
#define JUMP_APP_FAIL  0x02
#define JUMP_APP_SUCC  0x03
#define BOOT_RECOVER   0x04


#define FLASH_HARDVER_ADDR			3
#define FLASH_BOOTVER_ADDR			4

#define FLASH_RESERVE_ADDR			6
#define FLASH_TIME_ADDR				7
#define FLASH_EMPTY_ADDR			10
#define FLASH_BMSID_ADDR			11
#define FLASH_CAP_ADDR				14
#define FLASH_ERR_ADDR				19
#define FLASH_SOFTVER_ADDR			20


#define SOFT_VER       106
#define HARD_VER       102
#define BOOT_VER       100

#define OPEN_MOS    1
#define CLOSE_MOS   0

#define VPRO_EN_ON()            {gpio_set_pin_level(VPRO_EN,true);}
#define VPRO_EN_OFF()           {gpio_set_pin_level(VPRO_EN,false);}

#define CPEN_ON()				{gpio_set_pin_level(CP_EN,true);}
#define CPEN_OFF()				{gpio_set_pin_level(CP_EN,false);}

#define VCC_EN_ON()				{gpio_set_pin_level(VCC_EN,true);}
#define VCC_EN_OFF()			{gpio_set_pin_level(VCC_EN,false);}
	
#define STB_ON()				{gpio_set_pin_level(STB,true);}
#define STB_OFF()				{gpio_set_pin_level(STB,false);}

#define CTL_ON()				{gpio_set_pin_level(CTL,true);}
#define CTL_OFF()				{gpio_set_pin_level(CTL,false);}
	
#define RT_ON_ON()				{gpio_set_pin_level(RT_ON,true);}
#define RT_ON_OFF()				{gpio_set_pin_level(RT_ON,false);}
	
#define CP_EN_ON()				{gpio_set_pin_level(CP_EN,true);}
#define CP_EN_OFF()				{gpio_set_pin_level(CP_EN,false);}
	
#define PMOS_EN_ON()			{gpio_set_pin_level(PMOS_EN,true);}
#define PMOS_EN_OFF()			{gpio_set_pin_level(PMOS_EN,false);}
	

#define ADC_NTC1_CH      	2	
#define ADC_NTC2_CH      	3	

#define ADC_VPACK_CH      5



//休眠宏定义
#define SLEEP_NO_DCH_SysCtrl2		0x00		//深度休眠关闭CC_EN
#define SLEEP_BQ_SysCtrl2_HIGH4		0x4			//CC_EN
#define NORMAL_BQ_SysCtrl2_HIGH4	0x4	

#define SLEEP_BQ_SYSCTRL1			0x18		//开启ADC :电压，TS， TEMP_SEL：内部			
#define SLEEP_BQ_SYSCTRL2			0x43		//CC_ONESHOT,DELAY_DIS
#define NORMAL_BQ_SYSCTRL1			0x18		//ADC EN ，TEMP_SEL：外部
#define NORMAL_BQ_SYSCTRL2			0x40		//CC_EN,DELAY_DIS

#define WAKE_MODE_RTC				0x00		//RTC唤醒默认
#define WAKE_MODE_ID				0x01		//ID唤醒
#define WAKE_MODE_RS485				0x02		//通讯唤醒
#define WAKE_MODE_ADAPTER			0x03		//适配器唤醒
#define WAKE_MODE_BQ_STAT			0x04		//BQ唤醒，故障，CC_READY
#define WAKE_MODE_KEY				0x05		//按键唤醒
#define WAKE_MODE_TIMING			0x06		//休眠一定时长后唤醒		




typedef union
{
    struct
    {
        unsigned chg_en_flag								 : 1;
        unsigned dch_en_flag                         		 : 1;
        unsigned sys_sh367309_comm_err_flag   		         : 1;
        unsigned sys_id_error_flag                     		 : 1;
        unsigned chg_state_flag                     	     : 1;
        unsigned dch_state_flag                				 : 1;
        unsigned open_dch_30s_end_flag                       : 1;
        unsigned sys_comm_open_dch_flag    			   	   	 : 1;
        
        
        unsigned adapter_connect_flag	 	       			 : 1;
        unsigned com_current_cal							 : 1;
        unsigned current_calibration						 : 1;
        unsigned sys_recovery_flag                           : 1;
        unsigned sys_id_connect_flag                         : 1;
        unsigned sys_close_flag                              : 1;
        unsigned sleep_flag                                  : 1;
        unsigned read_spi_flash_flag                         : 1;
    }flag;
    uint16_t VAL;
}SystemFlags;   

typedef union
{
    struct
    {
        unsigned soc_fcc_4_flag      						 : 1;
        unsigned key_up_flag                         		 : 1;
        unsigned sys_led1_up_flag   		                 : 1;
        unsigned sys_chg_ch_flag                     		 : 1;
        unsigned sys_en_dch_flag                     		 : 1;
        unsigned sys_reset_flag                				 : 1;
        unsigned sys_adapter_ov_flag                         : 1;
        unsigned sys_err_display_flag    			  	 	 : 1;
        
        
        unsigned eerpom_changetowrite_flag	 	       	     : 1;
        unsigned fcc_power_on_flag							 : 1;
        unsigned soc_fcc_save_flag					         : 1;
        unsigned soc_fcc_1_flag                		         : 1;
        unsigned soc_fcc_2_flag                              : 1;
        unsigned soc_fcc_3_flag                              : 1;
        unsigned update_goto_reset_flag                      : 1;
        unsigned sys_close                                   : 1;
    }state;
    uint16_t VAL;
}SystemStates; 


typedef union
{
    struct
    {
        unsigned uart_receive_flag							 : 1;
        unsigned dc5v_ocp_flag                               : 1;
        unsigned uart_decode_flag		                     : 1;
        unsigned uart_flash_earse_flag                       : 1;
        unsigned cmd_reset_flag               		         : 1;
        unsigned update_begin_flag                      	 : 1;
        unsigned uart_sleep_flag  			  	 			 : 1;
        unsigned uart_update_ok_flag	 	       			 : 1;

        unsigned re_cap_update_flag							 : 1;
        unsigned cap_update_add_flag					   	 : 1;
        unsigned cap_update_sub_flag                         : 1;
        unsigned cc_250ms_flag                               : 1;
        unsigned blance_state_flag                           : 1;
        unsigned uart_en_tx2_flag                            : 1;
        unsigned uart_rev_timeout_flag                       : 1;
        unsigned goto_sleep_flag                             : 1;
    }flag;
    uint16_t VAL;
}SystemUarts;  

typedef union
{
    struct
    {
        unsigned chg_ovp_flag								 : 1;
        unsigned forbidden_output                            : 1;
        unsigned dch_lvp2_flag		                         : 1;
        unsigned dch_lvp_flag                       		 : 1;
        unsigned chg_ocp_flag               		         : 1;
        unsigned dch_ocp_flag                      	         : 1;
        unsigned dch_ocp2_flag  			  	 			 : 1;
        unsigned dch_scd_flag    	 	       			     : 1;

        unsigned chg_mos_err_flag							 : 1;
        unsigned dch_mos_err_flag							 : 1;
        unsigned ntc_open_flag                           	 : 1;
        unsigned ntc_short_flag                              : 1;
        unsigned chg_full_flag                               : 1;
        unsigned dch_empty_flag                              : 1;
        unsigned cell_low_2v_flag	                         : 1;
        unsigned cell_dif_volt_flag                          : 1;
    }protect;
    uint16_t VAL;
}SystemProtects;

typedef union
{
    struct
    {
        unsigned chg_one_ovw_flag							 : 1;
        unsigned dsg_sed_uvw_flag                            : 1;
        unsigned dsg_otw_flag		                         : 1;
        unsigned dsg_ltw_flag                       		 : 1;
        unsigned chg_otw_flag               		         : 1;
        unsigned chg_ltw_flag                      	         : 1;
        unsigned chg_ocw_flag  			  	 				 : 1;
        unsigned n1	 	       			                     : 1;

        unsigned n2							  	             : 1;
        unsigned n3								             : 1;
        unsigned n4                           		         : 1;
        unsigned n5                                          : 1;
        unsigned n6                                          : 1;
        unsigned n7                                          : 1;
        unsigned n8                                          : 1;
        unsigned n9                                          : 1;
    }waring;
    uint16_t VAL;
}SystemWaring;
typedef union
{
    struct
    {
        unsigned temp_diff_flag					             : 1;
        unsigned mosfet_temp_err_flag                        : 1;
        unsigned chg_ntc_err_flag		                     : 1;
        unsigned dch_ntc_err_flag                            : 1;
        unsigned chg_otp_flag               		         : 1;
        unsigned chg_ltp_flag                      	         : 1;
        unsigned dch_otp_flag  			  	 				 : 1;
        unsigned dch_ltp_flag	 	       			         : 1;

        unsigned chg_otp2_flag							  	 : 1;
        unsigned chg_ltp2_flag								 : 1;
        unsigned dch_otp2_flag                           	 : 1;
        unsigned dch_ltp2_flag                               : 1;
        unsigned soc_lp_flag			                     : 1;
        unsigned cell_dif_volt_dsg_flag                      : 1;
        unsigned cell_dif_volt_chgwaring_flag                : 1;
        unsigned cell_dif_volt_dsgwaring_flag                : 1;
    }protect;
    uint16_t VAL;
}SystemProtects2; 


typedef union
{
	struct{
				
		uint16_t  	soft_version;
		uint16_t   hard_version;
		uint32_t   boot_version;
		uint8_t    design_year;
		uint8_t    design_month;
		uint8_t    design_day;
	}val;
}SystemDesign;

#define  DESIGN_CAP   (u32)23000


/****************************************************************************
 Variable declaration
*****************************************************************************/

extern volatile SystemFlags sys_flags;
extern volatile SystemStates sys_states;
extern volatile SystemUarts  sys_uarts;
extern SystemDesign  sys_design;

extern SystemProtects  protects;
extern SystemProtects2  protects2;

extern uint16_t noid_enter_sleep_count ;
extern uint16_t id_enter_sleep_count ;


/****************************************************************************
 Prototype declaration
*****************************************************************************/

void Power_On(void);
void Sleep_Function(void);	 
void ShutDown_Function(void);
void RTC_Alarm_Funtion(void);
void Det_Alarm_Funtion(void);
void Can_Wake_Funtion(void);


#endif	/* MAIN */

