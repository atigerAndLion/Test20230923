#include <atmel_start.h>
#include "main.h"
#include "../comm/i2c_sh367309.h"
#include "../comm/user_bms.h"
#include "../comm/CAN_RW_Manage.h"
#include "../comm/soft_soc.h"

#include "comm/i2c_cht8305.h"
#include "comm/i2c_sd2058.h"
#include "comm/W25Q32.h"
#include "comm/error_save.h"


volatile SystemFlags  sys_flags;
volatile SystemStates sys_states;
volatile SystemUarts  sys_uarts;
SystemDesign          sys_design;

SystemProtects  protects;
SystemProtects2 protects2;

const uint32_t Send_ID_Test      = 0x00000001;
uint8_t        Send_Data_Test[8] = { 0x00, 0x01, 0x02, 0x03 };

void cal_cadc_funtion(void);


uint32_t soc_count = 0;
uint32_t soc_count_start_flag = 0;

uint16_t sys_close_count = 0;

uint16_t noid_enter_sleep_count = 0;
uint16_t id_enter_sleep_count = 0;

char Jedec_buff[3];

// APP偏移需要
static volatile unsigned char flash_data[] __attribute__((section(".physicalsection")))={"const data"};

int main(void)
{
	uint16_t temp = flash_data[0] ;
	uint8_t temp_buf[1];
	uint16_t sleep_count = 0;
	uint8_t led_count = 0;
	uint32_t tmp32 = 0;

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	wdt_disable(&WDT_0);
	wdt_set_timeout_period(&WDT_0, 1000, 2048); //1K 频率 2S超时

	hri_wdt_write_EWCTRL_EWOFFSET_bf((&(&WDT_0)->dev)->hw,WDT_EWCTRL_EWOFFSET_CYC1024_Val);
	hri_wdt_set_INTEN_EW_bit((&(&WDT_0)->dev)->hw);
	NVIC_DisableIRQ(WDT_IRQn);
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);
	wdt_enable(&WDT_0);
	wdt_feed(&WDT_0);
	
	gpio_set_pin_level(EN_SHIP,true);
	
	
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,0,0x12345678);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,1,0x23456781);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,2,0x34567812);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,3,0x45678123);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,4,0x56781234);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,5,0x67812345);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,6,0x78123456);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,7,0x81234567);
	//flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,8,0x87654321);
	
	//flash_read_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,&write_index,&read_data);

	flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_BOOTSTATUS_ADDR, temp_buf, 1);
	if (temp_buf[0] != JUMP_APP_SUCC)
	{
		temp_buf[0] = JUMP_APP_SUCC;
		flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_BOOTSTATUS_ADDR, temp_buf, 1);	
	}
	
	flash_read_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,&re_cap_rate_write_index,&tmp32);
	if (tmp32 <= 100)
	{
		sys_cap.val.re_cap_rate = tmp32;
	}
	else
	{
		sys_cap.val.re_cap_rate = 0xff;
	}

	
	flash_read_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,&chg_cap_cnt_write_index,&sys_cap.val.chg_cap_cnt);
	if (sys_cap.val.chg_cap_cnt == 0xffffffff)
	{
		sys_cap.val.chg_cap_cnt = 0;
	}
	flash_read_data(VER_FLASH_ADDR+FLASH_CYCLES_ADDR,&bat_cycle_cnt_write_index,&sys_cap.val.bat_cycle_cnt);	
	if (sys_cap.val.bat_cycle_cnt == 0xffffffff)
	{
		sys_cap.val.bat_cycle_cnt = 0;
	}
	
	flash_read_data(VER_FLASH_ADDR+FLASH_ERROR_ADDR,&Flash_error_write_index,&Flash_error);	
	if (Flash_error == 0xffffffff)
	{
		Flash_error = 0;
		Flash_error_tmp = 0;
	}
	else
	{
		Flash_error_tmp = Flash_error;
		if((Flash_error&0x01) == 0x01)
		{
			protects.protect.ntc_open_flag =1;
		}
		if((Flash_error&0x02) == 0x02)
		{
			protects.protect.cell_dif_volt_flag =1;
		}
		if((Flash_error&0x04) == 0x04)
		{
			protects.protect.cell_low_2v_flag =1;
		}
		if((Flash_error&0x08) == 0x08)
		{
			protects.protect.dch_mos_err_flag =1;
		}
		if((Flash_error&0x10) == 0x10)
		{
			protects.protect.chg_mos_err_flag =1;
		}
		if((Flash_error&0x20) == 0x20)
		{
			protects.protect.ntc_short_flag =1;
		}		
		
	}
	
	
	sys_design.val.soft_version = SOFT_VER;
	sys_design.val.hard_version = HARD_VER;
	sys_design.val.boot_version = BOOT_VER;
	sys_design.val.design_year  = 20;
	sys_design.val.design_month = 6;
	sys_design.val.design_year = 1;
	
	sys_cap.val.full_cap = DESIGN_CAP;
	
	//hri_supc_write_VREF_SEL_bf(SUPC,SUPC_VREF_SEL_2V048_Val);
	//hri_supc_clear_BODVDD_ENABLE_bit(SUPC);
	//hri_supc_set_BODVDD_RUNSTDBY_bit(SUPC);
	//hri_supc_write_BODVDD_LEVEL_bf(SUPC,8);
	//hri_supc_set_BODVDD_ENABLE_bit(SUPC);

	//CTL_ON();//锁死充放电管,7309上电无异常默认打开输出
	//delay_ms(300);//SH367309上电延迟300ms
	CP_EN_ON();	
	// AFE初始化	
	if(InitAFE())
	{
		//printf("InitAFE initialization succeeded..\r\n");
	}
	else
	{
		//printf("InitAFE initialization failed..\r\n");
	}
	
	if (sys_flags.flag.sys_id_connect_flag == 1)
	{
		// 直接输出 不需要初始化
	}
	else
	{
		//上次状态不是输出状态   进入ROM对比
		while((UpdataAfeConfig()))
		{
			wdt_feed(&WDT_0);
		}
	}
	//
	//if(  (UpdataAfeConfig()) )//读取ROM信息,确认是否更新ROM
	//{
		////printf("ROM initialization succeeded..\r\n");		
	//}
	//else
	//{
		////printf("ROM initialization failed..\r\n");		
	//}
	//初始CAN发送、接收回调与滤波器.
	CTL_ON();
	VCC_EN_ON();
	STB_OFF();
	
	wdt_feed(&WDT_0);
	delay_ms(50);
	CAN_RW_Init();
	//CAN 发送测试
	//CAN_Send(Send_ID_Test, CAN_FMT_STDID, Send_Data_Test, 8);
	
	//ext_irq_register(RTC_INT, RTC_Alarm_Funtion);
	ext_irq_register(INT_DET, Det_Alarm_Funtion);
	ext_irq_register(INT_WAKE, Can_Wake_Funtion);

	CHT8305_Init();
	CHT8305_GetData();
	wdt_feed(&WDT_0);
	SD2058_Time_Init();
	SD2058_Rtc_Read_Time(&sd2058_val);
	wdt_feed(&WDT_0);
	
	//test
	//cal_cadc_funtion();
	SPI_Init();
	SPI_Jedec_ID_Read(Jedec_buff,Jedec_buff+1,Jedec_buff+2);	
	wdt_feed(&WDT_0);
	//SPI_FLASH_Get_writeindex();
	//SPI_FLASH_Get_readindex();
	SPI_FLASH_Get_Index();
	wdt_feed(&WDT_0);
	/* Replace with your application code */
	while (1) {
		//gpio_toggle_pin_level(FUSE);
		wdt_feed(&WDT_0);
		led_count++;
		
		if (led_count == 25)
		{
			gpio_set_pin_level(FUSE,true);
			
		}
		else if (led_count > 25)
		{
			gpio_set_pin_level(FUSE,false);
			led_count = 0;
		}
		//gpio_set_pin_level(FUSE,false);
		//gpio_set_pin_level(FUSE,true);

		RT_ON_ON();
		
		SD2058_Rtc_Read_Time(&sd2058_val);
		CHT8305_GetData();
		PMOS_EN_ON();
		AFE_Update_Function();//更新AFE数据至RAM	
		ADC_Function();//ADC采集
		State_Change_Function();//状态位切换
		
		MOS_Control_Function();//MOSFET控制
		NormalCapacityProc();//容量更新
		SPI_FLASH_errorstate();//故障记录
		Sleep_Function();//休眠
		Comm_Proc(); //调整位置
		ShutDown_Function();//关机
		delay_ms(7); //控制在40ms 一个周期

		//delay_ms(19); //整个循环40mS
	}
}
void Sleep_Function()
{
	uint16_t tmp_16 = 0;
	static uint16_t lvp2_enter_shutdown_count = 0;
		
	if (protects.protect.dch_lvp2_flag == 1)
	{
		lvp2_enter_shutdown_count++;
		noid_enter_sleep_count = 0;
		id_enter_sleep_count = 0;
		sys_flags.flag.sleep_flag = 0;
		if (lvp2_enter_shutdown_count > 3000) //120s 进入关机
		{
			sys_flags.flag.sys_close_flag = 1;
			sys_close_count = GC_DELAY_30S;
		}
		
	}
	else
	{
		lvp2_enter_shutdown_count = 0;
	}
	
	
	if (sys_flags.flag.sys_id_connect_flag == 0 && sys_flags.flag.sleep_flag == 0 )
	{
		noid_enter_sleep_count++;
		if (noid_enter_sleep_count > 750)//暂定10S  40ms * 750  = 30s
		{
			sys_flags.flag.sleep_flag = 1;
			noid_enter_sleep_count = 0;
		}
	}
	else
	{
		noid_enter_sleep_count = 0;
	}
	
	if (sys_flags.flag.sys_id_connect_flag == 1 && sys_flags.flag.sleep_flag == 0)
	{
		if (sys_flags.flag.open_dch_30s_end_flag == 1 && sys_flags.flag.sys_comm_open_dch_flag ==0 && sys_flags.flag.adapter_connect_flag == 0) //预放电结束，非充非放
		{
			id_enter_sleep_count++;
			if (id_enter_sleep_count > 750)//暂定10S  40ms * 750  = 30s
			{
				sys_flags.flag.sleep_flag = 1;
				id_enter_sleep_count = 0;
			}
		}
		else if (sys_flags.flag.open_dch_30s_end_flag == 1 && sys_flags.flag.sys_comm_open_dch_flag ==1) //放电阶段
		{
			if (protects.protect.dch_lvp_flag == 1 || protects2.protect.soc_lp_flag == 1)
			{
				id_enter_sleep_count++;
				if (id_enter_sleep_count > 750)//暂定10S  40ms * 750  = 30s
				{
					sys_flags.flag.sleep_flag = 1;
					id_enter_sleep_count = 0;
				}
			}
			else
			{
				id_enter_sleep_count = 0;
			}
		}
		else
		{
			id_enter_sleep_count = 0;
		}
	}
	else
	{
		id_enter_sleep_count = 0;
	}
	
	
	//在老化模式 不进入休眠 清除计数
	flash_read(&FLASH_0, VER_FLASH_ADDR+FLASH_DCH_NO_COM_ADDR,(uint8_t *)&tmp_16, 2);
	if (tmp_16 == 0x0000)
	{
		noid_enter_sleep_count = 0;
		id_enter_sleep_count = 0;
		sys_flags.flag.sleep_flag = 0;
	}
	
	
	if (sys_flags.flag.sleep_flag == 1)
	{
		/* System entry into Power-down Mode */
		can_async_disable(&CAN_0);
		delay_ms(5);
		STB_ON();
		delay_ms(5);
		VCC_EN_OFF(); // 5V没开的情况下 38uA
		RT_ON_OFF();
		
		CP_EN_OFF();
		gpio_set_pin_level(FUSE,false);

		_ext_irq_enable(INT_WAKE, true);
		
		gpio_set_pin_direction(CS, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(CS, GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(CS,false);
		gpio_set_pin_direction(MISO, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(MISO, GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(MISO,false);
		gpio_set_pin_direction(SCK, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(SCK, GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(SCK,false);
		gpio_set_pin_direction(MOSI, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(MOSI, GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_level(MOSI,false);
sleep_table:	
		
		sleep(4);
		//休眠起来
		//hri_wdt_clear_interrupt_EW_bit(&WDT_0);
		sleep_cnt++;
		wdt_feed(&WDT_0);
		if (sleep_cnt % 10 == 0)
		{
			UpdateStates();
			if (AFE.BSTATUS1.Bit.UV == 1)
			{
				if (protects.protect.dch_lvp_flag == 1 || protects2.protect.soc_lp_flag == 1)
				{
					//触发一级了  再次触发二级
					//sys_flags.flag.sys_close_flag = 1;
					//sys_close_count = GC_DELAY_30S;
					sys_flags.flag.sleep_flag = 0;
				}
				else 
				{
					//没有触发一级 就直接触发二级
					sys_flags.flag.sleep_flag = 0;
				}
			}
			
			if (AFE.BSTATUS3.Bit.CHGING == 1 || AFE.BSTATUS3.Bit.DSGING == 1)
			{
				sys_flags.flag.sleep_flag = 0;
			}
			
			if (sleep_cnt == 1800)
			{
				sys_flags.flag.sleep_flag = 0;
				update_delay = 2400;
			}
			
		}
		
		if (sys_flags.flag.sleep_flag == 1)
		{
			goto sleep_table;
		}
		CP_EN_ON();
		VCC_EN_ON();
		delay_ms(100);
		STB_OFF();
		delay_ms(5);
		SPI_0_init();
		SPI_Init();
		CAN_RW_Init();
		_ext_irq_enable(INT_WAKE, false);
	}
}


void ShutDown_Function()
{
	
	if (sys_flags.flag.sys_close_flag == 1)
	{
		sys_close_count++;
		if (sys_close_count >= GC_DELAY_30S)
		{
			flash_save_data(VER_FLASH_ADDR+FLASH_CHGCAP_ADDR,chg_cap_cnt_write_index,sys_cap.val.chg_cap_cnt);
			chg_cap_cnt_write_index++;
			chg_cap_cnt_write_index &= 0x07;
			
			flash_save_data(VER_FLASH_ADDR+FLASH_SOC_ADDR,re_cap_rate_write_index,sys_cap.val.re_cap_rate);
			re_cap_rate_write_index++;
			re_cap_rate_write_index &= 0x07;
			
			VCC_EN_OFF();
			delay_ms(1000);
			gpio_set_pin_level(JUMP_DET,true);
		}
	}
	else
	{
		sys_close_count = 0;
	}
}

void RTC_Alarm_Funtion(void)
{
	;
}

void Det_Alarm_Funtion(void)
{
	sys_flags.flag.sleep_flag = 0;
}

void Can_Wake_Funtion(void)
{
	sys_flags.flag.sleep_flag = 0;
}

void HardFault_Handler(void)
{
	
}

void WDT_Handler(void)
{
	hri_wdt_clear_INTFLAG_EW_bit((&(&WDT_0)->dev)->hw);
}

/*
void cal_cadc_funtion(void)
{
	//校准测试
	//开充放电
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
	count = 10;
	while ( count != 0)
	{
		AFE_Update_Function();
		wdt_feed(&WDT_0);
		if (AFE.BFLAG2.Bit.CADC_FLG == 1)
		{
			AFE.BFLAG2.Bit.CADC_FLG = 0;
			cadc_temp[count] =(uint16_t)AFE.Cadc;
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
	//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_1A_ADC_ADDR,(uint8_t *)&AFE.Cadc, 2);
	
	cadc_sum = 0;
	cadc_min = 0xffff;
	cadc_max = 0;
	//关充放电
	Dsg_Control(CLOSE_MOS);
	Chg_Control(CLOSE_MOS);
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
			cadc_temp[count+16] = (uint16_t)AFE.Cadc;
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
	//flash_write(&FLASH_0, VER_FLASH_ADDR+FLASH_0A_ADC_ADDR,(uint8_t *)&AFE.Cadc, 2);
	sys_flags.flag.current_calibration = 1 ;
}
*/