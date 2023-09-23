/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

#include <hpl_adc_base.h>
#include <hpl_adc_base.h>

struct spi_m_sync_descriptor SPI_0;
struct can_async_descriptor  CAN_0;

struct adc_sync_descriptor ADC_0;

struct adc_sync_descriptor ADC_1;

struct flash_descriptor FLASH_0;

struct wdt_descriptor WDT_0;

void ADC_0_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_WT2, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_WT2, PINMUX_PA02B_ADC0_AIN0);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_BAT, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_BAT, PINMUX_PA03B_ADC0_AIN1);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_RT4, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_RT4, PINMUX_PA04B_ADC0_AIN4);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_RT5, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_RT5, PINMUX_PA05B_ADC0_AIN5);

	gpio_set_pin_level(PMOS_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PMOS_EN, GPIO_DIRECTION_OUT);


	gpio_set_pin_function(PMOS_EN, GPIO_PIN_FUNCTION_OFF);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_PP, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_PP, PINMUX_PA11B_ADC0_AIN11);
}

void ADC_0_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC0, _adc_get_adc_sync());
}

void ADC_1_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_5V, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_5V, PINMUX_PB02B_ADC1_AIN2);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AD_WT1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AD_WT1, PINMUX_PB03B_ADC1_AIN3);
}

void ADC_1_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_ADC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, CONF_GCLK_ADC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void ADC_1_init(void)
{
	ADC_1_CLOCK_init();
	ADC_1_PORT_init();
	adc_sync_init(&ADC_1, ADC1, _adc_get_adc_sync());
}

void EXTERNAL_IRQ_0_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, EIC_GCLK_ID, CONF_GCLK_EIC_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_EIC_bit(MCLK);
	

	// Set pin direction to input
	gpio_set_pin_direction(INT_WAKE, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(INT_WAKE,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(INT_WAKE, PINMUX_PA00A_EIC_EXTINT0);

	// Set pin direction to input
	gpio_set_pin_direction(INT_ALARM, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(INT_ALARM,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(INT_ALARM, PINMUX_PA01A_EIC_EXTINT1);

	// Set pin direction to input
	gpio_set_pin_direction(RTC_INT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(RTC_INT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(RTC_INT, PINMUX_PA22A_EIC_EXTINT6);

	// Set pin direction to input
	gpio_set_pin_direction(INT_DET, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(INT_DET,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(INT_DET, PINMUX_PB23A_EIC_EXTINT7);

	// Set pin direction to input  //¸ÄÎªÊä³ö
	//gpio_set_pin_direction(JUMP_DET, GPIO_DIRECTION_IN);
//
	//gpio_set_pin_pull_mode(JUMP_DET,
	                       //// <y> Pull configuration
	                       //// <id> pad_pull_config
	                       //// <GPIO_PULL_OFF"> Off
	                       //// <GPIO_PULL_UP"> Pull-up
	                       //// <GPIO_PULL_DOWN"> Pull-down
	                       //GPIO_PULL_OFF);
//
	//gpio_set_pin_function(JUMP_DET, PINMUX_PA28A_EIC_EXTINT8);
	
	gpio_set_pin_level(JUMP_DET,
	// <y> Initial level
	// <id> pad_initial_level
	// <false"> Low
	// <true"> High
	false);

	// Set pin direction to output
	gpio_set_pin_direction(JUMP_DET, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(JUMP_DET, GPIO_PIN_FUNCTION_OFF);

	// Set pin direction to input
	gpio_set_pin_direction(RH_ALM, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(RH_ALM,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(RH_ALM, PINMUX_PA27A_EIC_EXTINT15);

	ext_irq_init();

}

void FLASH_0_CLOCK_init(void)
{

	hri_mclk_set_AHBMASK_NVMCTRL_bit(MCLK);
}

void FLASH_0_init(void)
{
	FLASH_0_CLOCK_init();
	flash_init(&FLASH_0, NVMCTRL);
}

void SPI_0_PORT_init(void)
{

	gpio_set_pin_level(SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SCK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SCK, PINMUX_PA17C_SERCOM1_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(MISO, PINMUX_PA18C_SERCOM1_PAD2);

	gpio_set_pin_level(MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOSI, PINMUX_PA19C_SERCOM1_PAD3);
}

void SPI_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM1);
	SPI_0_PORT_init();
}

void WDT_0_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_WDT_bit(MCLK);
}

void WDT_0_init(void)
{
	WDT_0_CLOCK_init();
	wdt_init(&WDT_0, WDT);
}

void CAN_0_PORT_init(void)
{

	gpio_set_pin_function(CAN_RX, PINMUX_PA25G_CAN0_RX);

	gpio_set_pin_function(CAN_TX, PINMUX_PA24G_CAN0_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_0_init(void)
{
	hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_0, CAN0);
	CAN_0_PORT_init();
}

void system_init(void)
{
	init_mcu();
	

	// GPIO on PA06

	gpio_set_pin_level(CTL,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CTL, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CTL, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA07

	gpio_set_pin_level(STB,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(STB, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(STB, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA08

	gpio_set_pin_level(SDA,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(SDA, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SDA, GPIO_PIN_FUNCTION_OFF);
	
	// GPIO on PA09

	gpio_set_pin_level(SCL,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(SCL, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SCL, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA12

	gpio_set_pin_level(RH_SDA,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RH_SDA, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RH_SDA, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA13

	gpio_set_pin_level(RH_SCL,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RH_SCL, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RH_SCL, GPIO_PIN_FUNCTION_OFF);

	
	// GPIO on PA16

	gpio_set_pin_level(CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CS, GPIO_PIN_FUNCTION_OFF);
		

	// GPIO on PA20

	gpio_set_pin_level(RTC_SDA,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RTC_SDA, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RTC_SDA, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	gpio_set_pin_level(RTC_SCL,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RTC_SCL, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RTC_SCL, GPIO_PIN_FUNCTION_OFF);
	


	// GPIO on PA23

	gpio_set_pin_level(RT_ON,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(RT_ON, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RT_ON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	gpio_set_pin_level(FUSE,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(FUSE, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(FUSE, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09


	gpio_set_pin_level(EN_SHIP,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(EN_SHIP, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(EN_SHIP, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10

	gpio_set_pin_level(VCC_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(VCC_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(VCC_EN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11

	gpio_set_pin_level(CP_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CP_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CP_EN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	gpio_set_pin_level(VPRO_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(VPRO_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(VPRO_EN, GPIO_PIN_FUNCTION_OFF);
	

	
	ADC_0_init();

	ADC_1_init();

	EXTERNAL_IRQ_0_init();

	FLASH_0_init();

	SPI_0_init();

	WDT_0_init();
	CAN_0_init();
}
