/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMC21 has 9 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8

#define INT_WAKE GPIO(GPIO_PORTA, 0)
#define INT_ALARM GPIO(GPIO_PORTA, 1)
#define AD_WT2 GPIO(GPIO_PORTA, 2)
#define AD_BAT GPIO(GPIO_PORTA, 3)
#define AD_RT4 GPIO(GPIO_PORTA, 4)
#define AD_RT5 GPIO(GPIO_PORTA, 5)
#define CTL GPIO(GPIO_PORTA, 6)
#define STB GPIO(GPIO_PORTA, 7)
#define SDA GPIO(GPIO_PORTA, 8)
#define SCL GPIO(GPIO_PORTA, 9)
#define PMOS_EN GPIO(GPIO_PORTA, 10)
#define AD_PP GPIO(GPIO_PORTA, 11)
#define RH_SDA GPIO(GPIO_PORTA, 12)
#define RH_SCL GPIO(GPIO_PORTA, 13)
#define CS GPIO(GPIO_PORTA, 16)
#define SCK GPIO(GPIO_PORTA, 17)
#define MISO GPIO(GPIO_PORTA, 18)
#define MOSI GPIO(GPIO_PORTA, 19)
#define RTC_SDA GPIO(GPIO_PORTA, 20)
#define RTC_SCL GPIO(GPIO_PORTA, 21)
#define RTC_INT GPIO(GPIO_PORTA, 22)
#define RT_ON GPIO(GPIO_PORTA, 23)
#define CAN_TX GPIO(GPIO_PORTA, 24)
#define CAN_RX GPIO(GPIO_PORTA, 25)
#define RH_ALM GPIO(GPIO_PORTA, 27)
#define JUMP_DET GPIO(GPIO_PORTA, 28)
#define AD_5V GPIO(GPIO_PORTB, 2)
#define AD_WT1 GPIO(GPIO_PORTB, 3)
#define FUSE GPIO(GPIO_PORTB, 8)
#define EN_SHIP GPIO(GPIO_PORTB, 9)
#define VCC_EN GPIO(GPIO_PORTB, 10)
#define CP_EN GPIO(GPIO_PORTB, 11)
#define VPRO_EN GPIO(GPIO_PORTB, 22)
#define INT_DET GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
