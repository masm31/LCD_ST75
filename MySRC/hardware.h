/*
this file config IO and other pripheral for test Coductivity board
Shiraz Janaury 2025
*/


#include "main.h"
#include "stm32f4xx_hal.h"
#ifndef __STM32F4xx_HAL_GPIO_H
#define __STM32F4xx_HAL_GPIO_H
#endif




#define LED_PORT GPIOB
#define LED_Pin GPIO_PIN_13

#define SelVoltA0_PORT GPIOC
#define SelVoltA0_Pin GPIO_PIN_3
#define SelVoltA1_PORT GPIOC
#define SelVoltA1_Pin GPIO_PIN_2
#define SelVoltEn_PORT GPIOC
#define SelVoltEn_Pin GPIO_PIN_1

#define SelCurrentA0_PORT GPIOB
#define SelCurrentA0_Pin GPIO_PIN_1
#define SelCurrentA1_PORT GPIOB
#define SelCurrentA1_Pin GPIO_PIN_0
#define SelCurrentEn_PORT GPIOC
#define SelCurrentEn_Pin GPIO_PIN_4


#define Voltage_EXC PA_6



enum Gain {
  S1=0,
  S2=1,
  S3=2,
	S4=3
};


void MY_LED_POWER(GPIO_PinState);
void MY_V_EXC();

void Gain_Voltage(enum Gain GainVoltage);
void Gain_Current(enum Gain GainCurrent);