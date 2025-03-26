

#include "hardware.h"

void MY_LED_POWER(GPIO_PinState Status){
	HAL_GPIO_WritePin(LED_PORT, LED_Pin, Status);
}
void MY_V_EXC(){
	HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_4);
}


void Gain_Voltage(enum Gain GainVoltage){
	switch (GainVoltage) {
  case S1:
		HAL_GPIO_WritePin(SelVoltEn_PORT, SelVoltEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelVoltA0_PORT, SelVoltA0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SelVoltA1_PORT, SelVoltA1_Pin, GPIO_PIN_RESET);
	
    break;
  case S2:
		
    HAL_GPIO_WritePin(SelVoltEn_PORT, SelVoltEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelVoltA0_PORT, SelVoltA0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SelVoltA1_PORT, SelVoltA1_Pin, GPIO_PIN_RESET);
    break;
	case S3:
    HAL_GPIO_WritePin(SelVoltEn_PORT, SelVoltEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelVoltA0_PORT, SelVoltA0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SelVoltA1_PORT, SelVoltA1_Pin, GPIO_PIN_SET);
    break;
	case S4:
    HAL_GPIO_WritePin(SelVoltEn_PORT, SelVoltEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelVoltA0_PORT, SelVoltA0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SelVoltA1_PORT, SelVoltA1_Pin, GPIO_PIN_SET);
    break;
   
}
}

void Gain_Current(enum Gain GainCurrent){
	switch (GainCurrent) {
  case S1:
		HAL_GPIO_WritePin(SelCurrentEn_PORT, SelCurrentEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelCurrentA0_PORT, SelCurrentA0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SelCurrentA1_PORT, SelCurrentA1_Pin, GPIO_PIN_RESET);
	
    break;
  case S2:
		
    HAL_GPIO_WritePin(SelVoltEn_PORT, SelCurrentEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelCurrentA0_PORT, SelCurrentA0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SelCurrentA1_PORT, SelCurrentA1_Pin, GPIO_PIN_RESET);
    break;
	case S3:
    HAL_GPIO_WritePin(SelCurrentEn_PORT, SelCurrentEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelCurrentA0_PORT, SelCurrentA0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SelCurrentA1_PORT, SelCurrentA1_Pin, GPIO_PIN_SET);
    break;
	case S4:
    HAL_GPIO_WritePin(SelCurrentEn_PORT, SelCurrentEn_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SelCurrentA0_PORT, SelCurrentA0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SelCurrentA1_PORT, SelCurrentA1_Pin, GPIO_PIN_SET);
    break;
   
}
}