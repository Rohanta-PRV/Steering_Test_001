#include "ShifterLEDs.h"

#include "main.h"
#include "stm32f1xx_hal.h"

//uint8_t currentVal; uint8_t currentVal1;

void HC95write(){
	// ...
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
}

void HC95write1(){
	// ...
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	HAL_Delay(750);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
}
