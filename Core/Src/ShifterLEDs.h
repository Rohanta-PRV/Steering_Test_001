#ifndef ShifterLEDs_H
#define ShifterLEDs_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void HC95write();
void HC95write1();

// other LED Driving functions declarations
// void updateLEDs(uint16_t val);
// void shiftOP(uint16_t data, GPIO_TypeDef *ds_port, uint16_t ds_pin, GPIO_TypeDef *stcp_port, uint16_t stcp_pin, GPIO_TypeDef *shcp_port, uint16_t schp_pin, GPIO_TypeDef *mr_port, uint16_t mr_pin);

#endif
