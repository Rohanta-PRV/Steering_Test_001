#include "NXT_Functions.h"

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"

// Pin definition for the Nextion (NXT) display
#define NXT_TX_PIN GPIO_PIN_9
#define NXT_TX_PORT GPIOA
#define NXT_RX_PIN GPIO_PIN_10
#define NXT_RX_PORT GPIOA

UART_HandleTypeDef huart;					// huart1

uint8_t cmd_end[3] = {0xFF, 0xFF, 0xFF};	// Needed at the end of every instruction
char msg[50]; 								// Array for Sending/Transmitting messages

void NXT_SendNum(char *obj, int32_t num){

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Enable onboard led to signify starting of transmission
	uint8_t *buffer = malloc(30 * sizeof(char));
	int len = sprintf((char *) buffer, "%s.val=%ld", obj, num); //change to %ld
	HAL_UART_Transmit(&huart, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart, cmd_end, 3, 100);
	free(buffer);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendFloat(char *obj, float num, int dp){

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	int32_t number = num * (pow(10,dp));
	uint8_t *buffer = malloc(30 * sizeof(char));
	int len = sprintf((char *)buffer, "%s.vvs1=%d", obj, dp);
	HAL_UART_Transmit(&huart, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart, cmd_end, 3, 100);

	len = sprintf((char *)buffer, "%s.val=%ld", obj, number);
	HAL_UART_Transmit(&huart, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart, cmd_end, 3, 100);
	free(buffer);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendPB(char *obj, uint16_t num){

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	uint8_t *buffer = malloc(30 * sizeof(char));

	int len = sprintf((char *)buffer, "%s=%u", obj, num); //%u is unsigned integer
	HAL_UART_Transmit(&huart, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart, cmd_end, 3, 100);
	free(buffer);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendTXT(char *obj, char val[]){

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	uint8_t *buffer = malloc(30 * sizeof(char));

	int len = sprintf((char *)buffer, "%s.txt=\"%s\"", obj, val);
	HAL_UART_Transmit(&huart, buffer, len, 1000);
	HAL_UART_Transmit(&huart, cmd_end, 3, 100);
	free(buffer);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendCmd(char *obj, char *param, uint16_t num, char *st, bool isStr){

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	uint8_t *buffer = malloc(30 * sizeof(char));

	int len = (isStr) ? sprintf((char *)buffer, "%s.%s=%d", obj, param, num) : sprintf((char *)buffer, "%s.%s=\"%s\"", obj, param, st);
	HAL_UART_Transmit(&huart, buffer, len, 1000);
	HAL_UART_Transmit(&huart, cmd_end, 3, 100);
	free(buffer);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}
