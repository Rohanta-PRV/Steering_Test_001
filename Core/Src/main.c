/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @Author			: Rohanta Shaw
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
//#include "ShiftingLED.c"
//#include "can.h"

//#include "Nextion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAX_RPM 9000
//#define LEDS 15

// Pin definition for the Nextion (NXT) display
#define NXT_TX_PIN GPIO_PIN_9
#define NXT_TX_PORT GPIOA
#define NXT_RX_PIN GPIO_PIN_10
#define NXT_RX_PORT GPIOA

// pin definition for the first register
/*
#define GPIO_PIN_6 GPIO_PIN_6 //DS1
#define GPIOB GPIOB
#define GPIO_PIN_7 GPIO_PIN_7 //STCP1
#define GPIOB GPIOB
#define GPIO_PIN_8 GPIO_PIN_8 //SHCP1
#define GPIOB GPIOB
#define GPIO_PIN_9 GPIO_PIN_9 //MR1
#define GPIOB GPIOB*/

// pin definition for the second register
/*
#define GPIO_PIN_15 GPIO_PIN_15 //DS2
#define GPIOA GPIOA
#define GPIO_PIN_3 GPIO_PIN_3 //STCP2
#define GPIOB GPIOB
#define GPIO_PIN_4 GPIO_PIN_4 //SHCP2
#define GPIOB GPIOB
#define GPIO_PIN_5 GPIO_PIN_5 //MR2
#define GPIOB GPIOB*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char* deviationCheckFail = "DEV FAIL\r\n";
char* STM_ded = "STM ded!\r\n";
float value1, value2;
char gear;

uint8_t currentVal; uint8_t currentVal1;
uint8_t canData[8];

uint8_t *a_value = (uint8_t *)&canData[0]; //GEAR
uint8_t *b_value = (uint8_t *)&canData[1];
uint8_t *c_value = (uint8_t *)&canData[2];
uint8_t *d_value = (uint8_t *)&canData[3];
uint8_t *e_value = (uint8_t *)&canData[4];
uint8_t *f_value = (uint8_t *)&canData[5];
uint8_t *g_value = (uint8_t *)&canData[6];
uint8_t *h_value = (uint8_t *)&canData[7];

volatile uint16_t *x_value = (volatile uint16_t *)&canData[0];
volatile uint16_t *y_value = (volatile uint16_t *)&canData[2];
volatile uint16_t *z_value = (volatile uint16_t *)&canData[3];
volatile uint16_t *w_value = (volatile uint16_t *)&canData[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void NXT_SendNum(char *obj, int32_t num);
void NXT_SendFloat(char *obj, float num, int dp);
void NXT_SendPB(char *obj, uint16_t num);
void NXT_SendTXT(char *obj, char val[]);
//void HC595write();
//void HC595write1();

// function definition for LED Driving
//void updateLEDs(uint16_t val);
//void shiftOP(uint16_t data, GPIO_TypeDef *ds_port, uint16_t ds_pin, GPIO_TypeDef *stcp_port, uint16_t stcp_pin, GPIO_TypeDef *shcp_port, uint16_t schp_pin, GPIO_TypeDef *mr_port, uint16_t mr_pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeader;

uint8_t cmd_end[3] = {0xFF, 0xFF, 0xFF};
char msg[50];

//	Predefined functions for sending different data to display
void NXT_SendNum(char *obj, int32_t num){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Enable onboard led to signify starting of transmission
	uint8_t *buffer = malloc(30 * sizeof(char));
	int len = sprintf((char *) buffer, "%s.val=%ld", obj, num); //change to %ld
	HAL_UART_Transmit(&huart1, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart1, cmd_end, 3, 100);
	free(buffer);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendFloat(char *obj, float num, int dp){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	int32_t number = num * (pow(10,dp));
	uint8_t *buffer = malloc(30 * sizeof(char));
	int len = sprintf((char *)buffer, "%s.vvs1=%d", obj, dp);
	HAL_UART_Transmit(&huart1, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart1, cmd_end, 3, 100);

	len = sprintf((char *)buffer, "%s.val=%ld", obj, number);
	HAL_UART_Transmit(&huart1, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart1, cmd_end, 3, 100);
	free(buffer);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendPB(char *obj, uint16_t num){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	uint8_t *buffer = malloc(30 * sizeof(char));

	int len = sprintf((char *)buffer, "%s=%u", obj, num); //%u is unsigned integer
	HAL_UART_Transmit(&huart1, buffer, len, 1000); // hal_uart1, uint8_t data, uint16_t size, uint32_t timeout
	HAL_UART_Transmit(&huart1, cmd_end, 3, 100);
	free(buffer);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void NXT_SendTXT(char *obj, char val[]){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	uint8_t *buffer = malloc(30 * sizeof(char));

	int len = sprintf((char *)buffer, "%s.txt=\"%s\"", obj, val);
	HAL_UART_Transmit(&huart1, buffer, len, 1000);
	HAL_UART_Transmit(&huart1, cmd_end, 3, 100);
	free(buffer);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

// LED DRIVING CODE 1
/*
void update_LEDs(uint16_t value){

	uint16_t led1 = 0, led2 = 0;
	uint8_t litLEDs = value * (LEDS / MAX_RPM);

	for(uint8_t i = 0 ; i < litLEDs ; i++){
		if(i < 8)
			led1 |= (1 << i);
		else
			led2 |= (1 << (i-8));
	}
	//sending data to first reg
	shiftOP(led1, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
	//sending data to second pin
	shiftOP(led2, GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_4, GPIOB, GPIO_PIN_5);

}

void shiftOP(uint16_t data, GPIO_TypeDef *ds_port, uint16_t ds_pin, GPIO_TypeDef *stcp_port, uint16_t stcp_pin, GPIO_TypeDef *shcp_port, uint16_t shcp_pin, GPIO_TypeDef *mr_port, uint16_t mr_pin){

	HAL_GPIO_WritePin(mr_port, mr_pin, GPIO_PIN_RESET); HAL_Delay(1);
	HAL_GPIO_WritePin(shcp_port, shcp_pin, GPIO_PIN_RESET);

	for(int8_t i = 7 ; i >= 0 ; i--){
		HAL_GPIO_WritePin(shcp_port, shcp_pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(shcp_port, ds_pin, (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		HAL_GPIO_WritePin(shcp_port, shcp_pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(stcp_port, stcp_pin, GPIO_PIN_SET);

}*/

//LED DRIVING CODE 2
void HC595write()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_Delay(1); // Ensure the register is cleared
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    for(int i=0; i<8; i++)
    {
    	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    	//HAL_Delay(900);
    	currentVal = currentVal >> 1;
    	if(currentVal & (1<<i))
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
        //HAL_Delay(900);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}
void HC595write1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_Delay(1); // Ensure the register is cleared
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

    for(int i=0; i<8; i++)
    {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    	HAL_Delay(100);
    	currentVal1 = currentVal1 >> 1;
        if(currentVal1 & (1<<i))
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
        HAL_Delay(100);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	NXT_SendTXT("rad_state", "EMPTY"); HAL_Delay(2000);
	while(! HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1));
	NXT_SendTXT("rad_state", "FILL"); HAL_Delay(2000);
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, canData) != HAL_OK)
    {
    	Error_Handler();
    }
    NXT_SendTXT("rad_state", "NO_ERH"); HAL_Delay(2000);
    if(RxHeader.ExtId == 0x18F00400){
    		// RPM (uint16_t)
    	value1 = (float)(*z_value)*0.125;
    	NXT_SendNum("rpm", (int32_t)value1);
    	// CALL THE UPDATE LED FUNCTION HEREAT
    	// sprintf(msg,"%0.2f,",value1);
    	// HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    }
    else if(RxHeader.ExtId == 0x18FEEE00){
    	// COOLANT
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		HAL_Delay(1000);
   		value1 = (float)(*c_value)*1;
   		NXT_SendFloat("oil_temp", value1, 2);
   		sprintf(msg,"oil_temp : %0.2f\n",value1);
   		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    	if(value1 > 80){
    		NXT_SendTXT("rad_state", "ON");
    		sprintf(msg,"rad_state : ON\n");
    		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    	}
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    	HAL_Delay(1000);
    }
    else if(RxHeader.ExtId == 0x18FEF717){
    	// BATTERY VOTLAGE
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    	  HAL_Delay(1000);
    	value1 = (float)(*e_value)*1;
    	NXT_SendFloat("bat_v", value1, 2);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    	HAL_Delay(1000);
    }

    else if(RxHeader.ExtId == 0x050){
    		gear = (char)(*a_value);
    		NXT_SendNum("gear", (int32_t)gear);
    		sprintf(msg,"Gear : %d \r\n",gear);
    		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg),1000);
    }
    else if(RxHeader.ExtId == 0x0CF00301){
    		// tps
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    	HAL_Delay(1000);
    	value1 = (float)(*c_value)*1;
    	NXT_SendNum("tpsbar", (int32_t)value1);
    	sprintf(msg,"TPS : %0.2f per\r\n",value1);
    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    	HAL_Delay(1000);
   	}

    else{
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    	sprintf(msg,"ID : %lu \r\n",RxHeader.ExtId);
   		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    }

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  NXT_SendTXT("rad_state", "");
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  currentVal = (0b01111111);
  HC595write();
  currentVal1 = (0b11111111);
  //HC595write1();

  // INITIALIZING THE VALUES FOR STARTUP
  NXT_SendTXT("t4", "BPS");
  NXT_SendTXT("t5", "TPS");

  NXT_SendNum("rpm", 0);
  NXT_SendNum("speed", 0);
  NXT_SendNum("map", 0);

  NXT_SendNum("bbar", 0);
  NXT_SendNum("tpsbar", 0);

  NXT_SendTXT("gear", "");
  NXT_SendTXT("rad_state", "OFF");
  NXT_SendFloat("afr", 1.00, 2);
  NXT_SendFloat("bat_v", 1.00, 2);
  NXT_SendFloat("oil_temp", 1.00, 2);
  // fill in with the other values. This will signify that the code has started.

  if(HAL_CAN_Start(&hcan) != HAL_OK)
  {
  	  NXT_SendTXT("gear", "!CAN");
  	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int b=0;
  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  NXT_SendNum("tpsbar", (b+10)%100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  HAL_Delay(200);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef can_filter_init;

  	can_filter_init.FilterActivation = CAN_FILTER_ENABLE;
  	can_filter_init.FilterBank = 10;  // Filter 0 (contains 2 32 bit Reg scans preferred ID)
  	can_filter_init.FilterFIFOAssignment = CAN_FILTER_FIFO1;  // FIFO1
  	can_filter_init.FilterIdHigh = 0x0000;
  	can_filter_init.FilterIdLow = 0x0000;
  	can_filter_init.FilterMaskIdHigh = 0x0000;
  	can_filter_init.FilterMaskIdLow = 0x0000;
  	can_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
  	can_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
  	can_filter_init.SlaveStartFilterBank = 0;

  	if(HAL_CAN_ConfigFilter(&hcan, &can_filter_init) != HAL_OK)
  	{
  		Error_Handler();
  	}
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
