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
#include <stdbool.h>

//#include "can.h"
//#include "Nextion.h"

// SELF DEFINED Header files
#include "ShifterLEDs.h"
#include "NXT_Functions.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAX_RPM 9000
//#define LEDS 15

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
HAL_StatusTypeDef stat;
UART_HandleTypeDef huart1;				// Declared in the "NXT_Functions.c" src file

/* USER CODE BEGIN PV */
char* STM_ded = "STM ded!\r\n";
float value1, value2;
char gear, lol, unit;

int8_t units_val;
uint8_t canData[8];

uint16_t a_val = 0, b_val = 0, c_val = 0, d_val = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void Errrwhatthesigma(char *errMsg, int f);
void ErrLedBlink(int f);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

char* HAL_StatusToString(HAL_StatusTypeDef status) {
    switch(status) {
        case HAL_OK:      return "OK";					// HAL_OK
        case HAL_ERROR:   return "ERROR";				// HAL_ERROR
        case HAL_BUSY:    return "BUSY";				// HAL_BUSY
        case HAL_TIMEOUT: return "TIMEOUT";				// HAL_TIMEOUT
        default:          return "IDK";					// UNKNOWN STATUS
    }
}

void Errrwhatthesigma(char errMsg[], int f){				// main user defined Error / ALERT handling function

	if(f==3){												// LOW PRIORITY
		NXT_SendCmd("errMsg", "bco", 65535, "", false);		// changes the colour of the gear txt box to white.
		NXT_SendCmd("errMsg", "pco", 0, "", false);			// changes the colour of the text inside the "Gear" txt box to black
		NXT_SendTXT("errMsg", errMsg);
		ErrLedBlink(f);
	}
	else if(f==2){											// MEDIUM PRIORITY
		NXT_SendCmd("errMsg", "bco", 65505, "", false);		// changes the colour of the gear txt box to yellow.
		NXT_SendCmd("errMsg", "pco", 0, "", false);			// changes the colour of the text inside the "Gear" txt box to black
		NXT_SendTXT("errMsg", errMsg);
		ErrLedBlink(f);
	}
	else if(f==1){											// HIGH PRIORITY
		NXT_SendCmd("errMsg", "bco", 51200, "", false);		// changes the colour of the gear txt box to red.
		NXT_SendCmd("errMsg", "pco", 65535, "", false);		// changes the colour of the text inside the "Gear" txt box to white
		NXT_SendTXT("errMsg", errMsg);
		ErrLedBlink(f);
	}
	else{
		NXT_SendTXT("rad_state", errMsg);
		ErrLedBlink(f);
	}
}

void ErrLedBlink(int f){
	int x = 0;
	while((x++) < 20){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
		/*
		if(f == 1)				// HIGH PRIORITY / SYSTEM CRITICAL
			HAL_Delay(200);
		else if(f == 2)			// MEDIUM PRIORITY
			HAL_Delay(750);

		else if(f == 3)			// LOW PRIORITY
			HAL_Delay(1250);
		else
			HAL_Delay(3000);
		*/
		(f==1) ? HAL_Delay(200) : (f==2) ? HAL_Delay(750) : (f==3) ? HAL_Delay(1250) : HAL_Delay(2000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		(f==1) ? HAL_Delay(200) : (f==2) ? HAL_Delay(750) : (f==3) ? HAL_Delay(1250) : HAL_Delay(2000);
	}
}

void deelay(uint8_t x){
	while(x-- == 0){
		for(uint8_t i = 0 ; i < 10000 ; i++);
	}
}

//	Predefined functions for sending different data to display

//LED DRIVING CODE GOES HERE

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// could keep in infinite loop and keep checking if it getting filled, and after some time (1 min), if it doesn't, send to error handling.
	uint32_t FifoFill = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);
	while(! FifoFill){
		Errrwhatthesigma("!FIFO_FILL", 3);
		Error_Handler();
	}
	NXT_SendNum("map", FifoFill);					// Shares the fill level of FIFO1 to see how many messages are filling up the queue.

	NXT_SendTXT("rad_state", "FILL"); HAL_Delay(2000);

	stat = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, canData);
    if (stat != HAL_OK){							// Tells us if the CAN bus is ready, listening or not.
    	Errrwhatthesigma(HAL_StatusToString(stat), 3);
    	Error_Handler();
    }
    NXT_SendTXT("rad_state", "RxMsgRcvd");
    //NXT_SendNum("map", 2);						// will continue to show number of meesages in fifo until the code if figured out.
    HAL_Delay(2000);

    if(RxHeader.ExtId == 0x18F00400){							// ID: 0CFFF048
    	//value1 = (float)(*z_value)*0.125;
    	a_val = (canData[1] << 8) | canData[0];					// RPM (uint16_t)
    	NXT_SendNum("rpm", (int32_t)a_val);
    	    // 2. Next 2 bytes as signed 16-bit integer
    	b_val = (int16_t)((canData[3] << 8) | canData[2]);		// TPS (int16_t)
    	// if (b_val > 32767) b_val -= 65536;

    	NXT_SendNum("tpsbar", (int32_t)(b_val)); // can just output canData[2] as high byte (canData[3]) will always be 00
/*
    	    // 3. Next 2 bytes as signed 16-bit integer
    	c_val = (int16_t)((canData[4] << 8) | canData[5]);		// FUEL OPEN TIME (int16_t)
    	if (c_val > 32767) c_val -= 65536;
    	NXT_SendNum("rpm", (int32_t)a_val);
    	    // 4. Last 2 bytes as signed 16-bit integer
    	d_val = (int16_t)((canData[6] << 8) | canData[7]);		// INJECTOR ANGLE (int16_t)
    	if(d_val > 32767) d_val -= 65536;
    	*/
    	// CALL THE UPDATE LED FUNCTION HEREAT

    }

    else if(RxHeader.ExtId == 0x18FEEE00){						// ID: 0CFFF548

    	units_val = (int8_t)(canData[7]);						// TEMPERATURE TYPE / UNIT (0 / 1)
    	if(units_val == 01) unit = 'C';
    	else unit = 'F';

   		//value1 = (float)(*c_value)*1;
    	a_val = (int32_t)(canData[1] << 8) | canData[0];		// BATTERY VOLTAGE (uint)
    	NXT_SendFloat("bat_v", (float)(a_val*0.01), 2);

    	/* 	// can remove comments once we have acquire a sensor for the same
    	b_val = (int16_t)((canData[3] << 8) | canData[2]);		// AIR TEMP (signed int)
    	if (b_val > 32767) b_val -= 65536;
    	NXT_SendNum("tpsbar", (int32_t)b_val);*/

    	c_val = (int16_t)((canData[5] << 8) | canData[4]);		// COOLANT TEMP (signed int)
    	if (c_val > 32767) c_val -= 65536;
   		NXT_SendFloat("oil_temp", (float)(c_val*0.01), 2);

    	if(c_val > 8000){
    		NXT_SendCmd("rad_state", "bco", 65505, "", false); 		// Turns the radiator state box to yellow to signify change
    		NXT_SendTXT("rad_state", "ON");
    		deelay(2);
    	}




    }
    /*
    else if(RxHeader.ExtId == 0x18FEF717){						// ID: ???
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
    else if(RxHeader.ExtId == 0x0CF00301){						// ID: ???
    		// tps
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    	HAL_Delay(1000);
    	value1 = (float)(*c_value)*1;
    	NXT_SendNum("tpsbar", (int32_t)value1);
    	sprintf(msg,"TPS : %0.2f per\r\n",value1);
    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
    	HAL_Delay(1000);
   	}*/

    else{
    	NXT_SendNum("map", RxHeader.ExtId);
    	//sprintf(msg,"ID : %lu \r\n",RxHeader.ExtId);
   		//HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
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
  NXT_SendTXT("rad_state", ""); HAL_Delay(3000);
  stat = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  if(stat != HAL_OK){
	  Errrwhatthesigma(HAL_StatusToString(stat), 2);
	  Error_Handler();
  }

    HC95write();
    HC95write1();
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
  stat = HAL_CAN_Start(&hcan);
  if(stat != HAL_OK){
  	  Errrwhatthesigma(HAL_StatusToString(stat), 2);
  	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //int b=0;
  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //NXT_SendNum("tpsbar", (b+10)%100);
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
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  		Errrwhatthesigma(HAL_StatusToString(HAL_CAN_ConfigFilter(&hcan, &can_filter_init)), 2);
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
