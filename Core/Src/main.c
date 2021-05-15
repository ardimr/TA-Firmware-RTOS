/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mpu6050.h"
#include "MFRC522.h"
#include "integer_type.h"
#include <math.h>
#include "RCFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_RESOLUTION 4096
#define VOLTAGE_REFERENCE 3.3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 250 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 250 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RFIDTask */
osThreadId_t RFIDTaskHandle;
const osThreadAttr_t RFIDTask_attributes = {
  .name = "RFIDTask",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for SDCardTask */
osThreadId_t SDCardTaskHandle;
const osThreadAttr_t SDCardTask_attributes = {
  .name = "SDCardTask",
  .stack_size = 250 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADCProcessingTa */
osThreadId_t ADCProcessingTaHandle;
const osThreadAttr_t ADCProcessingTa_attributes = {
  .name = "ADCProcessingTa",
  .stack_size = 250 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//task Handle

//IMU Variable
MPU6050_t MPU6050;

//RFID Variable
uint8_t UID[4]={};

//GPS Variable
float latitude = 0;
float longitude = 0;
char strUTC[8] = {}; // UTC time in the readable hh:mm:ss format
uint8_t flag = 0;

//ADC Variables
float value[3];
uint32_t buffer[3];
RCFilter rcFiltFuel, rcFiltAccu, rcFiltBatt;
MovAvgFilter MAFiltFuel;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void Display(void *argument);
void IMU(void *argument);
void GPS(void *argument);
void RFID(void *argument);
void SDCard(void *argument);
void ADCProcesing(void *argument);

/* USER CODE BEGIN PFP */
// function to calculate checksum of the NMEA sentence
// -4, but not -3 because the NMEA sentences are delimited with \r\n, and there also is the invisible \r in the end
int nmea0183_checksum(char *msg) {
	int checksum = 0;
	int j = 0;
	// the first $ sign and the last two bytes of original CRC + the * sign
	for (j = 1; j < strlen(msg) - 4; j++) {
		checksum = checksum ^ (unsigned) msg[j];
	}
	return checksum;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  for(int i = 0; i<3; i++){
	  value[i] = buffer[i];
  }
  //Filtering Analog reading
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  char txBuffer [100] = {};
  sprintf(txBuffer, "Bismillah..\n");
  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(Display, NULL, &DisplayTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(IMU, NULL, &IMUTask_attributes);

  /* creation of GPSTask */
//  GPSTaskHandle = osThreadNew(GPS, NULL, &GPSTask_attributes);

  /* creation of RFIDTask */
  RFIDTaskHandle = osThreadNew(RFID, NULL, &RFIDTask_attributes);

  /* creation of SDCardTask */
//  SDCardTaskHandle = osThreadNew(SDCard, NULL, &SDCardTask_attributes);

  /* creation of ADCProcessingTa */
  ADCProcessingTaHandle = osThreadNew(ADCProcesing, NULL, &ADCProcessingTa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* Infinite loop */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RFID_RST_Pin|RFID_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SDCARD_CS_Pin|POWER_SEL_Pin|IGNITION_LOGIC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RFID_RST_Pin RFID_CS_Pin */
  GPIO_InitStruct.Pin = RFID_RST_Pin|RFID_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCARD_CS_Pin POWER_SEL_Pin IGNITION_LOGIC_Pin */
  GPIO_InitStruct.Pin = SDCARD_CS_Pin|POWER_SEL_Pin|IGNITION_LOGIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGING_SIGNAL_Pin */
  GPIO_InitStruct.Pin = CHARGING_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGING_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IGNITION_SIGNAL_Pin */
  GPIO_InitStruct.Pin = IGNITION_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IGNITION_SIGNAL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Display */
/**
  * @brief  Function implementing the DisplayTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Display */
void Display(void *argument)
{
  /* USER CODE BEGIN 5 */
	char txBuffer [200] = {};
	sprintf(txBuffer, "Running Display Task..\n");
  /* Infinite loop */
	uint8_t identification = 0;
  for(;;)
  {
	  //Identification Check
	  if (identification == 1){
		  xTaskNotifyGive(IMUTaskHandle);
	  }

	  if(UID[0]== 0x29){ // Need to add driver database
		  identification = 1;
	  } else {
		  identification = 0;
	  }
	  //End of Identification Check
	  sprintf(txBuffer,"ID : %x-%x-%x-%x\tAx = %.2f Ay = %.2f Az = %.2f Fuel : %.2f\n",
			  UID[0],UID[1],UID[2],UID[3], MPU6050.Ax, MPU6050.Ay,MPU6050.Az,MAFiltFuel.out);
	  HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 500);
//	HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 500);
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_IMU */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU */
void IMU(void *argument)
{
  /* USER CODE BEGIN IMU */
	char txBuffer[100]= {};
	sprintf(txBuffer, "Running IMU Task..\n");

	uint8_t ID = MPU6050_Init(&hi2c1);
	sprintf(txBuffer,"Id : %d Initialization Success .. \n", ID);
	HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, sizeof(txBuffer), 100);
	//Clearing Buffer
	memset(txBuffer,0,sizeof(txBuffer));
	osDelay(200);
  /* Infinite loop */
  for(;;)
  {
	//Blocking Until Notified
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	MPU6050_Read_Accel(&hi2c1, &MPU6050);
//	sprintf(txBuffer,"Ax = %.2f Ay = %.2f Az = %.2f\n", MPU6050.Ax, MPU6050.Ay,MPU6050.Az );
//	HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 500);
    osDelay(50);
  }
  /* USER CODE END IMU */
}

/* USER CODE BEGIN Header_GPS */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS */
void GPS(void *argument)
{
  /* USER CODE BEGIN GPS */
	uint8_t buff[255];
	  char buffStr[255];
	  char nmeaSnt[80];

	  char *rawSum;
	  char smNmbr[3];

	  // The Equator has a latitude of 0°,
	  //the North Pole has a latitude of 90° North (written 90° N or +90°),
	  //and the South Pole has a latitude of 90° South (written 90° S or −90°)
	  char *latRaw;
	  char latDg[10];
	  char latMS[20];
	  char *hemNS;

	  // longitude in degrees (0° at the Prime Meridian to +180° eastward and −180° westward)
	  // that is why 3
	  char *lonRaw;
	  char lonDg[10];
	  char lonMS[10];
	  char *hemEW;

	  char *utcRaw; // raw UTC time from the NMEA sentence in the hhmmss format


	  char hH[2]; // hours
	  char mM[2]; // minutes
	  char sS[2]; // seconds

	  uint8_t cnt = 0;

	  HAL_UART_Receive_DMA(&huart1, buff, 255);

	  HAL_UART_Transmit(&huart2, (unsigned char *) "Start\n", 6, 500);
  /* Infinite loop */
  for(;;)
  {
	  char txBuffer[200] = {};
	  sprintf(txBuffer,"\nFlag : %d", flag);
	  //	  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
	  if (flag) {
	  	memset(buffStr, 0, 255);
	  	sprintf(buffStr, "%s", buff);
	  //HAL_UART_Transmit(&huart2, (uint8_t *)buffStr, sizeof(buffStr), 70);
	  // splitting the buffStr by the "\n" delimiter with the strsep() C function
	  // see http://www.manpagez.com/man/3/strsep/
	  	char *token, *string;
	  	// actually splitting the string by "\n" delimiter
	  	string = strdup(buffStr);
	  	while ((token = strsep(&string, "\n")) != NULL) {
	  		memset(nmeaSnt, 0, 80);
	  		sprintf(nmeaSnt, "%s", token);

	  		memset(txBuffer,0,sizeof(txBuffer));
	  //HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 100);
	  // selecting only $GNGLL sentences, combined GPS and GLONAS
	  // on my GPS sensor this good NMEA sentence is always 50 characters
	  		if ((strstr(nmeaSnt, "$GPGGA") != 0) && (strlen(nmeaSnt) > 49) &&(strlen(nmeaSnt) <90) && strstr(nmeaSnt, "*") != 0) {
	  			rawSum = strstr(nmeaSnt, "*");
	  			memcpy(smNmbr, &rawSum[1], 2);
	  			smNmbr[2] = '\0';

	  			uint8_t intSum = nmea0183_checksum(nmeaSnt);
	  			char hex[2];
	  			// "%X" unsigned hexadecimal integer (capital letters)
	  			sprintf(hex, "%X", intSum);

	  			// checksum data verification, if OK, then we can really trust
	  			// the data in the the NMEA sentence
	  			if (strstr(smNmbr, hex) != NULL) {
	  				cnt = 0;
	  //			sprintf(txBuffer,"pV : %s\n", pV);
	  			// splitting the good NMEA sentence into the tokens by the comma delimiter
	  				for (char *pV = strtok(nmeaSnt, ","); pV != NULL; pV = strtok(NULL, ",")) {
	  					memset(txBuffer,0,sizeof(txBuffer));
	  					sprintf(txBuffer,"pV[%d] : %s\n",cnt, pV);
	  //				HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 100);
	  					switch (cnt) {
	  						case 1:
	  							  utcRaw = strdup(pV);
	  							  break;
	  						case 2:
	  							  latRaw = strdup(pV);
	  							  break;
	  						case 3:
	  							  hemNS = strdup(pV);
	  							  break;
	  						case 4:
	  							  lonRaw = strdup(pV);
	  							  break;
	  						case 5:
	  							  hemEW = strdup(pV);
	  							  break;
	  					}
	  					cnt++;
	  				} //end for

	  				//Converting Longitude and Latitude into Float
	  				latitude = atof(latRaw);
	  				longitude = atof(lonRaw);

	  				if (*hemNS == 'S') {
	  					latitude  *= -1.0;
	  				}
	  				if (*hemEW == 'W') {
	  					longitude *= -1.0;
	  				}
	  				char * token;
	  				//Get LatitudeDegree
	  				token = strtok(latRaw, ".");
	  				memset(latDg, 0, sizeof(latDg));
	  //			memcpy(latDg, token, strlen(token));
	  				sprintf(latDg, token);
	  				//Get Minutes
	  				token = strtok(NULL,".");
	  				memset(latMS, 0, sizeof(latMS));
	  //			memcpy(latMS, token, strlen(token));
	  				sprintf(latMS, token);
	  //			latMS[7] = '.';

	  				//Get longitude Degree
	  				float degrees = trunc(latitude / 100.0f);
	  				float minutes = latitude - (degrees * 100.0f);
	  				latitude = degrees + (minutes / 60.0f);

	  				degrees = trunc(longitude / 100.0f);
	  				minutes = longitude - (degrees * 100.0f);
	  				longitude = degrees + (minutes / 60.0f);


	  				token = strtok(lonRaw, ".");
	  				memset(lonDg, 0, sizeof(lonDg));
	  				memcpy(lonDg, token, strlen(token));

	  				token = strtok(NULL, ".");
	  				memset(lonMS, 0, sizeof(lonMS));
	  				memcpy(lonMS, token, strlen(token));

	  				memset(txBuffer,0,sizeof(txBuffer));
	  				sprintf(txBuffer, "latDg : %s latMs : %s lonDg : %s lonMs : %s\n",latDg,latMS,lonDg,lonMS );
	  //		  HAL_UART_Transmit(&huart2, (unsigned char *)txBuffer, sizeof(txBuffer), 100);

	  					  //converting the UTC time in the hh:mm:ss format
	  				memcpy(hH, &utcRaw[0], 2);
	  				hH[2] = '\0';

	  				memcpy(mM, &utcRaw[2], 2);
	  				mM[2] = '\0';

	  				memcpy(sS, &utcRaw[4], 2);
	  				sS[2] = '\0';

//	  				float latDg_f = atof(latDg)/100.0;
//	  				float latMS_f = atof(latMS)/60.0;
//	  				float lonDg_f = atof(lonDg)/100.0;
//	  				float lonMS_f = atof(lonMS)/60.0;

	  				strcpy(strUTC, hH);
	  				strcat(strUTC, ":");
	  				strcat(strUTC, mM);
	  				strcat(strUTC, ":");
	  				strcat(strUTC, sS);
	  				strUTC[8] = '\0';

	  				memset(txBuffer,0,sizeof(txBuffer));
	  				sprintf(txBuffer, "Latitude : %f Longitude : %f UTC : %s\n",latitude,longitude, strUTC);
	  				HAL_UART_Transmit(&huart2, (unsigned char *)txBuffer, sizeof(txBuffer), 100);

	  			} //end of the chekcsum data verification
	  		} //end of %GPPGA Sentences selection
	  	}// end of splotting the buffstr by the "\n" delimiter with strsep() c function
	  	flag = 0;
	  }
	  osDelay(500);
  }
  /* USER CODE END GPS */
}

/* USER CODE BEGIN Header_RFID */
/**
* @brief Function implementing the RFIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RFID */
void RFID(void *argument)
{
  /* USER CODE BEGIN RFID */
	char txBuffer [100] ={};
	u_char status, checksum1, cardstr[MAX_LEN];
	MFRC522_Init();
	status = 0;

	while (status == 0){
		status = Read_MFRC522(VersionReg);
		sprintf(txBuffer,"Running RC522 ver :%x\n", status);
		HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 5000);
		osDelay(100);
	}
	//Printing to PC
	memset(txBuffer,0,sizeof(txBuffer));
	status = 0;
	osDelay(200);
  /* Infinite loop */
  for(;;)
  {
	  status = MFRC522_Request(PICC_REQIDL, cardstr);
	  if(status == MI_OK){
		  sprintf(txBuffer,"Card detected ..\n");
		  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 500);
		  sprintf(txBuffer,"Card Type : %x %x %x\n", cardstr[0],cardstr[1],cardstr[2]);
//		  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 500);
		  memset(txBuffer,0,sizeof(txBuffer));

		  //Anti-collision, return card serial number == 4 bytes
		  status = MFRC522_Anticoll(cardstr);
		  if (status == MI_OK){
			  checksum1 = cardstr[0] ^ cardstr[1] ^ cardstr[2] ^ cardstr[3];
			  for(int i = 0; i <4 ;i++){
				  UID[i]=cardstr[i];
			  }
			  sprintf(txBuffer,"UID: %x %x %x %x\n\r",(u_char)cardstr[0], (u_char)cardstr[1],(u_char)cardstr[2],(u_char)cardstr[3]);
//			  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
		  }
	  }
	  else {
		  memset(txBuffer,0,sizeof(txBuffer));
		  sprintf(txBuffer,"Status :%x\n", status);
//		  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 5000);
		  //		  sprintf(txBuffer,"Finding ..\n");
//		  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
	  }
    osDelay(100);
  }
  /* USER CODE END RFID */
}

/* USER CODE BEGIN Header_SDCard */
/**
* @brief Function implementing the SDCardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SDCard */
void SDCard(void *argument)
{
  /* USER CODE BEGIN SDCard */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SDCard */
}

/* USER CODE BEGIN Header_ADCProcesing */
/**
* @brief Function implementing the ADCProcessingTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADCProcesing */
void ADCProcesing(void *argument)
{
  /* USER CODE BEGIN ADCProcesing */
	char txBuffer[100] = {};
	HAL_ADC_Start_DMA(&hadc1, buffer, 3);
	sprintf(txBuffer,"ADC Intialization..\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);
	/* Initialize RC Filter */
	RCFilter_Init(&rcFiltFuel, 5.0f, 100.0f);

	/*Initialize Moving Average Filter*/
	MovAvgFilter_init(&MAFiltFuel);
	/* Start ADC */
	HAL_ADC_Start_DMA(&hadc1, buffer, 3);
	sprintf(txBuffer,"ADC Intialization Success..\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);
  /* Infinite loop */
  for(;;)
  {
	  float input = (value[1]/ADC_RESOLUTION) * VOLTAGE_REFERENCE;
	  RCFilter_Update(&rcFiltFuel, input);
	  MovAvgFilter_Update(&MAFiltFuel, input);
//	  sprintf(txBuffer,"Raw : %.3f Filtered : %.3f\n", input, rcFiltFuel.out[0]);
//	  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);
	  osDelay(100); //100 Hz Sampling Rate
  }
  /* USER CODE END ADCProcesing */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
