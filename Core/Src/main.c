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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include<stdbool.h>
#include "mpu6050.h"
#include "MFRC522.h"
#include "integer_type.h"
#include <math.h>
#include "RCFilter.h"
#include <stdarg.h>
#include "KalmanFilter.h"
#include "MQTTSim800.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265

#define ADC_RESOLUTION 4096
#define VOLTAGE_REFERENCE 3.3
#define ACCU_THRESHOLD 12
#define BATT_THRESHOLD 3.7

#define IMU_TS 10
#define GPS_TS 1000

#define ACC_BUFF_LEN 100
#define SPEED_BUFF_LEN 100
#define LOG_LENGTH 2

#define pdTICKSTOMS( xTicks )    ( ( xTicks * 1000 ) / configTICK_RATE_HZ )

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
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for MainProcessTask */
osThreadId_t MainProcessTaskHandle;
const osThreadAttr_t MainProcessTask_attributes = {
  .name = "MainProcessTask",
  .stack_size = 250 * 4,
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
  .stack_size = 1024 * 4,
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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for ADCProcessingTa */
osThreadId_t ADCProcessingTaHandle;
const osThreadAttr_t ADCProcessingTa_attributes = {
  .name = "ADCProcessingTa",
  .stack_size = 250 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LoggingDataTask */
osThreadId_t LoggingDataTaskHandle;
const osThreadAttr_t LoggingDataTask_attributes = {
  .name = "LoggingDataTask",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SendDataTask */
osThreadId_t SendDataTaskHandle;
const osThreadAttr_t SendDataTask_attributes = {
  .name = "SendDataTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MutexSPI1 */
osMutexId_t MutexSPI1Handle;
const osMutexAttr_t MutexSPI1_attributes = {
  .name = "MutexSPI1"
};
/* Definitions for mutexIMU */
osMutexId_t mutexIMUHandle;
const osMutexAttr_t mutexIMU_attributes = {
  .name = "mutexIMU"
};
/* USER CODE BEGIN PV */
	//task Handle

	//IMU Variable
	MPU6050_t MPU6050;
	const float ref_range = 2;
	const float ref_low = -1;
	const float raw_low_x = -1.02;
	const float raw_low_y = -1.04;
	const float raw_low_z = -1.17;
	const float raw_high_x = 1.06;
	const float raw_high_y = 1.1;
	const float raw_high_z = 0.83;
	double speed;
	float acc_buff[ACC_BUFF_LEN];
	double acc, pitch, roll;
	double acc_max, acc_avg;
	double speed_max, speed_avg;
	uint16_t imu_index;
	MovAvgFilter MFiltAx, MFiltAy, MFiltAz;
	MovAvgFilter MFiltGx, MFiltGy, MFiltGz;

	MatrixTransform MatrixTranform;
	//RFID Variable
	uint8_t UID[4]={};
	uint8_t identification = 0;


	//GPS Variable
	double latitude, prev_latitude = 0;
	double longitude, prev_longitude = 0;
	char strUTC[8] = {}; // UTC time in the readable hh:mm:ss format
	uint8_t flag = 0;
	char GPS_msg[20] ={};
	double GPS_distance;
	double GPS_speed;

	//ADC Variables
	float value[3];
	uint32_t buffer[3];
	RCFilter rcFiltFuel, rcFiltAccu, rcFiltBatt;
	MovAvgFilter MAFiltFuel, MAFiltAccu, MAFiltBatt;
	float result_fuel = 0;
	//SD Card Variables
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; 	//Result after operations
	//Let's get some statistics from the SD card
	DWORD free_clusters, free_sectors, total_sectors;
	FATFS* getFreeFs;
	//Ignition Variables
	uint8_t ignition_status = 0;
	uint8_t ignition_logic = 0;

	//Power Variables
	uint8_t power_sel = 0; //accu usage
	uint8_t charging = 0;

	//Logging Data Variables
	float log_acc_max[LOG_LENGTH] = {};
	float log_acc_avg[LOG_LENGTH] = {};
	float log_gyro[LOG_LENGTH] = {};
	float log_speed_max[LOG_LENGTH] = {};
	float log_speed_avg[LOG_LENGTH] = {};
	double log_latitude[LOG_LENGTH] = {};
	double log_longitude[LOG_LENGTH] = {};
	float log_fuel[LOG_LENGTH] = {};
	float log_accu[LOG_LENGTH] = {};
	float log_batt[LOG_LENGTH] = {};
	uint8_t log_ignition_status [LOG_LENGTH] = {};
	char log_timestamp[LOG_LENGTH][32] = {};


	//GSM
	extern char answer[32];
	extern bool x;
	uint16_t queue = 0;
	//Timestamp
	char timestamp[32] = {};
	//Payload
	char payload[4096];


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
static void MX_USART6_UART_Init(void);
void MainProcess(void *argument);
void IMU(void *argument);
void GPS(void *argument);
void RFID(void *argument);
void SDCard(void *argument);
void ADCProcesing(void *argument);
void LoggingData(void *argument);
void SendData(void *argument);

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == UART_SIM800) {
		Sim800_RxCallBack();
		if(x){
			if(strstr(answer,"CCLK")!=NULL){
				memset(timestamp,0,sizeof(timestamp));
				//copy answer to timestamp
				memcpy(timestamp,answer,sizeof(answer));
			}
		}
	} else {
		flag = 1; //GPS Flag
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  for(int i = 0; i<3; i++){
	  value[i] = buffer[i];
  }
  //Filtering Analog reading
}

void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

double distance_on_geoid(double lat1, double lon1, double lat2, double lon2) {

	// Convert degrees to radians
	lat1 = lat1 * M_PI / 180.0;
	lon1 = lon1 * M_PI / 180.0;

	lat2 = lat2 * M_PI / 180.0;
	lon2 = lon2 * M_PI / 180.0;

	// radius of earth in metres
	double r = 6378100;

	// P
	double rho1 = r * cos(lat1);
	double z1 = r * sin(lat1);
	double x1 = rho1 * cos(lon1);
	double y1 = rho1 * sin(lon1);

	// Q
	double rho2 = r * cos(lat2);
	double z2 = r * sin(lat2);
	double x2 = rho2 * cos(lon2);
	double y2 = rho2 * sin(lon2);

	// Dot product
	double dot = (x1 * x2 + y1 * y2 + z1 * z2);
	double cos_theta = dot / (r * r);

	double theta = acos(cos_theta);

	// Distance in Metres
	return r * theta;
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
  MX_FATFS_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  char txBuffer [100] = {};
  sprintf(txBuffer, "Bismillah..\n");
  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of MutexSPI1 */
  MutexSPI1Handle = osMutexNew(&MutexSPI1_attributes);

  /* creation of mutexIMU */
  mutexIMUHandle = osMutexNew(&mutexIMU_attributes);

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
  /* creation of MainProcessTask */
//  MainProcessTaskHandle = osThreadNew(MainProcess, NULL, &MainProcessTask_attributes);

  /* creation of IMUTask */
//  IMUTaskHandle = osThreadNew(IMU, NULL, &IMUTask_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(GPS, NULL, &GPSTask_attributes);

  /* creation of RFIDTask */
//  RFIDTaskHandle = osThreadNew(RFID, NULL, &RFIDTask_attributes);

  /* creation of SDCardTask */
//  SDCardTaskHandle = osThreadNew(SDCard, NULL, &SDCardTask_attributes);

  /* creation of ADCProcessingTa */
//  ADCProcessingTaHandle = osThreadNew(ADCProcesing, NULL, &ADCProcessingTa_attributes);

  /* creation of LoggingDataTask */
//  LoggingDataTaskHandle = osThreadNew(LoggingData, NULL, &LoggingDataTask_attributes);

  /* creation of SendDataTask */
//  SendDataTaskHandle = osThreadNew(SendData, NULL, &SendDataTask_attributes);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, SD_CS_Pin|POWER_SEL_Pin|IGNITION_LOGIC_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : SD_CS_Pin POWER_SEL_Pin IGNITION_LOGIC_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|POWER_SEL_Pin|IGNITION_LOGIC_Pin;
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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IGNITION_SIGNAL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MainProcess */
/**
  * @brief  Function implementing the MainProcessTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MainProcess */
void MainProcess(void *argument)
{
  /* USER CODE BEGIN 5 */
	char txBuffer [200] = {};
	sprintf(txBuffer, "Running Main Process..\n");

	//Initialize Power Control
	HAL_GPIO_WritePin(POWER_SEL_GPIO_Port, POWER_SEL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHARGING_SIGNAL_GPIO_Port, CHARGING_SIGNAL_Pin, GPIO_PIN_RESET);


  /* Infinite loop */
  for(;;)
  {
	  xTaskNotifyGive(IMUTaskHandle);
	  //Identification Check
	  if ((identification == 1)&&(ignition_status == 1)){
//		  xTaskNotifyGive(IMUTaskHandle);
	  } else {
		  //Reset IMU Reading
//		  MPU6050.Ax = 0;
//		  MPU6050.Ay = 0;
//		  MPU6050.Az = 0;
	  }
	  if(UID[0]== 0x29){ // Need to add driver database
		  identification = 1;
	  } else {
		  identification = 0;
	  }
	  //End of Identification Check

	  /* Setting Ignition Switch Logic*/
	  if(identification == 1){
		  //Set Ignition Logic
		  HAL_GPIO_WritePin(IGNITION_LOGIC_GPIO_Port, IGNITION_LOGIC_Pin, GPIO_PIN_SET);
	  }
	  else {
		  HAL_GPIO_WritePin(IGNITION_LOGIC_GPIO_Port, IGNITION_LOGIC_Pin, GPIO_PIN_RESET);
	  }

	  /* Reading Ignition Switch Signal */
	  ignition_status = HAL_GPIO_ReadPin(IGNITION_SIGNAL_GPIO_Port, IGNITION_SIGNAL_Pin);

	  /* Power Management */
	  if(ignition_status == 0){
	  		  //Use Battery
	  		  power_sel = 1;
	  		  HAL_GPIO_WritePin(POWER_SEL_GPIO_Port, POWER_SEL_Pin, GPIO_PIN_SET);
	  	  }
	  	  else{ //Ignition On
	  		  //Always use Accu
	  		  power_sel = 0;
	  		  HAL_GPIO_WritePin(POWER_SEL_GPIO_Port, POWER_SEL_Pin, GPIO_PIN_RESET);
	  		  /* Check Accu Level */
	  		  	  if((MAFiltAccu.out > ACCU_THRESHOLD)&&(MAFiltBatt.out < BATT_THRESHOLD)) {
	  		  		  //Start Charging
	  		  		  //Set Charging Signal High
	  		  		  charging = 1;
	  		  		  HAL_GPIO_WritePin(CHARGING_SIGNAL_GPIO_Port, CHARGING_SIGNAL_Pin, GPIO_PIN_RESET);
	  		  	  }
	  		  	  else { // No charging
	  		  		  charging = 0;
	  		  		  HAL_GPIO_WritePin(CHARGING_SIGNAL_GPIO_Port, CHARGING_SIGNAL_Pin, GPIO_PIN_RESET);
	  		  	  }
	  	  }
	  /* End of Power Management */

	  memset(txBuffer,0,sizeof(txBuffer));
	  /*
	  sprintf(txBuffer,"\nID : %x-%x-%x-%x Ignition : %d SEL : %d CHRG : %d Lat : %.6lf Lon :%.6lf Ax = %.2f Ay = %.2f Az = %.2f V : %.2f Fuel : %.2f Accu : %.2f Batt : %.2f ",
			  UID[0],UID[1],UID[2],UID[3], ignition_status, power_sel, charging,latitude,longitude, MPU6050.Ax, MPU6050.Ay,MPU6050.Az,GPS_speed,MAFiltFuel.out, MAFiltAccu.out, MAFiltBatt.out);
	  */
	  sprintf(txBuffer,"Fuel : %.2f\n",
			  result_fuel);
//	  HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 500);
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
	double ax,ay,az;
	//Clearing Buffer
	memset(txBuffer,0,sizeof(txBuffer));
	sprintf(txBuffer, "Running IMU Task..\n");
	uint8_t ID = MPU6050_Init(&hi2c1);
	sprintf(txBuffer,"IMU Initialization Success : %d \n", ID);
	HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, sizeof(txBuffer), 100);

	//Init Matrix Transformation
	MatrixTransformInit(&MatrixTranform);
	//init filter
	MovAvgFilter_init(&MFiltAy);
	MovAvgFilter_init(&MFiltAy);
	MovAvgFilter_init(&MFiltAz);

	MovAvgFilter_init(&MFiltGx);
	MovAvgFilter_init(&MFiltGy);
	MovAvgFilter_init(&MFiltGz);

	osDelay(pdMS_TO_TICKS(1000));
  /* Infinite loop */
  for(;;)
  {

	//Blocking Until Notified
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	MPU6050_Read_Accel(&hi2c1, &MPU6050);
	MovAvgFilter_Update(&MFiltAx, MPU6050.Ax);
	MovAvgFilter_Update(&MFiltAy, MPU6050.Ay);
	MovAvgFilter_Update(&MFiltAz, MPU6050.Az);

	//Corrected Value
	ax = ((MFiltAx.out-raw_low_x)*ref_range)/(raw_high_x-raw_low_x)+ ref_low;
	ay = ((MFiltAy.out-raw_low_y)*ref_range)/(raw_high_y-raw_low_y)+ ref_low;
	az = ((MFiltAz.out-raw_low_z)*ref_range)/(raw_high_z-raw_low_z)+ ref_low;

	//Frame Transformation
	ax = (round(((0.9987 * ax - 0.0007*ay + 0.0510*az)*100)))/100.0;
	ay = (round(((-0.0007* ax + 0.9996*ay + 0.0265*az)*100)))/100.0;
	az = (round(((-0.0510* ax - 0.0265*ay + 0.9983*az)*100)))/100.0;

	//Calculate Pitch
	roll = atan2(ay,az);
//	roll = round((atan(-ay/sqrt(ax*ax+az*az))*180/PI))*PI/180;
//	roll = round((atan(-MFiltAy.out/sqrt(MFiltAx.out*MFiltAx.out+MFiltAz.out*MFiltAz.out))*180/PI))*PI/180;
	pitch = atan(-ax/sqrt(ay*ay+az*az));

//	if((roll>-(PI/18))&&(roll<(PI/18))){
//		acc = 1*(ay);
//
//	} else {
//		//choose axis
//		acc = 1*(ay + sin(roll));
//	}

	acc = ay;
	//Calculate Speed
	speed += acc* GRAVITY*IMU_TS *0.001 *3.6;

	/*
	vel[0] += MPU6050.Ax * IMU_TS * 0.001; //Vx TS in MS
	vel[1] += MPU6050.Ay * IMU_TS * 0.001; //Vy
	vel[2] += (MPU6050.Az - 5.52) * IMU_TS * 0.001; //Vz
	speed = sqrt(pow(vel[0],2) + pow(vel[1],2) + pow(vel[2],2));
	*/
	osMutexAcquire(mutexIMUHandle, portMAX_DELAY);

	imu_index++;
	//Calculate maximum speed
	if(speed > speed_max){
		speed_max = speed;
	}
	float comp = sin(roll);
	//calculate maximum acceleration
	if(acc > acc_max){
		acc_max = acc;
	}
	//calculate average
	acc_avg = (acc_avg*(imu_index-1)/imu_index) + (acc/imu_index);
	speed_avg = (speed_avg*(imu_index-1)/imu_index) + (speed/imu_index);

	osMutexRelease(mutexIMUHandle);

//	speed = CalSpeed(MPU6050, IMU_TS);
	memset(txBuffer,0,sizeof(txBuffer));
//	sprintf(txBuffer, "roll:%.2f comp:%.3f ax:%.2f ay:%.2f\n", roll,comp,ax,ay);
	sprintf(txBuffer,"acc = %.2f acc_avg = %.2f acc_max = %.2f speed =%.2f speed_avg = %.2f speed_max =%.2f\n",acc,acc_avg, acc_max, speed , speed_avg, speed_max);
//	HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 100);
    osDelay(pdMS_TO_TICKS(IMU_TS));
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
	TickType_t tick_now,tick_prev,dtick = 0;
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

	  osDelay(pdMS_TO_TICKS(5000));
	  HAL_UART_Transmit(&huart2, (unsigned char *) "Start\n", 6, 500);
  /* Infinite loop */
  for(;;)
  {
	  char txBuffer[200] = {};
	  if (flag) {
	  	memset(buffStr, 0, 255);
	  	sprintf(buffStr, "%s", buff);

	  	memset(txBuffer,0,sizeof(txBuffer));
	  	sprintf(txBuffer, "GPS Available..\n");
	  	HAL_UART_Transmit(&huart2, (uint8_t *)buffStr, strlen(buffStr), 70);


	   /*splitting the buffStr by the "\n" delimiter with the strsep() C function
	   	 see http://www.manpagez.com/man/3/strsep/
	    */
	  	char *token, *string;
	  	// actually splitting the string by "\n" delimiter
	  	string = strdup(buffStr);
	  	while ((token = strsep(&string, "\n")) != NULL) {
	  		memset(nmeaSnt, 0, 80);
	  		sprintf(nmeaSnt, "%s", token);

	  		/*memset(txBuffer,0,sizeof(txBuffer));
	  	  	  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 100);
	  	  	*/

	  		// selecting only $GNGLL sentences, combined GPS and GLONAS
	  		// on my GPS sensor this good NMEA sentence is always 50 characters
	  		if ((strstr(nmeaSnt, "$GNGGA") != 0) && (strlen(nmeaSnt) > 49) && strstr(nmeaSnt, "*") != 0) {
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
	  				// splitting the good NMEA sentence into the tokens by the comma delimiter
	  				for (char *pV = strtok(nmeaSnt, ","); pV != NULL; pV = strtok(NULL, ",")) {
	  					/*
	  					memset(txBuffer,0,sizeof(txBuffer));
	  					sprintf(txBuffer,"pV[%d] : %s\n",cnt, pV);
	  					HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 100);
	  					*/
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
	  				sprintf(latDg, token);

	  				//Get Minutes
	  				token = strtok(NULL,".");
	  				memset(latMS, 0, sizeof(latMS));
	  				sprintf(latMS, token);

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

	  				//converting the UTC time in the hh:mm:ss format
	  				memcpy(hH, &utcRaw[0], 2);
	  				hH[2] = '\0';

	  				memcpy(mM, &utcRaw[2], 2);
	  				mM[2] = '\0';

	  				memcpy(sS, &utcRaw[4], 2);
	  				sS[2] = '\0';

	  				strcpy(strUTC, hH);
	  				strcat(strUTC, ":");
	  				strcat(strUTC, mM);
	  				strcat(strUTC, ":");
	  				strcat(strUTC, sS);
	  				strUTC[8] = '\0';

	  				tick_now = xTaskGetTickCount();
	  				dtick = tick_now - tick_prev;
	  				tick_prev = tick_now;

	  				//Calculate Distance
	  				GPS_distance = distance_on_geoid(prev_latitude, prev_longitude, latitude, longitude);
	  				GPS_speed    = (double) (GPS_distance/(pdTICKSTOMS(dtick))*1000); //ms to s

	  				//Update previous location
	  				prev_latitude = latitude;
	  				prev_longitude = longitude;

	  				memset(txBuffer,0,sizeof(txBuffer));
	  				sprintf(txBuffer, "latitude : %f longitude : %f distance : %.2f speed : %.2f\n", latitude, longitude,GPS_distance,GPS_speed);
	  				HAL_UART_Transmit(&huart2, (unsigned char *)txBuffer, strlen(txBuffer), 100);
	  			} //end of the chekcsum data verification
	  		} //end of %GPPGA Sentences selection
	  	}// end of splotting the buffstr by the "\n" delimiter with strsep() c function
	  	flag = 0;

	  }
	  else {
		  memset(txBuffer,0,sizeof(txBuffer));
		  sprintf(txBuffer,"GPS no signal..\n");
		  HAL_UART_Transmit(&huart2, (uint8_t *) txBuffer, sizeof(txBuffer), 100);
	  }
	  osDelay(pdMS_TO_TICKS(GPS_TS));
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
	u_char status, cardstr[MAX_LEN];
//	u_char checksum

	osMutexAcquire(MutexSPI1Handle, portMAX_DELAY);

	MFRC522_Init();
	status = 0;

	while (status != 0x92){
		status = Read_MFRC522(VersionReg);
		sprintf(txBuffer,"Running RC522 ver :%x\n", status);
		HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 5000);
		osDelay(pdMS_TO_TICKS(1000));
	}
	osMutexRelease(MutexSPI1Handle);
	//Printing to PC
	memset(txBuffer,0,sizeof(txBuffer));
	status = 0;
	osDelay(pdMS_TO_TICKS(1000));
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(MutexSPI1Handle, portMAX_DELAY);
	  status = MFRC522_Request(PICC_REQIDL, cardstr);
	  if(status == MI_OK){
		  sprintf(txBuffer,"Card detected ..\n");
		  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 500);
//		  sprintf(txBuffer,"Card Type : %x %x %x\n", cardstr[0],cardstr[1],cardstr[2]);
//		  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 500);
		  memset(txBuffer,0,sizeof(txBuffer));

		  //Anti-collision, return card serial number == 4 bytes
		  status = MFRC522_Anticoll(cardstr);
		  if (status == MI_OK){
//			  checksum1 = cardstr[0] ^ cardstr[1] ^ cardstr[2] ^ cardstr[3];
			  for(int i = 0; i <4 ;i++){
				  UID[i]=cardstr[i];
			  }
			  sprintf(txBuffer,"UID: %x %x %x %x ",(u_char)cardstr[0], (u_char)cardstr[1],(u_char)cardstr[2],(u_char)cardstr[3]);
			  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
			  memset(txBuffer,0,sizeof(txBuffer));
			  if(UID[0]== 0x29){ // Need to add driver database
				  sprintf(txBuffer,"Verified ..\n");
			  } else {
				  sprintf(txBuffer,"NOT Verified ..\n");
			  }
			  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
		  }
	  }
	  else {
		  memset(txBuffer,0,sizeof(txBuffer));
		  sprintf(txBuffer,"Status :%x\n", status);
//		  HAL_UART_Transmit(&huart2, (unsigned char*) txBuffer, sizeof(txBuffer), 5000);
		  //		  sprintf(txBuffer,"Finding ..\n");
//		  HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
	  }
	  osMutexRelease(MutexSPI1Handle);

    osDelay(250);
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
	osMutexAcquire(MutexSPI1Handle, portMAX_DELAY);
	osDelay(pdMS_TO_TICKS(2000));
	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		osMutexRelease(MutexSPI1Handle);
	}

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
    	myprintf("f_getfree error (%i)\r\n", fres);
    	osMutexRelease(MutexSPI1Handle);
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

//    myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    //Now let's try to open file "test.txt"
    fres = f_open(&fil, "write.txt", FA_READ);
    if (fres != FR_OK) {
    	myprintf("f_open error (%i)\r\n");
    	osMutexRelease(MutexSPI1Handle);
    }else {
//    	myprintf("I was able to open 'tesjson.txt' for reading!\r\n");
    }


    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[100];

    //Name File Format
    char folder_name[10] ="logging/";
    char front_name [10] = "data";
    char back_name [20] = {};
    char filename [50] = {};
    //We can either use f_read OR f_gets to get data out of files
    //f_gets is a wrapper on f_read that does some string formatting for us
    TCHAR* rres = f_gets((TCHAR*)readBuf, 100, &fil);
    if(rres != 0) {
//    	myprintf("Read string from 'tesjson.txt'' contents: %s\r\n", readBuf);
    } else {
    	myprintf("f_gets error (%i)\r\n", fres);
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);

    //Now let's try and write a file "write.txt"
    fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if(fres == FR_OK) {
//    	myprintf("I was able to open 'write.txt' for writing\r\n");
    } else {
    	myprintf("f_open error (%i)\r\n", fres);
    }

    //Copy in a string
//    strncpy((char*)readBuf, "a new file is made!", 19);
    char* kirimjson = "{'Lokasi':'-6.914744,107.609810','Time_Stamp':'27 April 2021, 20:22'}" ;
    uint32_t length_var = strlen(kirimjson);
    strncpy((char*)readBuf, kirimjson, length_var);
    UINT bytesWrote;
    fres = f_write(&fil, readBuf, length_var, &bytesWrote);
    if(fres == FR_OK) {
//    	myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
    } else {
    	myprintf("f_write error (%i)\r\n");
    }
    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);
//    f_unlink("/write.txt"); Buat ngedelete file
    //We're done, so de-mount the drive
    f_mount(NULL, "", 0);

    osMutexRelease(MutexSPI1Handle);
  /* Infinite loop */
  for(;;)
  {
	 //Wait notification from Send Data Task
	 ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	 myprintf("Saving in ");
	 //Acquire Mutex
	 osMutexAcquire(MutexSPI1Handle, portMAX_DELAY);
	 //Mounting SD Card
	 fres = f_mount(&FatFs, "", 1); //1=mount now
	 if (fres != FR_OK) {
		 myprintf("f_mount error (%i)\r\n", fres);
		 osMutexRelease(MutexSPI1Handle);
	 }

	 //Determine filename
	 //Clearing buffer
	 memset(filename,0,sizeof(filename));
	 memset(back_name,0,sizeof(back_name));
	 //write folder name first
	 sprintf(filename,"%s",folder_name);
	 //write back_name in buffer
	 sprintf(back_name, "_%d.txt",queue);
	 //Concatenate foldername and front name
	 strcat(filename,front_name);
	 //concatenate filename and back_name
	 strcat(filename,back_name);
	 myprintf("filename : %s\n", filename);
	 fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	 if (fres != FR_OK) {
		 myprintf("f_open error (%i)\r\n");
		 osMutexRelease(MutexSPI1Handle);
	 } else {
		 //Write File
		 fres = f_write(&fil, payload, strlen(payload), &bytesWrote);
		 if(fres == FR_OK) {
		 //    	myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		 } else {
			 myprintf("f_write error (%i)\r\n");
		 }
		 //Closing File
		 f_close(&fil);
	 }
	 //We're done, so de-mount the drive
	 f_mount(NULL, "", 0);
	 osMutexRelease(MutexSPI1Handle);
    osDelay(pdMS_TO_TICKS(100));
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

	/* Initialize Input Value */
	float input_fuel, input_accu, input_batt = 0;
	float result_accu, result_batt = 0;
	/* Initialize RC Filter */
	RCFilter_Init(&rcFiltFuel, 5.0f, 100.0f);

	/*Initialize Moving Average Filter*/
	MovAvgFilter_init(&MAFiltFuel);
	MovAvgFilter_init(&MAFiltAccu);
	MovAvgFilter_init(&MAFiltBatt);
	/* Start ADC */
	HAL_ADC_Start_DMA(&hadc1, buffer, 3);
	sprintf(txBuffer,"ADC Intialization Success..\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);
	osDelay(pdMS_TO_TICKS(1000));
  /* Infinite loop */
  for(;;)
  {
	  input_accu = (value[1]/ADC_RESOLUTION)* 3.3 * 12.16/0.77;
	  input_batt = (value[0]/ADC_RESOLUTION)* 3.3 * 7.9/2.14;
	  input_fuel = ((value[2]/ADC_RESOLUTION)* 3.295)*1;

//	  input fuel = ((floor((input_fuel*100))
	  RCFilter_Update(&rcFiltFuel, input_fuel);
	  MovAvgFilter_Update(&MAFiltFuel, input_fuel);
	  MovAvgFilter_Update(&MAFiltAccu, input_accu);
	  MovAvgFilter_Update(&MAFiltBatt, input_batt);

	  MAFiltFuel.out = ((floor(MAFiltFuel.out*100))/100.0);
	  result_fuel = (1-((MAFiltFuel.out-0.2)/0.8))*3.8;

	  memset(txBuffer,0,sizeof(txBuffer));
	  sprintf(txBuffer,"RawAccu:%.2f RawBatt:%.2f Accu:%.2f Batt:%.2f\n", input_accu,input_batt,MAFiltAccu.out, MAFiltBatt.out);
	  HAL_UART_Transmit(&huart2, (unsigned char *) txBuffer, sizeof(txBuffer), 500);
	  osDelay(pdMS_TO_TICKS(10)); //100 Hz Sampling Rate
  }
  /* USER CODE END ADCProcesing */
}

/* USER CODE BEGIN Header_LoggingData */
/**
* @brief Function implementing the LoggingDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoggingData */
void LoggingData(void *argument)
{
  /* USER CODE BEGIN LoggingData */

	uint8_t index = 0;
	char txBuffer[100];
	memset(txBuffer,0,sizeof(txBuffer));
	sprintf(txBuffer,"Starting Logging..\n");
	HAL_UART_Transmit(&huart2, (uint8_t *) txBuffer , sizeof(txBuffer), 100);
	osDelay(pdMS_TO_TICKS(1000));
  /* Infinite loop */
  for(;;)
  {
	SIM800_SendCommand("AT+CCLK?\r\n", "OK\r\n", CMD_DELAY);
	log_acc_avg[index] = acc_avg;
	log_acc_max[index] = acc_max;
	log_gyro[index] = MFiltGz.out;
	log_speed_max[index] = speed_max;
	log_speed_avg[index] = speed_avg;
	log_latitude[index] = latitude;
	log_longitude[index] = longitude;
	log_fuel[index] = MAFiltFuel.out;
	log_accu[index] = MAFiltAccu.out;
	log_batt[index] = MAFiltBatt.out;
	log_ignition_status[index] = 0;

	memset(log_timestamp[index], 0, sizeof(log_timestamp[index]));
	memcpy(log_timestamp[index], &timestamp[8], 20);
	// Change timezone to UTC
	log_timestamp[index][19] = '0';
	memset(txBuffer,0,sizeof(txBuffer));
	sprintf(txBuffer,"index : %d Timestamp: %s\n", index, log_timestamp[index]);
//	HAL_UART_Transmit(&huart2, (uint8_t *) txBuffer , sizeof(txBuffer), 100);
	//Increment Index
	index++;
	if(index >= LOG_LENGTH){

		index = 0;
		//clearing array
		osMutexAcquire(mutexIMUHandle, portMAX_DELAY);
		imu_index = 0;
		speed_avg = 0;
		speed_max = 0;
		acc_avg = 0;
		acc_max = 0;

		osMutexRelease(mutexIMUHandle);
		memset(payload,0,sizeof(payload)); //clearing form
		//Create Payload Form
		sprintf(payload,"`{\\\"tw\\\":[\\\"%s\\\",\\\"%s\\\"],\\\"lk\\\":[\\\"%f,%f\\\",\\\"%f,%f\\\"],\\\"k_max\\\":[%.2f,%.2f],\\\"k_avg\\\":[%.2f,%.2f],\\\"k_ang\\\":[%.2f,%.2f],\\\"a_max\\\":[%.2f,%.2f],\\\"a_avg\\\":[%.2f,%.2f],\\\"tb\\\":[%.2f,%.2f],\\\"ta\\\":[%.2f,%.2f],\\\"bb\\\":[%.2f,%.2f],\\\"si\\\":[%d,%d],\\\"kurir\\\":17}`",
				log_timestamp[0], log_timestamp[1], log_latitude[0],log_longitude[0], log_latitude[1], log_longitude[1], log_speed_max[0],log_speed_max[1],log_speed_avg[0],log_speed_avg[1],log_gyro[0], log_gyro[1],log_acc_max[0],log_acc_max[1], log_acc_avg[0], log_acc_avg[1], log_batt[0], log_batt[1], log_accu[0], log_accu[1],log_fuel[0], log_fuel[1], log_ignition_status[0], log_ignition_status[1]);


//		sprintf(payload,"`{\\\"tw\\\":[\\\"2021-05-26T07:47:21.810Z\\\",\\\"2021-05-26T07:47:21.916Z\\\"],\\\"lk\\\":[\\\"-6.8385324382250205,107.6955557148961\\\",\\\"-6.828578043365432,107.6112333432733\\\"],\\\"k_max\\\":[43.99,55.65],\\\"k_avg\\\":[76.99,82.97],\\\"k_ang\\\":[13.24,2.99],\\\"a_max\\\":[3.62,83.87],\\\"a_avg\\\":[91.39,62.66],\\\"tb\\\":[6.07,6.39],\\\"ta\\\":[5.10,1.99],\\\"bb\\\":[0.92,0.98],\\\"si\\\":[0,1],\\\"kurir\\\":1}`");
//		xTaskNotifyGive(SendDataTaskHandle);
	}

//	HAL_UART_Transmit(&huart2, (uint8_t*)payload, sizeof(payload), 100);
//	for(int i = 0; i<LOG_LENGTH; i++){
//				memset(txBuffer, 0, sizeof(txBuffer));
//				sprintf(txBuffer,"\n[%d] acc_avg: %.2f acc_max: %.2f speed_avg: %.2f speed_max: %.2f",
//						i,log_acc_avg[i],log_acc_max[i], log_speed_avg[i], log_speed_max[i]);
//				HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, sizeof(txBuffer), 100);
//			}
    osDelay(pdMS_TO_TICKS(10*1000));
  }
  /* USER CODE END LoggingData */
}

/* USER CODE BEGIN Header_SendData */
/**
* @brief Function implementing the SendDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendData */
void SendData(void *argument)
{
  /* USER CODE BEGIN SendData */
	char txBuffer[50] ={};
	uint8_t pub_status = 1;
	char * topic = "client-1";
	uint8_t error = 0;
	error += SIM800_Init();

	if(error >0 ){
		sprintf(txBuffer,"GSM Initialization fail\n");
	} else {
		sprintf(txBuffer,"GSM Initialization success\n");
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
	//Connecting to broker
	error += MQTT_Connect("indosatgprs", "", "", "3.210.14.248", 1883, "faisa", "disadacepetlulus", "client-1", 1000);

	memset(txBuffer,0,sizeof(txBuffer));
	if(error >0 ){
		memset(txBuffer,0,sizeof(txBuffer));
		sprintf(txBuffer,"Reconnecting..\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
		error = 0;
		error += SIM800_SendCommand("AT+RST=1\r\n", "READY\r\n", 1000);
		osDelay(pdMS_TO_TICKS(20*1000));
		error += MQTT_Connect("indosatgprs", "", "", "3.210.14.248", 1883, "faisa", "disadacepetlulus", "client-1", 1000);
	}
	else {
		memset(txBuffer,0,sizeof(txBuffer));
		sprintf(txBuffer,"Connected..\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) txBuffer, sizeof(txBuffer), 100);
	}
	osDelay(pdMS_TO_TICKS(1000));

	/* Infinite loop */
  for(;;)
  {
	  //Blocking Until Notified
	  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	  //Send Payload
//	  HAL_UART_Transmit(&huart2, (uint8_t *) payload , sizeof(payload), 100);
	  pub_status = MQTT_Pub(topic, payload);
	  memset(txBuffer, 0, sizeof(txBuffer));
	  sprintf(txBuffer,"Sending : %d\n", pub_status);
//	  HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, sizeof(txBuffer), 100);
	  if(pub_status!=0){
		  //Sending Fail
		  queue++;
//		  xTaskNotifyGive(SDCardTaskHandle);
	  }
	  osDelay(pdMS_TO_TICKS(5000));
  }
  /* USER CODE END SendData */
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
