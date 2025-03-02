/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMAGE_SIZE 784

QueueHandle_t xQueue_DigitResult;
QueueHandle_t xQueue_Display;
QueueHandle_t xQueue_SoundCommand;

int8_t image_buffer[28 * 28];
int8_t player_guess = 9;

int8_t image_ready = 0;
int8_t process_ready = 0;


int8_t game_started = 0;
uint8_t random_digit = 0;

uint32_t wav_index;
uint32_t playsong = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Task01input */
osThreadId_t Task01inputHandle;
const osThreadAttr_t Task01input_attributes = {
  .name = "Task01input",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Task02img */
osThreadId_t Task02imgHandle;
const osThreadAttr_t Task02img_attributes = {
  .name = "Task02img",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Task03game */
osThreadId_t Task03gameHandle;
const osThreadAttr_t Task03game_attributes = {
  .name = "Task03game",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for Task04oled */
osThreadId_t Task04oledHandle;
const osThreadAttr_t Task04oled_attributes = {
  .name = "Task04oled",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Task05music */
osThreadId_t Task05musicHandle;
const osThreadAttr_t Task05music_attributes = {
  .name = "Task05music",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
void Taskuser(void *argument);
void Task02image(void *argument);
void Task03logic(void *argument);
void Task04display(void *argument);
void Task05song(void *argument);

/* USER CODE BEGIN PFP */

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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
//  MX_USART3_UART_Init();
  ssd1306_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);
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
//  xQueue_Image = xQueueCreate(1, IMAGE_SIZE);
  xQueue_DigitResult = xQueueCreate(1, 10);
  xQueue_Display = xQueueCreate(1, 20);
  xQueue_SoundCommand = xQueueCreate(1, 10);


  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01input */
  Task01inputHandle = osThreadNew(Taskuser, NULL, &Task01input_attributes);

  /* creation of Task02img */
  Task02imgHandle = osThreadNew(Task02image, NULL, &Task02img_attributes);

  /* creation of Task03game */
  Task03gameHandle = osThreadNew(Task03logic, NULL, &Task03game_attributes);

  /* creation of Task04oled */
  Task04oledHandle = osThreadNew(Task04display, NULL, &Task04oled_attributes);

  /* creation of Task05music */
  Task05musicHandle = osThreadNew(Task05song, NULL, &Task05music_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  // top 1223 bot 1
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1223;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
//    HAL_UART_Transmit(&huart3, (uint8_t *)"Received", 8, HAL_MAX_DELAY);

	  image_ready = 1;

//	  HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
  }
}



void PlayWav(void) {
    wav_index = 0;
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Taskuser */
/**
  * @brief  Function implementing the Task01input thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Taskuser */
void Taskuser(void *argument)
{
  /* USER CODE BEGIN 5 */
//	char buffer[10];

  /* Infinite loop */
  for(;;)
  {

	  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == 0) {
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);

	        if (game_started == 0) {
	          srand(HAL_GetTick());
	          random_digit = rand() % 10;
	          game_started = 1;
	          ssd1306_Fill(Black);
	          ssd1306_UpdateScreen();
	          HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);
	        }
	        else{
	        	srand(HAL_GetTick());
	        	random_digit = rand() % 10;
	        }
	      }

	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, game_started ? 1 : 0);

	 HAL_UART_Transmit(&huart3, "1", 1, HAL_MAX_DELAY);
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task02image */
/**
* @brief Function implementing the Task02img thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task02image */
void Task02image(void *argument)
{
  /* USER CODE BEGIN Task02image */
	char buffer[10];
  /* Infinite loop */
  for(;;)
  {
	   if(image_ready == 1)
	    {

	      MX_X_CUBE_AI_Process();

	      sprintf(buffer, "%d", player_guess);
	      HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

//	      HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);

	      memset(image_buffer, 0, sizeof(image_buffer));

	      image_ready = 0;

	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	      xQueueSend(xQueue_DigitResult, &buffer, portMAX_DELAY);

	    }

	  HAL_UART_Transmit(&huart3, "2", 1, HAL_MAX_DELAY);
    osDelay(10);
  }
  /* USER CODE END Task02image */
}

/* USER CODE BEGIN Header_Task03logic */
/**
* @brief Function implementing the Task03game thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task03logic */
void Task03logic(void *argument)
{
  /* USER CODE BEGIN Task03logic */
	char buffer[10];
	char messageDis[20];
	char messageSound[10];
	uint8_t song = 0;
  /* Infinite loop */
  for(;;)
  {

	  if(game_started == 1){

	   if (xQueueReceive(xQueue_DigitResult, &buffer, portMAX_DELAY) == pdTRUE) {


		            	 uint8_t number_player = atoi(buffer);

		            	 if (random_digit  == number_player) {

		            		 srand(HAL_GetTick());
		            		 random_digit = rand() % 10;

//		            		 game_started = 1; //1

							 sprintf((char*)messageDis, "Correct!");
							 xQueueSend(xQueue_Display, &messageDis, portMAX_DELAY);

							 song = 3;
							 sprintf(messageSound, "%d", song);
							 xQueueSend(xQueue_SoundCommand, &messageSound, portMAX_DELAY);

							 game_started = 0; //1

		            	 }
		            	 else if (random_digit < number_player) {

		            		 sprintf((char*)messageDis, "%d Too much",number_player);
		            	 	 xQueueSend(xQueue_Display, &messageDis, portMAX_DELAY);


		            	 	 song = 1;
		            	 	 sprintf(messageSound, "%d", song);
		            	 	 xQueueSend(xQueue_SoundCommand, &messageSound, portMAX_DELAY);
		            	 }
		            	 else if (random_digit > number_player) {

		            		 sprintf((char*)messageDis, "%d Too few",number_player);
		            	 	 xQueueSend(xQueue_Display, &messageDis, portMAX_DELAY);

		            	 	 song = 2;
		            	 	 sprintf(messageSound, "%d", song);
		            	 	 xQueueSend(xQueue_SoundCommand, &messageSound, portMAX_DELAY);
		            	 }


	   }
	 }


	  HAL_UART_Transmit(&huart3, "3", 1, HAL_MAX_DELAY);
    osDelay(10);
  }
  /* USER CODE END Task03logic */
}

/* USER CODE BEGIN Header_Task04display */
/**
* @brief Function implementing the Task04oled thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task04display */
void Task04display(void *argument)
{
  /* USER CODE BEGIN Task04display */
	char messageDis[20];
	char buffer[10];
	char result[10];
	char start_message[20] = "Guess the";
	char end_message[20] = "number?";
  /* Infinite loop */
  for(;;)
  {

	  if(game_started == 1){
		  if (xQueueReceive(xQueue_Display, messageDis, portMAX_DELAY) == pdTRUE) {

			  	  	  	sprintf(buffer, "%d", game_started);
			  	  	  	sprintf(result, "%d", random_digit);

						ssd1306_Fill(Black);

						ssd1306_SetCursor(5,20);
						ssd1306_WriteString(messageDis, Font_11x18, White);

//						ssd1306_SetCursor(5,15);
//						ssd1306_WriteString(buffer, Font_7x10, White);
//
//						ssd1306_SetCursor(5,15*2);
//						ssd1306_WriteString(result, Font_7x10, White);

						ssd1306_UpdateScreen();

		  }
	  }
	  else{

		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(5,15);
		  ssd1306_WriteString(start_message, Font_11x18, White);
		  ssd1306_SetCursor(50,32);
		  ssd1306_WriteString(end_message, Font_11x18, White);
		  ssd1306_UpdateScreen();

	  }

	  HAL_UART_Transmit(&huart3, "4", 1, HAL_MAX_DELAY);
    osDelay(20);
  }
  /* USER CODE END Task04display */
}

/* USER CODE BEGIN Header_Task05song */
/**
* @brief Function implementing the Task05music thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task05song */
void Task05song(void *argument)
{
  /* USER CODE BEGIN Task05song */
	char messageSound[10];
  /* Infinite loop */
  for(;;)
  {
	  if(game_started == 1){

		  if (xQueueReceive(xQueue_SoundCommand, messageSound, portMAX_DELAY) == pdTRUE) {

				  uint8_t number_song = atoi(messageSound);
							 if (number_song == 1){

								playsong = 1;
								PlayWav();
								HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);

							 }
							 else if (number_song == 2){

								playsong = 2;
								PlayWav();
								HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);
							 }
							 else if (number_song == 3){

								playsong = 3;
								PlayWav();
							    HAL_UART_Receive_IT(&huart3, (uint8_t*)image_buffer, 28 * 28);
						     }

			}
	  }

	  HAL_UART_Transmit(&huart3, "5", 1, HAL_MAX_DELAY);
    osDelay(10);
  }
  /* USER CODE END Task05song */
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
