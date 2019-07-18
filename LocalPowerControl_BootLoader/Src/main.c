/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_if.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned int Timer1ms;
unsigned int TimerJumpToApp;
unsigned int TimerRefreshIwdg;
unsigned int TimerInputScan;
unsigned int TimerMcuStatusLed;
unsigned int TimerUart3Parsing;
unsigned int TimeOutValue_JumpToApp;
pFunction JumpToApplication;
unsigned int JumpAddress;
unsigned char McuStatusLedState;
unsigned char Uart3TxBuf[UART3_TX_BUF_SIZE];
unsigned char Uart3RxBuf[UART3_RX_BUF_SIZE];
unsigned short Uart3RxWrCnt;
unsigned short Uart3RxRdCnt;
unsigned char Uart3RxData;
unsigned char Uart3ParsingState;
unsigned short Uart3Cks;
unsigned short Uart3DataLength;
unsigned short Uart3DataCnt;
unsigned char Uart3Cmd;
unsigned char Uart3DataBuf[UART3_DATA_BUF_SIZE];
unsigned char ChannelRelayState;
unsigned char InputScanState;
unsigned int InputOld;
unsigned int InputNow;

const unsigned short Crc16Table[256] = {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
unsigned short VirtAddVarTab[NB_OF_VAR] = {
	0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,0x0009,0x000A,0x000B,0x000C,0x000D,0x000E,0x000F,	// 00~15 (for BootLoader)
	0x0010,0x0011,0x0012,0x0013,0x0014,0x0015,0x0016,0x0017,0x0018,0x0019,0x001A,0x001B,0x001C,0x001D,0x001E,0x001F,	// 16~31 (for App)
	0x0020,0x0021,0x0022,0x0023,0x0024,0x0025,0x0026,0x0027,0x0028,0x0029,0x002A,0x002B,0x002C,0x002D,0x002E,0x002F,	// 32~47 (for App)
	0x0030,0x0031,0x0032,0x0033,0x0034,0x0035,0x0036,0x0037,0x0038,0x0039,0x003A,0x003B,0x003C,0x003D,0x003E,0x003F,	// 48~63 (for App)
};
unsigned short VarDataTab[NB_OF_VAR] = {
	0x1234,0,0,0,0,RELAY_OFF,0,
};
unsigned short VarValue = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Eeprom_Check_Init(void);
unsigned short Eeprom_Read(unsigned short Addr);
unsigned short Eeprom_Write(unsigned short Addr, unsigned short Value);
void JumpToAppMain(void);
void CheckJumpApp(void);
void Channel_Relay_Cont(unsigned char Channel, GPIO_PinState OnOff);
void Refresh_IWDG(void);
void Led_All_Cont(void);
unsigned short Calculation_Crc16(unsigned char*Buf, unsigned int Length);
void Input_Scan(void);
void MCU_Status_Led_Cont(void);
void Send_BoardToPc_WithCks(unsigned char* pBuf, unsigned short Length);
void BoardToPC_AnsReset(void);
void BoardToPC_AnsJumpToApp(void);
void Uart3_Parsing(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(TimerJumpToApp)
	{
		TimerJumpToApp--;
	}
	if(TimerRefreshIwdg)
	{
		TimerRefreshIwdg--;
	}
	if(TimerInputScan)
	{
		TimerInputScan--;
	}
	if(TimerMcuStatusLed)
	{
		TimerMcuStatusLed--;
	}
	if(TimerUart3Parsing)
	{
		TimerUart3Parsing--;
	}
	if(Timer1ms)
	{
		Timer1ms--;
		if(Timer1ms==0)
		{
			Timer1ms = 1000;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		Uart3RxBuf[Uart3RxWrCnt] = Uart3RxData;
		HAL_UART_Receive_IT(&huart3, &Uart3RxData, 1);
		Uart3RxWrCnt++;
		if(Uart3RxWrCnt==UART3_RX_BUF_SIZE)
		{
			Uart3RxWrCnt = 0;
		}
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
	Eeprom_Check_Init();
	if(Eeprom_Read(EEPROM_IDX_UPDATE_RSLT)==0 && Eeprom_Read(EEPROM_IDX_RESET_REQ)==0)
	{
		JumpToAppMain();
	}
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
  	HAL_UART_Receive_IT(&huart3, &Uart3RxData, 1);
	if(Eeprom_Read(EEPROM_IDX_RESET_REQ)==1)	// Reset from App
	{		
		TimeOutValue_JumpToApp = Eeprom_Read(EEPROM_IDX_JUMP_TO_APP_TIMEOUT);
		TimeOutValue_JumpToApp *= 1000;
		Eeprom_Write(EEPROM_IDX_JUMP_TO_APP_TIMEOUT, 0);
		Eeprom_Write(EEPROM_IDX_RESET_REQ, 0);
		BoardToPC_AnsReset();		
	}
	else	// Emergency Mode
	{
		TimeOutValue_JumpToApp = 1200000;	// 1200 x 1000ms = 20minute
	}
	TimerJumpToApp = TimeOutValue_JumpToApp;	
	ChannelRelayState = Eeprom_Read(EEPROM_IDX_CHANNEL_RELAY_STATUS);
	Channel_Relay_Cont(ChannelRelayState, GPIO_PIN_SET);
	Channel_Relay_Cont(~ChannelRelayState, GPIO_PIN_RESET);
	Led_All_Cont();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	Refresh_IWDG();
	CheckJumpApp();
	Led_All_Cont();
	Uart3_Parsing();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 625;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RELAY_CTR_2_Pin|RELAY_CTR_3_Pin|RELAY_CTR_4_Pin|RELAY_CTR_5_Pin 
                          |RELAY_CTR_6_Pin|RELAY_CTR_7_Pin|RELAY_CTR_0_Pin|RELAY_CTR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EXT_IN_1_LED_Pin|EXT_IN_2_LED_Pin|MCU_STATUS_Pin|LAN_RESET_Pin 
                          |INPUT_DET_0_LED_Pin|INPUT_DET_1_LED_Pin|INPUT_DET_2_LED_Pin|INPUT_DET_3_LED_Pin 
                          |INPUT_DET_4_LED_Pin|INPUT_DET_5_LED_Pin|INPUT_DET_6_LED_Pin|INPUT_DET_7_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY_CTR_2_Pin RELAY_CTR_3_Pin RELAY_CTR_4_Pin RELAY_CTR_5_Pin 
                           RELAY_CTR_6_Pin RELAY_CTR_7_Pin RELAY_CTR_0_Pin RELAY_CTR_1_Pin */
  GPIO_InitStruct.Pin = RELAY_CTR_2_Pin|RELAY_CTR_3_Pin|RELAY_CTR_4_Pin|RELAY_CTR_5_Pin 
                          |RELAY_CTR_6_Pin|RELAY_CTR_7_Pin|RELAY_CTR_0_Pin|RELAY_CTR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_DET_0_Pin INPUT_DET_1_Pin INPUT_DET_2_Pin INPUT_DET_3_Pin 
                           INPUT_DET_4_Pin INPUT_DET_5_Pin INPUT_DET_6_Pin INPUT_DET_7_Pin 
                           AUX_IN_1_Pin AUX_IN_2_Pin */
  GPIO_InitStruct.Pin = INPUT_DET_0_Pin|INPUT_DET_1_Pin|INPUT_DET_2_Pin|INPUT_DET_3_Pin 
                          |INPUT_DET_4_Pin|INPUT_DET_5_Pin|INPUT_DET_6_Pin|INPUT_DET_7_Pin 
                          |AUX_IN_1_Pin|AUX_IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF10 PF11 PF12 PF13 
                           PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12 
                           PB13 PB14 PB15 PB5 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3 
                           PG4 PG5 PG6 PG7 
                           PG8 PG9 PG10 PG11 
                           PG12 PG13 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11 
                           PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_IN_1_LED_Pin EXT_IN_2_LED_Pin MCU_STATUS_Pin LAN_RESET_Pin 
                           INPUT_DET_0_LED_Pin INPUT_DET_1_LED_Pin INPUT_DET_2_LED_Pin INPUT_DET_3_LED_Pin 
                           INPUT_DET_4_LED_Pin INPUT_DET_5_LED_Pin INPUT_DET_6_LED_Pin INPUT_DET_7_LED_Pin */
  GPIO_InitStruct.Pin = EXT_IN_1_LED_Pin|EXT_IN_2_LED_Pin|MCU_STATUS_Pin|LAN_RESET_Pin 
                          |INPUT_DET_0_LED_Pin|INPUT_DET_1_LED_Pin|INPUT_DET_2_LED_Pin|INPUT_DET_3_LED_Pin 
                          |INPUT_DET_4_LED_Pin|INPUT_DET_5_LED_Pin|INPUT_DET_6_LED_Pin|INPUT_DET_7_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Eeprom_Check_Init(void)
{
	unsigned char Cnt = 0;
	unsigned short Ret = 0;

	HAL_FLASH_Unlock();
	EE_Init();
	Ret = EE_ReadVariable(EEPROM_IDX_EEPROM_INIT_FLAG, &VarValue);	// EEPROM Addr0 is for EEPROM Initialization
	switch(Ret)
	{
		case 0x0000:	// Variable was found
			if(VarValue != 0x1234)	// EEPROM Address is not initialized
			{	// Initialize EEPROM Address
				for(Cnt=0; Cnt<NB_OF_VAR; Cnt++)
				{
					Eeprom_Write(Cnt, VarDataTab[Cnt]);
				}
			}
			break;
		case 0x0001:	// Variable was not found
			for(Cnt=0; Cnt<NB_OF_VAR; Cnt++)
			{	// Initialize EEPROM Address
				Eeprom_Write(Cnt, VarDataTab[Cnt]);
			}
			break;
		case NO_VALID_PAGE:
			break;
	}
}

unsigned short Eeprom_Read(unsigned short Addr)
{
	unsigned short Val;

	EE_ReadVariable(Addr, &Val);
	return Val;
}

unsigned short Eeprom_Write(unsigned short Addr, unsigned short Value)
{
	unsigned short Ret;

	HAL_FLASH_Unlock();
	Ret = EE_WriteVariable(Addr, Value);
	HAL_FLASH_Lock();
	return Ret;
}

void JumpToAppMain(void)
{
	/* Jump to user application */
	JumpAddress = *(__IO uint32_t*) (APP_MAIN_ADDR+ 4);
	JumpToApplication = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
	__set_MSP(*(__IO uint32_t*)APP_MAIN_ADDR);
	JumpToApplication();
}

void CheckJumpApp(void)
{
	if(TimerJumpToApp==0)
	{
		JumpToAppMain();
	}
}

void Channel_Relay_Cont(unsigned char Channel, GPIO_PinState OnOff)
{
	if(OnOff==RELAY_ON)	// Channel Power On(Relay Off)
	{
		HAL_GPIO_WritePin(RELAY_CTR_0_GPIO_Port, Channel, GPIO_PIN_RESET);
	}
	else	// Channel Power Off(Relay On)
	{
		HAL_GPIO_WritePin(RELAY_CTR_0_GPIO_Port, Channel, GPIO_PIN_SET);
	}
}

void Led_Cont(unsigned char Led, GPIO_PinState OnOff)
{
	HAL_GPIO_WritePin(INPUT_DET_0_LED_GPIO_Port, Led, OnOff);
}

void Refresh_IWDG(void)
{
	if(TimerRefreshIwdg==0)
	{
		HAL_IWDG_Refresh(&hiwdg);
		TimerRefreshIwdg = REFRESH_IWDG_TIMEOUT;
	}
}

void Led_All_Cont(void)
{
	if(TimerMcuStatusLed==0)
	{
		if(McuStatusLedState==0)
		{
			HAL_GPIO_WritePin(MCU_STATUS_GPIO_Port, MCU_STATUS_Pin, GPIO_PIN_SET);
			McuStatusLedState = 1;
		}
		else
		{
			HAL_GPIO_WritePin(MCU_STATUS_GPIO_Port, MCU_STATUS_Pin, GPIO_PIN_RESET);
			McuStatusLedState = 0;
		}
		TimerMcuStatusLed = 250;
	}
}

unsigned short Calculation_Crc16(unsigned char*Buf, unsigned int Length)
{
	unsigned int Cnt;
	unsigned short Crc = 0;
	
	for( Cnt = 0; Cnt < Length; Cnt++)
	{
		Crc = (Crc<<8) ^ Crc16Table[((Crc>>8) ^ *Buf)&0x00FF];
		Buf++;
	}
	
	return Crc;
}

void Send_BoardToPc_WithCks(unsigned char* pBuf, unsigned short Length)
{
	unsigned short Cnt = 0;
	unsigned short CheckSum = 0;

	pBuf[0] = STX_BOARD_TO_PC;
	pBuf[1] = PC_ADDR;
	pBuf[2] = BOARD_ADDR;
	*(unsigned short*)(pBuf+3) = Length;
	for(Cnt=0; Cnt<(Length+6); Cnt++)
	{
		CheckSum += pBuf[Cnt];
	}
	*(unsigned short*)(pBuf+Length+6) = CheckSum;
	pBuf[Length+8] = END_BOARD_TO_PC;

	HAL_UART_Transmit(&huart3, pBuf, Length+9, 100);
}

void BoardToPC_AnsReset(void)
{
	Uart3TxBuf[5] = BOARD_TO_PC_ANS_RESET;
	Uart3TxBuf[6] = 0;
	Send_BoardToPc_WithCks(Uart3TxBuf, 1);
}

void Uart3_Parsing(void)
{
	if(Uart3RxRdCnt != Uart3RxWrCnt)
	{
		unsigned char Rx3Data = 0;		

		Rx3Data = Uart3RxBuf[Uart3RxRdCnt];
		switch(Uart3ParsingState)
		{
			case 0:	// STX
				if(Rx3Data==0x7E)
				{
					Uart3Cks = 0;
					Uart3Cks += Rx3Data;
					Uart3ParsingState++;
					TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				}
				break;
			case 1:	// DA(Destination Addr)
				if(Rx3Data==BOARD_ADDR)
				{
					Uart3Cks += Rx3Data;
					Uart3ParsingState++;
					TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				}
				else
				{
					Uart3ParsingState = 0;
					TimerUart3Parsing = 0;
				}
				break;
			case 2:	// SA(Source Addr)
				if(Rx3Data==PC_ADDR)
				{
					Uart3Cks += Rx3Data;
					Uart3ParsingState++;
					TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				}
				else
				{
					Uart3ParsingState = 0;
					TimerUart3Parsing = 0;
				}
				break;
			case 3:	// Length(LSB)
				Uart3DataLength = Rx3Data;
				Uart3Cks += Rx3Data;
				Uart3ParsingState++;
				TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				break;
			case 4:	// Length(MSB)
				Uart3DataLength |= (unsigned short)Rx3Data << 8;				
				Uart3Cks += Rx3Data;
				Uart3ParsingState++;
				TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				break;
			case 5:	// Cmd
				if(Rx3Data >= 0xA0 && Rx3Data <= 0xAF)
				{
					Uart3Cmd = Rx3Data;
					Uart3DataCnt = 0;
					Uart3Cks += Rx3Data;
					Uart3ParsingState++;
					TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				}
				else
				{
					Uart3ParsingState = 0;
					TimerUart3Parsing = 0;
				}
				break;
			case 6:	// Data
				Uart3DataBuf[Uart3DataCnt] = Rx3Data;
				Uart3Cks += Rx3Data;
				Uart3DataCnt++;
				if(Uart3DataCnt==Uart3DataLength)
				{
					Uart3DataCnt = 0;
					Uart3ParsingState++;
				}
				TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				break;
			case 7:	// CheckSum(LSB)
				if(Rx3Data==(Uart3Cks & 0x00FF))
				{
					Uart3ParsingState++;
					TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				}
				else
				{
					Uart3ParsingState = 0;
					TimerUart3Parsing = 0;
				}
				break;
			case 8:	// CheckSum(MSB)
				if(Rx3Data==(Uart3Cks >> 8))
				{
					Uart3ParsingState++;
					TimerUart3Parsing = UART3_PARSING_TIMEOUT;
				}
				else
				{
					Uart3ParsingState = 0;
					TimerUart3Parsing = 0;
				}
				break;
			case 9:	// End
				if(Rx3Data == 0x81)
				{
					unsigned short Crc16 = 0;
					unsigned int PageAddr;
					unsigned char Cnt = 0;
					unsigned int SourcePageAddr;

					switch(Uart3Cmd)
					{
						case PC_TO_BOARD_REQ_VER:
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_VER;
							*(unsigned short*)(Uart3TxBuf+6) = *(unsigned short*)(0x08007800+8);
							Uart3TxBuf[8] = 0;	// BootLoader Mode
							Send_BoardToPc_WithCks(Uart3TxBuf, 3);
							break;
						case PC_TO_BOARD_REQ_RESET:
							Eeprom_Write(EEPROM_IDX_RESET_REQ, 1);
							Eeprom_Write(EEPROM_IDX_JUMP_TO_APP_TIMEOUT, *(unsigned short*)Uart3DataBuf);
							HAL_NVIC_SystemReset();
							break;
						case PC_TO_BOARD_REQ_ERASE_PAGE:
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_ERASE_PAGE;
							if(Uart3DataBuf[0] > 13)	// BootLoader page is from 0 to 13
							{
								Refresh_IWDG();
								if(FLASH_If_Erase(0x08000000+2048*Uart3DataBuf[0], Uart3DataBuf[1])==FLASHIF_OK)
								{
									Uart3TxBuf[6] = 0;	// OK
								}
								else
								{
									Uart3TxBuf[6] = 1;	// Error
								}
							}
							else
							{
								Uart3TxBuf[6] = 2;	// Error
							}
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
							break;
						case PC_TO_BOARD_REQ_WRITE_PAGE:
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_WRITE_PAGE;
							if(Uart3DataLength==2051)
							{
								Crc16 = Calculation_Crc16(Uart3DataBuf+1, 2048);
								if(Crc16==*(unsigned short*)(Uart3DataBuf+1+2048))
								{
									PageAddr = 0x08000000+2048*Uart3DataBuf[0];
									Refresh_IWDG();
									if(FLASH_If_Write(PageAddr, (uint32_t*)(Uart3DataBuf+1), 2048/4)==FLASHIF_OK)
									{
										Crc16 = Calculation_Crc16((unsigned char*)PageAddr, 2048);
										if(Crc16==*(unsigned short*)(Uart3DataBuf+1+2048))
										{
											Uart3TxBuf[6] = 0;	// OK!
										}
										else
										{
											Uart3TxBuf[6] = 1;	// Crc16 Error of 2ritten data
										}
									}
									else
									{
										Uart3TxBuf[6] = 2;	// Write Error
									}
								}
								else
								{
									Uart3TxBuf[6] = 3;	// Crc16 Error of received data
								}
							}
							else
							{
								Uart3TxBuf[6] = 4;	// received data Error
							}
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_SET_EACH_CHANNEL_RELAY;
							Uart3TxBuf[6] = 0;
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
							break;
						case PC_TO_BOARD_REQ_START_UPDATE:
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_START_UPDATE;
							if(Uart3DataBuf[1] > 13)	// Page 0~13 is for BootLoader
							{
								PageAddr = 0x08000000+2048*Uart3DataBuf[0];	// Address of downloaded page
								Crc16 = Calculation_Crc16((unsigned char*)PageAddr, 2048*Uart3DataBuf[2]);
								if(Crc16==*(unsigned short*)(Uart3DataBuf+3))	// Checksum of all binary in update data
								{
									Refresh_IWDG();
									Eeprom_Write(EEPROM_IDX_UPDATE_RSLT, 1);	// Set update fail in default
									if(FLASH_If_Erase(0x08000000+2048*Uart3DataBuf[1], Uart3DataBuf[2])==FLASHIF_OK)
									{
										for(Cnt=0; Cnt<Uart3DataBuf[2]; Cnt++)
										{
											PageAddr = 0x08000000+2048*(Uart3DataBuf[1]+Cnt);	// Destination page(App)
											SourcePageAddr = 0x08000000+2048*(Uart3DataBuf[0]+Cnt);
											if(FLASH_If_Write(PageAddr, (unsigned int*)SourcePageAddr, 2048/4) != FLASHIF_OK)
											{
												Uart3TxBuf[6] = 1;	// Write Error
												break;
											}
										}
										if(Cnt==Uart3DataBuf[2])	// Write All
										{
											PageAddr = 0x08000000+2048*Uart3DataBuf[1];	// Address of updated page
											Crc16 = Calculation_Crc16((unsigned char*)PageAddr, 2048*Uart3DataBuf[2]);
											if(Crc16==*(unsigned short*)(Uart3DataBuf+3))
											{
												Eeprom_Write(EEPROM_IDX_UPDATE_RSLT, 0);	// Update Result is OK!
												Uart3TxBuf[6] = 0;	// OK!
											}
											else
											{
												Uart3TxBuf[6] = 2;	// Crc16 Error of updated data
											}
										}
									}
									else
									{
										Uart3TxBuf[6] = 3;	// Erase Error
									}
								}
								else
								{
									Uart3TxBuf[6] = 4;	// Crc16 Error of downloaded data
								}
							}
							else
							{
								Uart3TxBuf[6] = 5;	// Page Error!
							}
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
							break;
						case PC_TO_BOARD_REQ_JUMP_TO_APP:
							Eeprom_Write(EEPROM_IDX_JUMP_REQ, 1);
							JumpToAppMain();
							break;
					}
				}
				Uart3ParsingState = 0;
				TimerUart3Parsing = 0;
				break;
		}
		Uart3RxRdCnt++;
		if(Uart3RxRdCnt==UART3_RX_BUF_SIZE)
		{
			Uart3RxRdCnt = 0;
		}
	}
	if(TimerUart3Parsing==0)
	{
		if(Uart3ParsingState != 0)
		{
			Uart3ParsingState = 0;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
