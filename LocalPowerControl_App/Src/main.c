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
#include "eeprom.h"
#pragma location=".Ver_Info"
const unsigned char VerInfo[8] = {0x7E, 0x81, 0x96, 0xA5, 0x64, 0x42, 0, 1};	// Parsing Flag + Version 0.1
#pragma location=".App_Start"
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
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned int TimerDelay;
unsigned int TimerShtRead;
unsigned int TimerRefreshIwdg;
unsigned int TimerInputScan;
unsigned int TimerMcuStatusLed;
unsigned int TimerUart3Parsing;
unsigned int TimerLedForChannel;
unsigned int Timer1ms = 1000;
unsigned int Timer10us;
unsigned char McuStatusLedState;
unsigned char Uart3TxBuf[UART3_TX_BUF_SIZE] = "<RI>";
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
unsigned short ShtStatusReg;
double TemperatureRaw;
unsigned short Temperature;
double HumidityRaw;
unsigned short Humidity;
unsigned char ShtCmd;
GPIO_TypeDef* LedPort[8] = 
{
	INPUT_DET_0_LED_GPIO_Port, 
	INPUT_DET_1_LED_GPIO_Port,
	INPUT_DET_2_LED_GPIO_Port,
	INPUT_DET_3_LED_GPIO_Port,
	INPUT_DET_4_LED_GPIO_Port,
	INPUT_DET_5_LED_GPIO_Port,
	INPUT_DET_6_LED_GPIO_Port,
	INPUT_DET_7_LED_GPIO_Port,				
};
unsigned short LedPin[8] =
{
	INPUT_DET_0_LED_Pin,
	INPUT_DET_1_LED_Pin,
	INPUT_DET_2_LED_Pin,
	INPUT_DET_3_LED_Pin,
	INPUT_DET_4_LED_Pin,
	INPUT_DET_5_LED_Pin,
	INPUT_DET_6_LED_Pin,
	INPUT_DET_7_LED_Pin,
};
unsigned char LedCnt=0;
unsigned char LedForChannel_OnOff=0;

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
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Eeprom_Check_Init(void);
unsigned short Eeprom_Read(unsigned short Addr);
unsigned short Eeprom_Write(unsigned short Addr, unsigned short Value);
void GpioModeChange(GPIO_TypeDef * Port, unsigned short Pin, unsigned char InOutMode);
void Delay10us(unsigned int DelayCnt);
void ReadStatusRegSht(void);
unsigned char SendCmdSht(unsigned char Cmd);
void ReadMeasureSht(void);
void Channel_Relay_Cont(unsigned char Channel, unsigned char OnOff);
void Led_Cont(unsigned char Led, GPIO_PinState OnOff);
void Refresh_IWDG(void);
void Input_Scan(void);
void MCU_Status_Led_Cont(void);
void LedForChannel_Display(void);
void Send_BoardToPc_WithCks(unsigned char* pBuf, unsigned short Length);
void BoardToPC_AnsJumpToApp(void);
void Uart3_Parsing(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(TimerDelay)
		{
			TimerDelay--;
		}
		if(TimerShtRead)
		{
			TimerShtRead--;
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
		if(TimerLedForChannel)
		{
			TimerLedForChannel--;
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
	if(htim->Instance == TIM3)
	{
		if(Timer10us)
		{
			Timer10us--;
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
	Eeprom_Check_Init();	// bySeo_20190712
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LAN_RESET_GPIO_Port, LAN_RESET_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart3, &Uart3RxData, 1);	
	ChannelRelayState = Eeprom_Read(EEPROM_IDX_CHANNEL_RELAY_STATUS);
	Channel_Relay_Cont(ChannelRelayState, RELAY_ON);
	Channel_Relay_Cont(~ChannelRelayState, RELAY_OFF);
	InputNow = INPUT_DET_0_GPIO_Port->IDR;
	InputOld = InputNow;
	Led_Cont(~InputOld & 0x000000FF, GPIO_PIN_RESET);	// Input Detect LED On
	Led_Cont(InputOld & 0x000000FF, GPIO_PIN_SET);	// Input Detect LED Off
	Led_Cont(InputOld & 0x00000300, GPIO_PIN_RESET);	// Aux In LED On
	Led_Cont(~InputOld & 0x00000300, GPIO_PIN_SET);	// Aux In LED Off
	MCU_Status_Led_Cont();
	TimerDelay = 100;
	while(TimerDelay);
	HAL_GPIO_WritePin(LAN_RESET_GPIO_Port, LAN_RESET_Pin, GPIO_PIN_RESET);
	ShtCmd=0x07;
	SendCmdSht(ShtCmd);	// Read Status Register
	ReadStatusRegSht();
	ShtCmd=0x03;	// Measure Temperature
	SendCmdSht(ShtCmd);
	TimerShtRead = 500;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	Refresh_IWDG();
	Input_Scan();
	MCU_Status_Led_Cont();
	LedForChannel_Display();
	ReadMeasureSht();
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart4.Init.BaudRate = 4800;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TEMP_HUMI_SCK_Pin|TEMP_HUMI_SDA_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : TEMP_HUMI_SCK_Pin TEMP_HUMI_SDA_Pin */
  GPIO_InitStruct.Pin = TEMP_HUMI_SCK_Pin|TEMP_HUMI_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

void Delay10us(unsigned int DelayCnt)
{
	if(DelayCnt)
	{
		Timer10us = DelayCnt;
		HAL_TIM_Base_Start_IT(&htim3);
		while(Timer10us);
		HAL_TIM_Base_Stop(&htim3);
	}
}

void GpioModeChange(GPIO_TypeDef * Port, unsigned short Pin, unsigned char InOutMode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = Pin;
	switch(InOutMode)
	{
		case 0:	// Input Mode			
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			break;
		case 1:	// Output Mode
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			break;
	}
	HAL_GPIO_Init(Port, &GPIO_InitStruct);
}

unsigned char SendCmdSht(unsigned char Cmd)
{
	unsigned char Cnt = 0;
	unsigned char Ret = 0;

	GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 1);
	SHT_DATA_HIGH;	
	Delay10us(1);
	SHT_SCK_HIGH;
	Delay10us(1);
	SHT_DATA_LOW;
	Delay10us(1);
	SHT_SCK_LOW;
	Delay10us(2);
	SHT_SCK_HIGH;
	Delay10us(1);
	SHT_DATA_HIGH;
	Delay10us(1);
	SHT_SCK_LOW;
	Delay10us(1);
	for(;Cnt<8; Cnt++)
	{
		if(Cmd & (0x80 >> Cnt))
		{
			SHT_DATA_HIGH;
		}
		else
		{
			SHT_DATA_LOW;
		}
		SHT_SCK_HIGH;
		Delay10us(1);		
		SHT_SCK_LOW;
		Delay10us(1);		
	}
	GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 0);
	SHT_SCK_HIGH;
	Delay10us(1);
	if(HAL_GPIO_ReadPin(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin) != GPIO_PIN_RESET)
	{
		Ret |= 0x01;	// NACK
	}
	SHT_SCK_LOW;
	Delay10us(1);
	switch(Cmd)
	{
		case 0x03:
		case 0x05:
			if(HAL_GPIO_ReadPin(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin) != GPIO_PIN_SET)
			{
				Ret |= 0x10;	// Error
			}
			break;
		case 0x07:
			break;
	}

	return Ret;
}

void ReadStatusRegSht(void)
{
	unsigned char DataCnt = 0;
	unsigned char BitCnt = 0;
	unsigned char ShtData[2] = {0,};

	GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 0);	// Input Mode
	for(;DataCnt<2; DataCnt++)
	{
		for(BitCnt=0; BitCnt<8; BitCnt++)
		{
			SHT_SCK_HIGH;
			Delay10us(1);
			if(HAL_GPIO_ReadPin(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin)==GPIO_PIN_SET)
			{
				ShtData[DataCnt] |= 0x80 >> BitCnt;
			}
			SHT_SCK_LOW;					
			Delay10us(1);
		}
		/* Send ACK to SHT */
		GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 1);	// Output Mode
		SHT_DATA_LOW;
		Delay10us(1);
		SHT_SCK_HIGH;
		Delay10us(1);
		SHT_SCK_LOW;
		GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 0);	// Input Mode
		Delay10us(1);
	}
	ShtStatusReg = ShtData[0];
}

void ReadMeasureSht(void)
{
	if(TimerShtRead==0)
	{
		GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 0);	// Input Mode
		if(HAL_GPIO_ReadPin(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin)==GPIO_PIN_RESET)
		{
			unsigned char DataCnt = 0;
			unsigned char BitCnt = 0;
			unsigned char ShtData[3] = {0,};
			unsigned short Data = 0;

			for(;DataCnt<3; DataCnt++)
			{
				for(BitCnt=0; BitCnt<8; BitCnt++)
				{
					SHT_SCK_HIGH;
					Delay10us(1);
					if(HAL_GPIO_ReadPin(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin)==GPIO_PIN_SET)
					{
						ShtData[DataCnt] |= 0x80 >> BitCnt;
					}
					SHT_SCK_LOW;					
					Delay10us(1);
				}
				/* Send ACK to SHT */
				GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 1);	// Output Mode
				SHT_DATA_LOW;
				Delay10us(1);
				SHT_SCK_HIGH;
				Delay10us(1);
				SHT_SCK_LOW;
				GpioModeChange(TEMP_HUMI_SDA_GPIO_Port, TEMP_HUMI_SDA_Pin, 0);	// Input Mode
				Delay10us(1);
			}
			Data = ((unsigned short)ShtData[0] << 8) + (unsigned short)ShtData[1];
			switch(ShtCmd)
			{
				case 0x03:	// Measure Temperature
					TemperatureRaw = 0.01*Data - 39.7;
					Temperature = (unsigned short)TemperatureRaw;
					ShtCmd = 0x05;
					break;
				case 0x05:	// Measure Relative Humidity
					HumidityRaw = 0.0367*Data + (-1.5955*Data*Data)/1000000 - 2.0468;
					Humidity = (unsigned short)HumidityRaw;
					ShtCmd = 0x03;
					break;
			}
			SendCmdSht(ShtCmd);	// Measure Relative Humidity
		}
		TimerShtRead = 500;
	}
}

void Channel_Relay_Cont(unsigned char Channel, unsigned char OnOff)
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

void Input_Scan(void)
{
	if(TimerInputScan==0)
	{
		switch(InputScanState)
		{
			case 0:
				InputNow = (unsigned char)INPUT_DET_0_GPIO_Port->IDR;
				if(InputOld != InputNow)
				{
					InputScanState = 1;
				}
				break;
			case 1:
				InputNow = INPUT_DET_0_GPIO_Port->IDR;
				if(InputOld != InputNow)
				{
					InputOld = InputNow;
					Led_Cont(~InputOld & 0x000000FF, GPIO_PIN_RESET);	// Input Detect LED On
					Led_Cont(InputOld & 0x000000FF, GPIO_PIN_SET);	// Input Detect LED Off
					Led_Cont(InputOld & 0x00000300, GPIO_PIN_RESET);	// Aux In LED On
					Led_Cont(~InputOld & 0x00000300, GPIO_PIN_SET);	// Aux In LED Off
				}
				InputScanState = 0;
				break;
		}
		TimerInputScan = 10;
	}
}

void MCU_Status_Led_Cont(void)
{
	if(TimerMcuStatusLed==0)
	{
		if(McuStatusLedState==0)
		{
			HAL_GPIO_WritePin(MCU_STATUS_GPIO_Port, MCU_STATUS_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EXT_IN_1_LED_GPIO_Port, EXT_IN_1_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EXT_IN_2_LED_GPIO_Port, EXT_IN_2_LED_Pin, GPIO_PIN_RESET);
			McuStatusLedState = 1;
		}
		else
		{
			HAL_GPIO_WritePin(MCU_STATUS_GPIO_Port, MCU_STATUS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EXT_IN_1_LED_GPIO_Port, EXT_IN_1_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EXT_IN_2_LED_GPIO_Port, EXT_IN_2_LED_Pin, GPIO_PIN_SET);
			McuStatusLedState = 0;			
		}		
		TimerMcuStatusLed = 125;
	}
}

void LedForChannel_Display(void)
{
	if(TimerLedForChannel==0)
	{
		switch(LedForChannel_OnOff)
		{
			case 0:	// Led Off
				HAL_GPIO_WritePin(LedPort[LedCnt], LedPin[LedCnt], GPIO_PIN_RESET);
				break;
			case 1:	// Led On
				HAL_GPIO_WritePin(LedPort[LedCnt], LedPin[LedCnt], GPIO_PIN_SET);
				break;
		}
		LedCnt++;
		if(LedCnt==8)
		{
			LedCnt = 0;
			switch(LedForChannel_OnOff)
			{
				case 0:
					LedForChannel_OnOff = 1;
					break;
				case 1:
					LedForChannel_OnOff = 0;
					break;
			}
			TimerLedForChannel = 500;
		}
		else
		{
			TimerLedForChannel = 50;
		}
	}
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

void BoardToPC_AnsJumpToApp(void)
{
	Uart3TxBuf[5] = BOARD_TO_PC_ANS_JUMP_TO_APP;
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
					unsigned char Extractor = 0;

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
							break;
						case PC_TO_BOARD_REQ_WRITE_PAGE:							
							break;
						case PC_TO_BOARD_REQ_START_UPDATE:
							break;
						case PC_TO_BOARD_REQ_JUMP_TO_APP:
							break;
						case PC_TO_BOARD_REQ_GET_CHANNEL_POWER:
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_GET_CHANNEL_POWER;
							Uart3TxBuf[6] = InputOld;
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
							break;
						case PC_TO_BOARD_REQ_GET_CHANNEL_RELAY:
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_GET_CHANNEL_RELAY;
							Uart3TxBuf[6] = ChannelRelayState;
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
							break;
						case PC_TO_BOARD_REQ_SET_EACH_CHANNEL:
							if(Uart3DataBuf[0] < 8)	// 0~7
							{
								Extractor = 0x01 << Uart3DataBuf[0];
								if(Uart3DataBuf[1])	// Power On
								{
									ChannelRelayState |= Extractor;
									Channel_Relay_Cont(Extractor, RELAY_ON);
								}
								else
								{
									ChannelRelayState &= ~Extractor;
									Channel_Relay_Cont(Extractor, RELAY_OFF);
								}
							}
							if(ChannelRelayState != Eeprom_Read(EEPROM_IDX_CHANNEL_RELAY_STATUS))
							{
								Eeprom_Write(EEPROM_IDX_CHANNEL_RELAY_STATUS, ChannelRelayState);
							}
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_SET_EACH_CHANNEL_RELAY;
							Uart3TxBuf[6] = 0;
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
							break;
						case PC_TO_BOARD_REQ_SET_ALL_CHANNEL:
							if(ChannelRelayState != Uart3DataBuf[0])
							{
								ChannelRelayState = Uart3DataBuf[0];
								Eeprom_Write(EEPROM_IDX_CHANNEL_RELAY_STATUS, ChannelRelayState);
								Channel_Relay_Cont(ChannelRelayState, RELAY_ON);
								Channel_Relay_Cont(~ChannelRelayState, RELAY_OFF);
							}
							Uart3TxBuf[5] = BOARD_TO_PC_ANS_SET_ALL_CHANNEL_RELAY;
							Uart3TxBuf[6] = 0;
							Send_BoardToPc_WithCks(Uart3TxBuf, 1);
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
