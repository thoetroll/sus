/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "ICM20602.h"

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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef ret;
	static uint8_t buf_SPI_TX[2];
	static uint8_t buf_SPI_RX[2];
	static uint8_t buf_UART[50];
    static char message[50];
	static uint8_t buf_Data_ATG[15];
	static uint16_t Data_ATG[7];
	static float Accel_X;
	static float Accel_Y;
	static float Accel_Z;
	static float Temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void ICM20602_WR_Cmd(void);
static void ICM20602_Data_ATG(void);
static void ICM20602_Init(void);
static void Format_PuTTy(void);

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
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart3.Init.BaudRate = 38400;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  HAL_GPIO_WritePin(GPIOE, LED_R0_Pin|LED_R1_Pin|LED_R2_Pin|LED_R3_Pin 
                          |LED_R4_Pin|LED_R5_Pin|LED_R6_Pin|LED_R7_Pin 
                          |LED_R8_Pin|LED_R9_Pin|LED_R10_Pin|LED_R11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CMD_3E_GPIO_Port, CMD_3E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_E_Pin|LCD_DB4_Pin|LCD_DB5_Pin|LCD_DB6_Pin 
                          |LCD_DB7_Pin|LCD_RS_Pin|LCD_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS2_SPI_GPIO_Port, CS2_SPI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_R0_Pin LED_R1_Pin LED_R2_Pin LED_R3_Pin 
                           LED_R4_Pin LED_R5_Pin LED_R6_Pin LED_R7_Pin 
                           LED_R8_Pin LED_R9_Pin LED_R10_Pin LED_R11_Pin */
  GPIO_InitStruct.Pin = LED_R0_Pin|LED_R1_Pin|LED_R2_Pin|LED_R3_Pin 
                          |LED_R4_Pin|LED_R5_Pin|LED_R6_Pin|LED_R7_Pin 
                          |LED_R8_Pin|LED_R9_Pin|LED_R10_Pin|LED_R11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CMD_3E_Pin */
  GPIO_InitStruct.Pin = CMD_3E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CMD_3E_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INTR_SPI_Pin */
  GPIO_InitStruct.Pin = INTR_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INTR_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_0_Pin BP_1_Pin */
  GPIO_InitStruct.Pin = BP_0_Pin|BP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTER_0_Pin INTER_1_Pin INTER_2_Pin INTER_3_Pin 
                           USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = INTER_0_Pin|INTER_1_Pin|INTER_2_Pin|INTER_3_Pin 
                          |USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_DB4_Pin LCD_DB5_Pin LCD_DB6_Pin 
                           LCD_DB7_Pin LCD_RS_Pin LCD_RW_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_DB4_Pin|LCD_DB5_Pin|LCD_DB6_Pin 
                          |LCD_DB7_Pin|LCD_RS_Pin|LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS2_SPI_Pin */
  GPIO_InitStruct.Pin = CS2_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS2_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin MISO_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|MISO_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
static void ICM20602_WR_Cmd(void)
{
	HAL_GPIO_WritePIN(CS2_SPI_GPIO_Port,CS2_SPI_Pin,GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive( &hspi3, buf_SPI_TX, buf_SPI_RX, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePIN(CS2_SPI_GPIO_Port,CS2_SPI_Pin,GPIO_PIN_SET);


	if(ret != HAL_OK)
		{
			sprintf((char*)buf_UART,"Error Tx\r\n");
		}
	else
		{
			sprintf((char*)buf_UART,message,buf_SPI_RX[1]);
		}
	HAL_UART_Transmit(&huart3, buf_UART, strlen((char*)buf_UART), HAL_MAX_DELAY);
}

static void ICM20602_Data_ATG(void)
{
	HAL_GPIO_WritePIN(CS2_SPI_GPIO_Port,CS2_SPI_Pin,GPIO_PIN_RESET);
	ret = HAL_SPI_TransmitReceive( &hspi3, buf_SPI_TX, buf_Data_ATG, 15, HAL_MAX_DELAY);
	HAL_GPIO_WritePIN(CS2_SPI_GPIO_Port,CS2_SPI_Pin,GPIO_PIN_SET);

		if(ret != HAL_OK)
			{
				sprintf((char*)buf_UART,"Error Tx\r\n");
			}
		else
			{
				sprintf((char*)buf_UART,"RECUP DATA ATG\r\n");
			}
		HAL_UART_Transmit(&huart3, buf_UART, strlen((char*)buf_UART), HAL_MAX_DELAY);
}

static void ICM20602_Init(void)
{
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_Delay(2);
	buf_SPI_TX[0] = Adr_I2C_IF;
	buf_SPI_TX[1] = Cmd_I2C_IF_SPI;
	memset(message,'\0',sizeof(message));
	strcpy(message,"MODE SPI\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_WHO_AM_I|SPI_READ_Flag;
	buf_SPI_TX[1] = 0x00;
	memset(message,'\0',sizeof(message));
	strcpy(message,"WHO I AM = %x\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_PWR_MGMT_1;
	buf_SPI_TX[1] = Cmd_PWR_MGMT_1_CD;
	memset(message,'\0',sizeof(message));
	strcpy(message,"DEMARAGE HORLOGEEEEEE\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_PWR_MGMT_2;
	buf_SPI_TX[1] = Cmd_PWR_MGMT_2_AA;
	memset(message,'\0',sizeof(message));
	strcpy(message,"ACTIVATION FUSEEE\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_SMPLRT_DIV;
	buf_SPI_TX[1] = Cmd_SMPLRT_DIV;
	memset(message,'\0',sizeof(message));
	strcpy(message,"PREDIV FAIT = &x\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_ACCEL_CONFIG;
	buf_SPI_TX[1] = Cmd_ACCEL_CONFIG;
	memset(message,'\0',sizeof(message));
	strcpy(message,"CONFIG\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_ACCEL_CONFIG2;
	buf_SPI_TX[1] = Cmd_ACCEL_CONFIG2;
	memset(message,'\0',sizeof(message));
	strcpy(message,"CONFIG2\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_INT_PIN_CFG;
	buf_SPI_TX[1] = Cmd_INT_PIN_CFG;
	memset(message,'\0',sizeof(message));
	strcpy(message,"CONFIG PIN\r\n");
	ICM20602_WR_Cmd();

	buf_SPI_TX[0] = Adr_INT_ENABLE;
	buf_SPI_TX[1] = Cmd_INT_ENABLE_EID;
	memset(message,'\0',sizeof(message));
	strcpy(message,"ACTIV INT E\r\n");
	ICM20602_WR_Cmd();

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}


HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int i;
	if(GPIO_Pin == INTR_SPI_Pin)
	{
		buf_SPI_TX[0] = Adr_Data_ATG|SPI_READ_Flag;
		buf_SPI_TX[1] = 0x00;
		ICM20602_Data_ATG();

		for(i=0;i<=3;i++)
		{
			Data_ATG[i] = buf_Data_ATG[1+2*i]<<8|buf_Data_ATG[2+2*i];
		}
		Accel_X = ((float) (Data_ATG[0])/16384.0);
		Accel_Y = ((float) (Data_ATG[1])/16384.0);
		Accel_Z = ((float) (Data_ATG[2])/16384.0);
		Temp = ((float) (Data_ATG[3])/326.8+25.0);
	}
}

static void Format_PuTTy(void)
{
	  float val;
	  uint8_t buf[70];

	  if(Accel_X < 0)
	  {
	    Accel_X_s = '-';
		val = -Accel_X;
	  }
	  else
	  {
	    Accel_X_s = '+';
		val = Accel_X;
	  }
	  Accel_X_pe = (int)val;
	  Accel_X_pd = (int)((val - (float)Accel_X_pe)*1000);
	  if(Accel_Y < 0)
	  {
	    Accel_Y_s = '-';
		val = -Accel_Y;
	  }
	  else
	  {
	    Accel_Y_s = '+';
		val = Accel_Y;
	  }
	  Accel_Y_pe = (int)val;
	  Accel_Y_pd = (int)((val - (float)Accel_Y_pe)*1000);
	  if(Accel_Z < 0)
	  {
	    Accel_Z_s = '-';
	    val = -Accel_Z;
	  }
	  else
	  {
	    Accel_Z_s = '+';
		val = Accel_Z;
	  }
	  Accel_Z_pe = (int)val;
	  Accel_Z_pd = (int)((val - (float)Accel_Z_pe)*1000);
	  Temp_pe = (int)Temp;
	  Temp_pd = (int)((Temp - (float)Temp_pe)*100);
	  sprintf((char*)buf,"Accel_X  = %c%d.%3d\tAccel_Y  = %c%d.%3d\tAccel_Z  = %c%d.%3d\tTemp  = %u.%u\r\n"
				  ,Accel_X_s,Accel_X_pe,Accel_X_pd,Accel_Y_s,Accel_Y_pe,Accel_Y_pd,Accel_Z_s,Accel_Z_pe,Accel_Z_pd,Temp_pe,Temp_pd);
	  HAL_UART_Transmit(&huart3, buf, strlen((char*)buf),HAL_MAX_DELAY);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
