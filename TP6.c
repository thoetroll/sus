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
#include <stdlib.h> //pour fonction atoi
#include <stdio.h> // pour fonction rewind
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VRAI 1
#define FAUX 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void RAZ_Ecran(void);
void Afficher_menu(void);
void choix_errone(void);
void quitter(void);
void PWM(int);
void choixPWM(int);

void RAZ_Chaine(char *, int);

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
	int SESSION = VRAI;
	char connexion_OK[] = "Connexion OK";
	char choix_utilisateur[2];
	char choix_utilisateur2[4];
	int choix_int;
	int choix2;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //VALIDATION BUFFER
    HAL_GPIO_WritePin(GPIOF, CMD_3E_Pin, GPIO_PIN_RESET);

    //lance PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    //test connexion UART
    if(HAL_UART_Transmit(&huart3, (uint8_t *)connexion_OK, sizeof(connexion_OK), 500) != HAL_OK)
    {

    }
    HAL_Delay(2000);
    RAZ_Ecran();
    Afficher_menu();
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while(SESSION==VRAI)
	  {
		  RAZ_Ecran();
		  Afficher_menu();
		  RAZ_Chaine(choix_utilisateur,sizeof(choix_utilisateur));
		  HAL_UART_Receive(&huart3, (uint8_t *)choix_utilisateur, sizeof(choix_utilisateur), HAL_MAX_DELAY);
		  rewind(stdin);

		  choix_int = atoi(choix_utilisateur);

		  switch(choix_int)
		  {
		  case 0 :
		  {
			  SESSION=FAUX;
			  quitter();
			  break;
		  }
		  case 1 :
		  {
			  PWM(0);
		 	  break;
		  }
		  case 2 :
		 		  {
		 			  PWM(25);
		 			  break;
		 		  }
		  case 3 :
		 		  {
		 			  PWM(75);
		 			  break;
		 		  }
		  case 4 :
		 		  {
		 			 HAL_UART_Receive(&huart3, (uint8_t *)choix_utilisateur2, sizeof(choix_utilisateur2), HAL_MAX_DELAY);
		 			  rewind(stdin);

		 			 choix2 = atoi(choix_utilisateur2);
		 			  PWM(choix2);
		 			  break;
		 		  }
		  default :
		 		  {
		 			  choix_errone();
		 			  break;
		 		  }
		  }
	  }
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
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

}

/* USER CODE BEGIN 4 */
void RAZ_Ecran(void)
{
	char Chaine_ESC = 27, * Pteur_ESC;
	char Chaine_RAZ[] = "[2J";
	char retour_gauche [] = "\r";

	Pteur_ESC = &Chaine_ESC;


	HAL_UART_Transmit(&huart3, (uint8_t *)Pteur_ESC, 1, 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)Chaine_RAZ, sizeof(Chaine_RAZ), 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)retour_gauche, sizeof(retour_gauche), 500);


}

void Afficher_menu(void)
{
	char choix_0[] = "0 : sortir";
	char choix_1[] = "1 : arret moteur";
	char choix_2[] = "2 : PWM 25%";
	char choix_3[] = "3 : PWM 75%";
	char choix_4[] = "4 : Choisir rapport cyclique ";
	char votre_choix[] = "choisir choix -> ";
	char Retour_ligne[] = "\r\n";

	HAL_UART_Transmit(&huart3, (uint8_t *)choix_0, sizeof(choix_0), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)choix_1, sizeof(choix_1), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)choix_2, sizeof(choix_2), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)choix_3, sizeof(choix_3), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)choix_4, sizeof(choix_4), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);

	HAL_UART_Transmit(&huart3, (uint8_t *)votre_choix, sizeof(votre_choix), 500);
}

void choix_errone(void)
{
	char erreur[] = "ERREURRRRR";
	char Retour_ligne[] = "\r\n";

	RAZ_Ecran();
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)erreur, sizeof(erreur), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);



}

void quitter(void)
{
	char bisous[]="Comment te recuperer ptn";
	char Retour_ligne[] = "\r\n";

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	RAZ_Ecran();

	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)bisous, sizeof(bisous), 500);
	HAL_UART_Transmit(&huart3, (uint8_t *)Retour_ligne, sizeof(Retour_ligne), 500);

}
void PWM(int CCR)
{
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,CCR);
}

void RAZ_Chaine(char * chaine, int Dim)
{
	int i;
	for(i=0;i< Dim;i++)
	{
		*(chaine + i) = '0';
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
