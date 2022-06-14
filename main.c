/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
	*                                                                            *
  *          **              BALCI ELEKTRONIK                 **               *
  *          **     iletisim:hacolladanmert48@gmail.com       **               *
  *                                                                            *
  ******************************************************************************
	*  Potansiyometre PA6                    *
	*  Encoder PA8,PA9                       *   
	*  OLED scl->PB6, sda->PB7               *
	*  L298n in1->PC2, in2->PC3, ena->PA15   * 
	*  A4988 PC1                             *
 	*  Servo PB0                             *
	 ****************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"             //Genel kutuphane
#include "string.h"            //Genel kutuphane
#include "stm32f4xx_hal_def.h" //Genel kutuphane
#include "stm32f4xx_ll_adc.h"  //Genel kutuphane
#include "ssd1306.h"           //OLED kutuphane
#include "i2c-lcd.h"           //OLED kutuphane
#include "fonts.h"             //OLED font kutuphane




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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t ort,ort2,pwmd,pwm1,step1,k,adimm=0,say=0,tur=0,selamlama=0,potkararlilik=0;
uint32_t adc1,adc2,motorhizi,motorhizort;
int16_t adim_sayisi=0;
char buf[4];
char buf2[4];
char karakter_dizisi[16];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void selamlama_durumu(unsigned int);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);   //Modul baslatilir
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);        //Modul baslatilir
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);        //Modul baslatilir
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);        //Modul baslatilir
	
	
	SSD1306_Init ();            //OLED ayarlarini cagirir
  lcd_init ();                //OLED ayarlarini cagirir
	
	//Motorlari baslangicta test eder
  selamlama_durumu(1);        //1-aktif,0-pasif



  SSD1306_Fill (0);
	SSD1306_UpdateScreen(); //display

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
 {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
HAL_GPIO_TogglePin  (GPIOA,  GPIO_PIN_5);   // LED'i aç/kapa 
HAL_Delay  (10 ); 


		//ADC Ve Encoder veri okuma 
	adim_sayisi= __HAL_TIM_GET_COUNTER(&htim1);
	adimm=TIM_COUNTERMODE_CENTERALIGNED1;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
		adc1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		adc2 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop (&hadc1);
		
	//DC Motor kontrol bolumu
	 //********************************************************************************************
	 
	//Potansiyometrenin kararli calismasi icin 5 kez veri okunup ortalamasi alinir
	    motorhizi=adc2/20;	
			motorhizort+=motorhizi;
			ort2++;
			potkararlilik=5; //kararlilik degeri (yukseltildikce okuma hizi duser)(onerilen=5)
			
			while(ort2==potkararlilik)
{
			    	ort2=0;
			     	motorhizort/=potkararlilik;
  if((95<=motorhizort)&&(105>=motorhizort))
				{
					  TIM2->CCR1=0;
	        	motorhizort=100;
				   	SSD1306_GotoXY (70,30);
	          sprintf(buf, " %%%d     ", motorhizort-100);
           	SSD1306_Puts (buf, &Font_7x10, 1);
			     	SSD1306_UpdateScreen(); 
					  GPIOC->ODR &=0x00;
				} 
  if((105<motorhizort)&&(200>motorhizort))
			{
				    GPIOC->ODR &=0x00;
				    GPIOC->ODR |=0x08;
			    	TIM2->CCR1=(motorhizort-100)*10;
			    	SSD1306_GotoXY (70,30);
	          sprintf(buf, "CW %%%d ", motorhizort-100);
	          SSD1306_Puts (buf, &Font_7x10, 1);
		     		SSD1306_UpdateScreen(); 
			}

		      	
			
  if((1<=motorhizort)&&(95>motorhizort))
			{     GPIOC->ODR &=0x00;
				    GPIOC->ODR |=0x04;
				    TIM2->CCR1=(100-motorhizort)*10;
				    SSD1306_GotoXY (70,30);
	          sprintf(buf, "CCW %%%d ", 100-motorhizort);
	          SSD1306_Puts (buf, &Font_7x10, 1);
			     	SSD1306_UpdateScreen(); 
			}
			    	motorhizort=0;		
}
	
	//***********************************************************************************************	

//Servo kontrol bolumu
	
if(adim_sayisi>90)
	{
   adim_sayisi=90;
	}		
		//SSD1306_Fill (0);
	SSD1306_GotoXY (3,15);
	SSD1306_Puts ("Servo Derece=", &Font_7x10, 1);
	SSD1306_GotoXY (100,15);
	sprintf(buf, "%d ", adim_sayisi*2);
	SSD1306_Puts (buf, &Font_7x10, 1);	
 
	TIM3->CCR3=adim_sayisi+24;
	
	
	SSD1306_GotoXY (3,30);
	SSD1306_Puts ("DC Motor=", &Font_7x10, 1);
	
	HAL_Delay(1);
	SSD1306_UpdateScreen(); //display
	
	

	

 }
	
}


  /* USER CODE END 3 */
void selamlama_durumu(unsigned int selamlama)
{
	 if(selamlama==1)
	 {
		 SSD1306_Fill (0);
	SSD1306_GotoXY (10,20);
	SSD1306_Puts ("DC Motor Testi", &Font_7x10, 1);
		 SSD1306_GotoXY (50,40);
	SSD1306_Puts ("Test ", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
		 
 GPIOC->ODR |=0x08;
for(pwmd=0;pwmd<500;pwmd+=50)		
{
TIM2->CCR1=pwmd;
HAL_Delay  (100);
}	

 HAL_Delay (100);

 SSD1306_GotoXY (50,40);
	SSD1306_Puts ("Test .", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display

 for(pwmd=500;pwmd>0;pwmd-=50)		
{
TIM2->CCR1=pwmd;
HAL_Delay  (100);
}


 GPIOC->ODR &=0x00;
 HAL_Delay  (10);
 GPIOC->ODR |=0x04;

 SSD1306_GotoXY (50,40);
	SSD1306_Puts ("Test ..", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display 

for(pwmd=0;pwmd<500;pwmd+=50)		
{
TIM2->CCR1=pwmd;
HAL_Delay  (100);
}	

 HAL_Delay  (2000);

 for(pwmd=500;pwmd>0;pwmd-=50)		
{
TIM2->CCR1=pwmd;
HAL_Delay  (100);
}
 GPIOC->ODR &=0x00;

 SSD1306_GotoXY (50,40);
	SSD1306_Puts ("BITTI ", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
HAL_Delay  (1000);
 SSD1306_Fill (0);
	SSD1306_GotoXY (10,20);
	SSD1306_Puts ("Step Motor Testi", &Font_7x10, 1);
		 SSD1306_GotoXY (50,40);
	SSD1306_Puts ("Test ", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display

while(tur!=8)
{
 for(step1=0;step1<50; step1++)
  {
    GPIOC->ODR |=0x02;
    HAL_Delay  (3);
    GPIOC->ODR &=0x00;
    HAL_Delay  (3);
  }
 HAL_Delay  (1000);
	tur++;
}
SSD1306_GotoXY (50,40);
	SSD1306_Puts ("BITTI ", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
HAL_Delay  (1000);
 SSD1306_Fill (0);
	SSD1306_GotoXY (10,20);
	SSD1306_Puts ("Servo Motor Testi", &Font_7x10, 1);
		 SSD1306_GotoXY (50,40);
	SSD1306_Puts ("Test ", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display

TIM3->CCR3=0;
HAL_Delay(1000);
TIM3->CCR3=30;
HAL_Delay(1000);
TIM3->CCR3=0;
SSD1306_GotoXY (50,40);
	SSD1306_Puts ("BITTI ", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
HAL_Delay(1000);
    }
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 69;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1679;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

