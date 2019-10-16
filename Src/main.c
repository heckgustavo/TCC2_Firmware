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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L3GD20.h"
#include "stdio.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t i;
uint8_t ctrl_reg_add[5] = {0x20, 0x21, 0x22, 0x23, 0x24};
uint8_t ctrl_reg_val[5] = {0xFF, 0x00, 0x8, 0x00, 0x00};

float sensitivity_250 = 114.285f;
float ydata[3]={0,0,0};
float zdata[3]={0,0,0};
uint16_t pwmY;
uint16_t pwmZ;
uint16_t motorY[2]={0,0};
uint16_t motorZ[2]={0,0};


float yhat[3];
float zhat[3];
float yhatant[3];
float zhatant[3];
float uy;
float uz;
float yy[2];
float yz[2];
float ALCyant[3];
float ALCzant[3];
float Byuy[3];
float Bzuz[3];
float Lyyy[3];
float Lzyz[3];

//COISAS PARA ALTERAR:

//sistema eixo Y
float Ay[3][3] = {{0, 1, 0}, {20.0515, 0, 0.1348}, {-20.0515, 0, 5.5058}};
float By[3]={0,-2.2887, 93.4769}; //Matriz B (eixo Y)
float Cy[2][3]={{1, 0, 0},{0, 1, 0}};
float ALCy[3][3];

//sistema eixo Z
float Az[3][3] = {{0, 1, 0}, {20.0488, 0, 0.1348}, {-20.0488, 0, 5.4894}};
float Bz[3]={0,-2.2884,93.1993};
float Cz[2][3]={{1, 0, 0},{0, 1, 0}};
float ALCz[3][3];

//Ganhos [ORIGINAL]
//float Ly[3][2]={{1.9995,0.9490},{18.5689,10.5063},{-102.7477,459.3545}};
//float Lz[3][2]={{1.9995,0.9489},{18.5690,10.4899},{-102.4052,459.4749}};
//float Ky[3] = {65.9847, 11.6463, 0.9046};
//float Kz[3] = {66.2867, 11.7091, 0.9086};


float Ky[3] = {65.9847, 11.6463, 0.90461};
float Kz[3] = {66.2867, 11.7091, 0.90864};
float Lz[3][2]={{16.37,24.2094},{91.02,513.11},{8376,61540.3}};
float Ly[3][2]={{16.37,24.2094},{91.02,513.11},{8376,61540.3}};


float dt = 0.00125;
float movmeanY[100];
float movmeanZ[100];
float meanY;
float meanZ;

int cont = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void set_pwmY(float saida);
void set_pwmZ(float saida);
void set_dirY(float saida);
void set_dirZ(float saida);
void get_ALC();
void get_movmeanY(float value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(htim->Instance==TIM2){
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
	  ydata[1]=(int16_t)L3GD20_GetAngularRateY(sensitivity_250);
	  zdata[1]=(int16_t)L3GD20_GetAngularRateZ(sensitivity_250);
	  ydata[1]=ydata[1]*3.14158/180;
	  zdata[1]=zdata[1]*3.14158/180;
	  ydata[0]+=ydata[1]*dt;
	  zdata[0]+=zdata[1]*dt;

	  //Observador:
	  yy[0]=ydata[0];
	  yy[1]=ydata[1];

	  yz[0]=zdata[0];
	  yz[1]=zdata[1];


	  //(A-LC)*yhatant
	  //(A-LC)*zhatant

	  ALCyant[0] = ALCy[0][0]*yhatant[0]+ALCy[0][1]*yhatant[1]+ALCy[0][2]*yhatant[2];
	  ALCyant[1] = ALCy[1][0]*yhatant[0]+ALCy[1][1]*yhatant[1]+ALCy[1][2]*yhatant[2];
	  ALCyant[2] = ALCy[2][0]*yhatant[0]+ALCy[2][1]*yhatant[1]+ALCy[2][2]*yhatant[2];

	  ALCzant[0] = ALCz[0][0]*zhatant[0]+ALCz[0][1]*zhatant[1]+ALCz[0][2]*zhatant[2];
	  ALCzant[1] = ALCz[1][0]*zhatant[0]+ALCz[1][1]*zhatant[1]+ALCz[1][2]*zhatant[2];
	  ALCzant[2] = ALCz[2][0]*zhatant[0]+ALCz[2][1]*zhatant[1]+ALCz[2][2]*zhatant[2];

	  //By*uy;
	  //Bz*uz;
	  Byuy[0] =By[0]*uy;
	  Byuy[1] =By[1]*uy;
	  Byuy[2] =By[2]*uy;

	  Bzuz[0] =Bz[0]*uz;
	  Bzuz[1] =Bz[1]*uz;
	  Bzuz[2] =Bz[2]*uz;

	  //Ly*yy
	  //Lz*yz
	  Lyyy[0]=Ly[0][0]*yy[0]+Ly[0][1]*yy[1];
	  Lyyy[1]=Ly[1][0]*yy[0]+Ly[1][1]*yy[1];
	  Lyyy[2]=Ly[2][0]*yy[0]+Ly[2][1]*yy[1];

	  Lzyz[0]=Lz[0][0]*yz[0]+Lz[0][1]*yz[1];
	  Lzyz[1]=Lz[1][0]*yz[0]+Lz[1][1]*yz[1];
	  Lzyz[2]=Lz[2][0]*yz[0]+Lz[2][1]*yz[1];

	  //yhat = ((A-LC)*yhatant + By*uy + Ly*yy)*dt
	  //zhat = ((A-LC)*zhatant + Bz*uz + Lz*yz)*dt
	  yhat[0]+=(ALCyant[0]+Byuy[0]+Lyyy[0])*dt;
	  yhat[1]+=(ALCyant[1]+Byuy[1]+Lyyy[1])*dt;
	  yhat[2]+=(ALCyant[2]+Byuy[2]+Lyyy[2])*dt;

	  zhat[0]+=(ALCzant[0]+Bzuz[0]+Lzyz[0])*dt;
	  zhat[1]+=(ALCzant[1]+Bzuz[1]+Lzyz[1])*dt;
	  zhat[2]+=(ALCzant[2]+Bzuz[2]+Lzyz[2])*dt;

	  //Salva na amostra anterior
	  yhatant[0]=yhat[0];
	  yhatant[1]=yhat[1];
	  yhatant[2]=yhat[2];

	  zhatant[0]=zhat[0];
	  zhatant[1]=zhat[1];
	  zhatant[2]=zhat[2];


	  //Gera a saida


	  uz = -(zhat[0]*Kz[0]+zhat[1]*Kz[1]+zhat[2]*Kz[2]);
	  uy = -(yhat[0]*Ky[0]+yhat[1]*Ky[1]+yhat[2]*Ky[2]);

	  if(uy<5 & uy>-5)
	  {
		  meanY=0;
	  }else
	  {
		  meanY=uy;
	  }

	  set_pwmY(uy);
	  set_pwmZ(uz);
	  set_dirY(uy);
	  set_dirZ(uz);
	 /* if(cont>=100)
	  {

		  cont=0;
	  }else
	  {
		  cont++;
	  }*/
  }
}

void set_dirY(float saida)
{
	if(saida>0)
	{
		motorY[0]=0;
		motorY[1]=1;
	}
	else
	{
		motorY[0]=1;
		motorY[1]=0;
	}
	HAL_GPIO_WritePin(dirYa_GPIO_Port, dirYa_Pin, motorY[0]);
	HAL_GPIO_WritePin(dirYb_GPIO_Port, dirYb_Pin, motorY[1]);
}

void set_dirZ(float saida)
{
	if(saida>0)
	{
		motorZ[0]=1;
		motorZ[1]=0;
	}
	else
	{
		motorZ[0]=0;
		motorZ[1]=1;
	}
	HAL_GPIO_WritePin(dirZa_GPIO_Port, dirZa_Pin, motorZ[0]);
	HAL_GPIO_WritePin(dirZb_GPIO_Port, dirZb_Pin, motorZ[1]);
}

void get_ALC()
{
	ALCy[0][0]=Ay[0][0] - (Ly[0][0]*Cy[0][0]+Ly[0][1]*Cy[1][0]);
	ALCy[1][0]=Ay[1][0] - (Ly[1][0]*Cy[0][0]+Ly[1][1]*Cy[1][0]);
	ALCy[2][0]=Ay[2][0] - (Ly[2][0]*Cy[0][0]+Ly[2][1]*Cy[1][0]);
	ALCy[0][1]=Ay[0][1] - (Ly[0][0]*Cy[0][1]+Ly[0][1]*Cy[1][1]);
	ALCy[1][1]=Ay[1][1] - (Ly[1][0]*Cy[0][1]+Ly[1][1]*Cy[1][1]);
	ALCy[2][1]=Ay[2][1] - (Ly[2][0]*Cy[0][1]+Ly[2][1]*Cy[1][1]);
	ALCy[0][2]=Ay[0][2] - (Ly[0][0]*Cy[0][2]+Ly[0][1]*Cy[1][2]);
	ALCy[1][2]=Ay[1][2] - (Ly[1][0]*Cy[0][2]+Ly[1][1]*Cy[1][2]);
	ALCy[2][2]=Ay[2][2] - (Ly[2][0]*Cy[0][2]+Ly[2][1]*Cy[1][2]);


	ALCz[0][0]=Az[0][0] - (Lz[0][0]*Cz[0][0]+Lz[0][1]*Cz[1][0]);
	ALCz[1][0]=Az[1][0] - (Lz[1][0]*Cz[0][0]+Lz[1][1]*Cz[1][0]);
	ALCz[2][0]=Az[2][0] - (Lz[2][0]*Cz[0][0]+Lz[2][1]*Cz[1][0]);
	ALCz[0][1]=Az[0][1] - (Lz[0][0]*Cz[0][1]+Lz[0][1]*Cz[1][1]);
	ALCz[1][1]=Az[1][1] - (Lz[1][0]*Cz[0][1]+Lz[1][1]*Cz[1][1]);
	ALCz[2][1]=Az[2][1] - (Lz[2][0]*Cz[0][1]+Lz[2][1]*Cz[1][1]);
	ALCz[0][2]=Az[0][2] - (Lz[0][0]*Cz[0][2]+Lz[0][1]*Cz[1][2]);
	ALCz[1][2]=Az[1][2] - (Lz[1][0]*Cz[0][2]+Lz[1][1]*Cz[1][2]);
	ALCz[2][2]=Az[2][2] - (Lz[2][0]*Cz[0][2]+Lz[2][1]*Cz[1][2]);
}

void set_pwmY(float saida)
{
	saida=saida*8000;
	saida = (uint16_t) saida;
	if(saida>1919)
	{
		pwmY = 1919;
	}
	else
	{
		pwmY=saida;
	}
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwmY);
}

void set_pwmZ(float saida)
{
	saida=saida*8000;
	saida = (uint16_t) saida;

	if(saida>1919)
	{
		pwmZ = 1919;
	}
	else
	{
		pwmZ=saida;
	}
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwmZ);

}

void get_movmeanY(float value)
{
	for (int k = 1; k < 100; ++k) {
		movmeanY[k] = movmeanY[k-1];
	}
	movmeanY[0] = value;
	float sum = 0;
	for (int x = 0; x < 100; ++x) {
		sum = sum + movmeanY[x];
	}
	meanY = sum/100;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  L3GD20_SetSPI(hspi1);
      for (i = 0; i < 5; ++i)
      {
    	L3GD20_Write(ctrl_reg_val[i], ctrl_reg_add[i]);
      }
     get_ALC();
     HAL_TIM_Base_Start_IT(&htim2);
     HAL_TIM_PWM_Init(&htim3);
     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  htim2.Init.Prescaler = 96;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1236;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1919;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|dirZa_Pin|dirZb_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dirYa_Pin|dirYb_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 dirZa_Pin dirZb_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|dirZa_Pin|dirZb_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : dirYa_Pin dirYb_Pin */
  GPIO_InitStruct.Pin = dirYa_Pin|dirYb_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
