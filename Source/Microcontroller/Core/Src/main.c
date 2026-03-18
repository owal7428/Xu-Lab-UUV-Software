/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "task_defs.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* external definitions of task queues so they can be initialized here first before use */
extern QueueHandle_t IMUQueue;
extern QueueHandle_t MagQueue;
extern QueueHandle_t Bar30Queue;
extern QueueHandle_t HumidQueue;
extern QueueHandle_t BoardQueue;
extern QueueHandle_t ThrusterQueue;
extern QueueHandle_t ServoQueue[8];

/* Mutex lock for making sure that SPI1 communication doesn't collide */
extern osMutexId_t spiMutex;

/* Arrays to store parameters that ID each servo task to its corresponding servo */
TIM_HandleTypeDef* tims[2] = {&htim2, &htim3};
uint32_t channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
extern ServoParams_t servoParams[2][4];

/* Definitions for task space and handles */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
	.name = "LED_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
	.name = "IMU_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t magTaskHandle;
const osThreadAttr_t magTask_attributes = {
	.name = "Mag_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t bar30TaskHandle;
const osThreadAttr_t bar30Task_attributes = {
	.name = "Bar30_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t humidTaskHandle;
const osThreadAttr_t humidTask_attributes = {
	.name = "Humid_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t boardTaskHandle;
const osThreadAttr_t boardTask_attributes = {
	.name = "Board_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t thrusterTaskHandle;
const osThreadAttr_t thrusterTask_attributes = {
	.name = "Thruster_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t servoTaskHandle[8];
const osThreadAttr_t servoTask_attributes = {
	.name = "Servo_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t picomTaskHandle;
const osThreadAttr_t picomTask_attributes = {
	.name = "PiCom_Task",
	.stack_size = 1024 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* MCU Configuration--------------------------------------------------------*/
	tims[0] = &htim2;
	tims[1] = &htim3;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_UART4_Init();
	MX_SPI2_Init();
	MX_UART5_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();

	/* Init scheduler */
	osKernelInitialize();

	/* force all pins high as soon as possible for SPI1 */
	HAL_GPIO_WritePin(GPIOC, SPI1_CS0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_SET);

	/* Enforce only one device to communicate over SPI line at a time */
	spiMutex = osMutexNew(NULL);

	/* Queue creation */
	IMUQueue = xQueueCreate(10, sizeof(IMUCom_t));
	MagQueue = xQueueCreate(10, sizeof(MagCom_t));
	Bar30Queue = xQueueCreate(10, sizeof(Bar30Com_t));
	HumidQueue = xQueueCreate(10, sizeof(HumidCom_t));
	BoardQueue = xQueueCreate(10, sizeof(BoardCom_t));
	ThrusterQueue = xQueueCreate(10, sizeof(ThrusterCmd_t));
	for(int i = 0; i < 8; i++) { ServoQueue[i] = xQueueCreate(10, sizeof(ServoCmd_t)); }

	/* Create the thread(s) */
	ledTaskHandle = osThreadNew(LED_Task, NULL, &ledTask_attributes);
	picomTaskHandle = osThreadNew(PiCom_Task, NULL, &picomTask_attributes);
	imuTaskHandle = osThreadNew(IMU_Task, NULL, &imuTask_attributes);
	magTaskHandle = osThreadNew(Mag_Task, NULL, &magTask_attributes);
	bar30TaskHandle = osThreadNew(Bar30_Task, NULL, &bar30Task_attributes);
	humidTaskHandle = osThreadNew(Humid_Task, NULL, &humidTask_attributes);
	boardTaskHandle = osThreadNew(Board_Task, NULL, &boardTask_attributes);
	thrusterTaskHandle = osThreadNew(Thruster_Task, NULL, &thrusterTask_attributes);
	for (int i=0; i<2; i++)
	{
		for (int k=0; k<4; k++)
		{
			servoParams[i][k].tim = tims[i];
			servoParams[i][k].channel = channels[k];
			servoTaskHandle[(i*4)+k] = osThreadNew(Servo_Task, &servoParams[i][k], &servoTask_attributes);
		}
	}

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	while (1) {}
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

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
	/* I2C1 parameter configuration*/
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
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
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_SLAVE;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 83;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 19999;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 83;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 19999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
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
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		Error_Handler();
	}
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
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED_OUT1_Pin|LED_OUT2_Pin|SPI1_CS0_Pin|SPI1_CS1_Pin
						  |SPI1_CS2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_OUT3_GPIO_Port, LED_OUT3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
						   PC1 PC2 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
						  |GPIO_PIN_1|GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_OUT1_Pin LED_OUT2_Pin SPI1_CS0_Pin SPI1_CS1_Pin
						   SPI1_CS2_Pin */
	GPIO_InitStruct.Pin = LED_OUT1_Pin|LED_OUT2_Pin|SPI1_CS0_Pin|SPI1_CS1_Pin
						  |SPI1_CS2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_OUT3_Pin */
	GPIO_InitStruct.Pin = LED_OUT3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_OUT3_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {}
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
