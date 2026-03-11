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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "freertos.h"
#include "queue.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include <math.h>
#include <stdbool.h>

#define INITPAUSE 7000 // 7.0 s
#define BAR30_ADDR 0x76<<1
//constexpr int BAR30_ADDR = 0x76<<1;
//constexpr HUM_ADDR = 0x70<<1;
//constexpr EPROM_ADDR = 0x50<<1;

typedef struct
{
	float amplitude;
	uint16_t speed;
	int16_t offset;
} ServoCmd_t;

typedef struct
{
	int8_t thrust;
} ThrusterCmd_t;

typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}IMUCom_t;

typedef struct
{
	float head_x;
	float head_y;
	float head_z;
}MagCom_t;

typedef struct
{
	float pressure;
	float water_temp;
}Bar30Com_t;

typedef struct
{
	float humidity;
	float air_temp;
}HumidCom_t;

typedef struct
{
	float board_temp;
}BoardCom_t;

#define SERVO_QUEUE_ITEM_SIZE sizeof(ServoCmd_t)
#define SERVO_STEP_SIZE (0x01<<14)

// definitions for SPI high and low pulling
#define CS_PORT GPIOC
#define ACC_CS_PIN  SPI1_CS1_Pin
#define GYRO_CS_PIN SPI1_CS2_Pin
#define MAG_CS_PIN SPI1_CS3_Pin

void IMU_CS_LOW() {
	HAL_GPIO_WritePin(CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}

void IMU_CS_HIGH() {
	HAL_GPIO_WritePin(CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

void MAG_CS_LOW() {
	HAL_GPIO_WritePin(CS_PORT, MAG_CS_PIN, GPIO_PIN_RESET);
}

void MAG_CS_HIGH() {
	HAL_GPIO_WritePin(CS_PORT, MAG_CS_PIN, GPIO_PIN_SET);
}

// Bar30 Calibration coefficients
uint16_t C[7];

TIM_HandleTypeDef* tims[2];
uint32_t channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

typedef struct
{
    TIM_HandleTypeDef *tim;
    uint32_t channel;
} ServoParams_t;
ServoParams_t servoParams[2][4];

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

QueueHandle_t ServoQueue[8];
QueueHandle_t ThrusterQueue;
QueueHandle_t IMUQueue;
QueueHandle_t MagQueue;
QueueHandle_t Bar30Queue;
QueueHandle_t HumidQueue;
QueueHandle_t BoardQueue;

osMutexId_t spiMutex;

/* Definitions for task space and handles */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
	.name = "LED_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t servoTaskHandle[8];
const osThreadAttr_t servoTask_attributes = {
	.name = "Servo_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t thrusterTaskHandle;
const osThreadAttr_t thrusterTask_attributes = {
	.name = "Thruster_Task",
	.stack_size = 128 * 4,  // stack in bytes
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t bar30TaskHandle;
const osThreadAttr_t bar30Task_attributes = {
	.name = "Bar30_Task",
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM12_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);

static float sin_t[SERVO_STEP_SIZE];

/*
 *  Convert angle in degrees to a pulse width between 900-2100 µs
 *  	For User convenience input 0 degrees is centered, +/- 80 degrees are the extremes
 */
uint16_t angle_to_pulse(int16_t angle)
{
	// clamp to bounds first to ensure valid inputs
	if (angle > 80) angle = 80;
	if (angle < -80) angle = -80;

	// translating an angle between -80 to 80 degrees into 0 to 160 degrees for the math
	angle += 80;

	return 900 + ((uint32_t)angle * 1200) / 160;
}

/*
 * Convert percentage between -100 to 100 into a pulse width between 1000 µs to 2000 µs
 * 		-100 is reverse thrust while 100 is forward thrust
 */
uint16_t percent_to_pulse(int16_t percent)
{
	// clamp to bounds
	if (percent > 100) percent = 100;
	if (percent < -100) percent = -100;

	// translate to range of 0 to 200 for pulse width math
	percent += 100;

	return 1000 + ((uint32_t)percent * 1000) / 200;
}

/*
 *  Test Task for checking correct flashing
 */
void LED_Task(void* argument)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(LED_Output_GPIO_Port, LED_Output_Pin);
		osDelay(500);
	}
}

/*
 *  set to 0 position and wait while other systems initialize
 */
void Servo_Init(TIM_HandleTypeDef* tim, uint32_t channel)
{
	__HAL_TIM_SET_COMPARE(tim, channel, 1500);
	HAL_TIM_PWM_Start(tim, channel);
	osDelay(INITPAUSE);
}

/*
 *  Task Prototype that handles actuating a/the servos
 *  Spawning one task per servo allows them to segment and control more easily independently
 */
void Servo_Task(void *argument)
{
	ServoParams_t* params = (ServoParams_t*)argument;
	int16_t index = 0;
	int16_t step = 1;

	Servo_Init(params->tim, params->channel);

	for (;;)
	{
		index = (index + step) % SERVO_STEP_SIZE;
		__HAL_TIM_SET_COMPARE(params->tim, params->channel, angle_to_pulse(lroundf(80*sin_t[index])));
		osDelay(1);
	}
}

/*
 *  Initialize thruster by ensuring that PWM is to 1500 µs and a delay is enforced
 *  	delay required for the thruster to start
 */
void Thruster_Init()
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	osDelay(INITPAUSE);
}

/*
 *  Task to handle signaling control to thruster via PWM
 *  	1000 µs = full reverse, 1500 µs = stopped, 2000 µs = full ahead
 */
void Thruster_Task(void *argument)
{
	int16_t step = 0;
	int16_t stride = 1;
	Thruster_Init();

	for (;;)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, percent_to_pulse(step));
		step += stride;
		if (step >= 100) stride*=-1;
		if (step <= -100) stride*=-1;
		osDelay(5);
	}
}

/*
 *  Initialize sensor and read PROM
 */
void Bar30_Init(void) {
    uint8_t cmd;
    HAL_Delay(10);

    // 1. Reset sensor
    cmd = 0x1E;
    HAL_I2C_Master_Transmit(&hi2c1, BAR30_ADDR, &cmd, 1, 100);
    osDelay(10);

    // 2. Read PROM calibration C1..C6
    for(uint8_t i=0; i<6; i++) {
        cmd = 0xA0 + (i+1)*2;
        uint8_t buf[2];

        HAL_I2C_Master_Transmit(&hi2c1, BAR30_ADDR, &cmd, 1, 100);
        HAL_I2C_Master_Receive(&hi2c1, BAR30_ADDR, buf, 2, 100);

        C[i+1] = (buf[0]<<8) | buf[1];
    }

    osDelay(INITPAUSE);
}

/*
 *  Read ADC for a given conversion command (D1 or D2)
 */
uint32_t Bar30_ReadADC(uint8_t conv_cmd, uint16_t delay_ms) {
    uint8_t cmd = conv_cmd;
    uint8_t buf[3];

    HAL_I2C_Master_Transmit(&hi2c1, BAR30_ADDR, &cmd, 1, 100);
    HAL_Delay(delay_ms);

    cmd = 0x00; // ADC read
    HAL_I2C_Master_Transmit(&hi2c1, BAR30_ADDR, &cmd, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, BAR30_ADDR, buf, 3, 100);

    return ((uint32_t)buf[0]<<16) | ((uint32_t)buf[1]<<8) | buf[2];
}

/*
 *  Read temperature (°C) and pressure (Pa)
 */
void Bar30_ReadTempPressure(float *temperature, float *pressure)
{
    // 1. Read raw ADC values (OSR=4096)
    uint32_t D1 = Bar30_ReadADC(0x48, 10); // Pressure
    uint32_t D2 = Bar30_ReadADC(0x58, 10); // Temperature

    // 2. First-order compensation
    int32_t dT = (int32_t)D2 - ((int32_t)C[5] << 8);

    int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;   // 0.01 °C

    int64_t OFF  = ((int64_t)C[2] << 17) + (((int64_t)C[4] * dT) >> 6);
    int64_t SENS = ((int64_t)C[1] << 16) + (((int64_t)C[3] * dT) >> 7);

    // 3. Second-order temperature compensation
    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000)  // Below 20°C
    {
        T2    = (3LL * ((int64_t)dT * dT)) >> 33;
        OFF2  = (3LL * (TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = (5LL * (TEMP - 2000) * (TEMP - 2000)) >> 3;

        if (TEMP < -1500)  // Below -15°C
        {
            OFF2  += 7LL * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 4LL * (TEMP + 1500) * (TEMP + 1500);
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    // 4. Final pressure calculation
    int32_t P = (int32_t)((((int64_t)D1 * SENS) >> 21) - OFF) >> 13;
    // P is in 0.1 mbar

    // 5. Convert to requested units
    *temperature = TEMP / 100.0f;   // °C
    *pressure    = P / 10000.0f;    // bar
}

/*
 *  Pressure/Temperature Sensor Task
 */
void Bar30_Task(void* argument)
{
	Bar30_Init();
	for (uint8_t addr = 0; addr < 128; addr++)
	{
	    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
	    {
	        printf("Found device at 0x%02X\r\n", addr);
	    }
	}
	if (HAL_I2C_IsDeviceReady(&hi2c1, BAR30_ADDR, 5, 100) != HAL_OK) return;

	float t0, p0, dt, dp, t, p = 0;
	for (;;)
	{
		Bar30_ReadTempPressure(&t, &p);
	}
}

/*
 *  Write to IMU (for initializing settings)
 */
void IMU_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   // write mode
    tx[1] = data;

    osMutexAcquire(spiMutex, osWaitForever);
    IMU_CS_LOW();

    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);

    IMU_CS_HIGH();
    osMutexRelease(spiMutex);
}

/*
 *  for checking IMU online by reading a specific register only
 */
uint8_t IMU_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;
    tx[1] = 0x00;

    osMutexAcquire(spiMutex, osWaitForever);
    IMU_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    IMU_CS_HIGH();
    osMutexRelease(spiMutex);

    return rx[1];
}

/*
 *  For retrieving raw gyroscope data
 */
void IMU_Read(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t tx[length + 1];
    uint8_t rx[length + 1];

    tx[0] = reg | 0x80;   // read + auto increment
    memset(&tx[1], 0, length);

    osMutexAcquire(spiMutex, osWaitForever);
    IMU_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 1, HAL_MAX_DELAY);

    IMU_CS_HIGH();
    osMutexRelease(spiMutex);

    memcpy(buffer, &rx[1], length);
}

/*
 *  Initialise SPI connection and check that IMU is available
 */
bool IMU_Init()
{
	IMU_ReadReg(0x00); // Dummy read to activate SPI
	osDelay(5);
	if (IMU_ReadReg(0x00) != 0x0F) return false; // device not found
	osDelay(INITPAUSE);
    return true;
}

/*
 *  Task for reading from on-board IMU (accelerometer & gyroscope) on SPI
 */
void IMU_Task(void *argument)
{
	uint8_t buffer[6];
	int16_t raw_gx, raw_gy, raw_gz;
	float gx, gy, gz;

	// constants to convert to dps
	float convgyro = 500.0f / 32768.0f;

	if (!IMU_Init()) return;

	for (;;)
	{
		// gyroscope data
		IMU_Read(0x02, buffer, 6);
		raw_gx = (int16_t)(buffer[1] << 8 | buffer[0]);
		raw_gy = (int16_t)(buffer[3] << 8 | buffer[2]);
		raw_gz = (int16_t)(buffer[5] << 8 | buffer[4]);

		gx = raw_gx * convgyro;
		gy = raw_gy * convgyro;
		gz = raw_gz * convgyro;

		osDelay(10); // 10ms delay = 100Hz
	}
}

/*
 *  SPI write function to Magnetometer
 */
void Mag_Write(uint8_t reg, uint8_t data)
{
	uint8_t tx[2];
	tx[0] = reg & 0x7F;   // write mode
	tx[1] = data;

	osMutexAcquire(spiMutex, osWaitForever);
	MAG_CS_LOW();

	HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);

	MAG_CS_HIGH();
	osMutexRelease(spiMutex);
}

/*
 *  SPI register read function from Magnetometer
 */
uint8_t Mag_ReadReg(uint8_t reg)
{
	uint8_t tx[2];
	uint8_t rx[2];

	tx[0] = reg | 0x80;
	tx[1] = 0x00;

	osMutexAcquire(spiMutex, osWaitForever);
	MAG_CS_LOW();

	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

	MAG_CS_HIGH();
	osMutexRelease(spiMutex);

	return rx[1];
}

/*
 *  SPI burst read function from Magnetometer
 */
void Mag_Read(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t tx[length + 1];
    uint8_t rx[length + 1];

    tx[0] = reg | 0x80;   // read + auto increment
    memset(&tx[1], 0, length);

    osMutexAcquire(spiMutex, osWaitForever);
    MAG_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 1, HAL_MAX_DELAY);

    MAG_CS_HIGH();
    osMutexRelease(spiMutex);

    memcpy(buffer, &rx[1], length);
}

/*
 *  Initialize Magnetometer and pause for initialization sequence
 */
bool Init_Mag()
{
	Mag_Write(0x09, 0x10); // set magnetometer
	osDelay(1);
	Mag_Write(0x09, 0x20); // set automatic set/reset
	osDelay(INITPAUSE);

	return (Mag_ReadReg(0x2F) == 0x30); // return device ID to check online
}

/*
 *  Top-level task for reading from on-board magnetometer heading sensor
 */
void Mag_Task(void *argument)
{
	uint8_t buffer[7];
	uint32_t raw_x, raw_y, raw_z;
	int32_t x,y,z;
	float x_gauss, y_gauss, z_gauss;
	float heading;

	if (!Init_Mag()) return;

	for (;;)
	{
		Mag_Write(0x09,0x01); // initiate measurement
		while (!(Mag_ReadReg(0x08) & 0x01)); // wait for reading to be ready
		Mag_Read(0x00, buffer, 7); // read all 7 registers of data (0x00 - 0x06)

		// take 18 bit data
		raw_x =
		    ((uint32_t)buffer[0] << 10) |
		    ((uint32_t)buffer[1] << 2)  |
		    ((buffer[6] >> 6) & 0x03);

		raw_y =
		    ((uint32_t)buffer[2] << 10) |
		    ((uint32_t)buffer[3] << 2)  |
		    ((buffer[6] >> 6) & 0x03);

		raw_z =
		    ((uint32_t)buffer[4] << 10) |
		    ((uint32_t)buffer[5] << 2)  |
		    ((buffer[6] >> 6) & 0x03);

		// convert to signed by subtracting mid-point
		x = raw_x - 131072;
		y = raw_y - 131072;
		z = raw_z - 131072;

		// convert to Gaussian units
		x_gauss = x / 16384.0f;
		y_gauss = y / 16384.0f;
		z_gauss = z / 16384.0f;

		// convert to a heading with 0 currently east
		float heading = atan2f(y, x) * (180.0f / M_PI);
		if (heading < 0) heading += 360;

		osDelay(9); // reading at 100 Hz so need to read every 9 ms and give 1 ms time to communicate (10 ms total)
	}

	return;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// populate a global lookup table for sin and cos ahead of time for performance
	for (int i=0; i<SERVO_STEP_SIZE; i++)
	{
		sin_t[i] = sin((2*M_PI*i)/SERVO_STEP_SIZE);
	}

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
	MX_TIM12_Init();
	MX_SPI2_Init();
	MX_UART5_Init();

	/* Init scheduler */
	osKernelInitialize();

	/* force all pins high as soon as possible for SPI */
	HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SPI1_CS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SPI1_CS3_Pin, GPIO_PIN_SET);

	/* Enforce only one device to communicate over SPI line at a time */
	spiMutex = osMutexNew(NULL);

	/* Queue creation */
	for(int i = 0; i < 8; i++)
	{
	    ServoQueue[i] = xQueueCreate(10, sizeof(ServoCmd_t));
	}

	ThrusterQueue = xQueueCreate(10, sizeof(ThrusterCmd_t));
	IMUQueue = xQueueCreate(10, sizeof(IMUCom_t));
	MagQueue = xQueueCreate(10, sizeof(MagCom_t));
	Bar30Queue = xQueueCreate(10, sizeof(Bar30Com_t));

	/* Create the thread(s) */
	ledTaskHandle = osThreadNew(LED_Task, NULL, &ledTask_attributes);
//	thrusterTaskHandle = osThreadNew(Thruster_Task, NULL, &thrusterTask_attributes);
	bar30TaskHandle = osThreadNew(Bar30_Task, NULL, &bar30Task_attributes);
//	imuTaskHandle = osThreadNew(IMU_Task, NULL, &imuTask_attributes);
//	magTaskHandle = osThreadNew(Mag_Task, NULL, &magTask_attributes);

	/* start separate tasks for all 8 servos across both timers and each of their 4 channels */
//	for (int i=0; i<2; i++)
//	{
//		for (int k=0; k<4; k++)
//		{
//			servoParams[i][k].tim = tims[i];
//			servoParams[i][k].channel = channels[k];
//			servoTaskHandle[(i*4)+k] = osThreadNew(Servo_Task, &servoParams[i][k], &servoTask_attributes);
//		}
//	}

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
  RCC_OscInitStruct.PLL.PLLM = 25;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim12);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Output_GPIO_Port, LED_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, SPI1_CS0_Pin|SPI1_CS1_Pin|SPI1_CS2_Pin|SPI1_CS3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, SPI1_CS0_Pin|SPI1_CS1_Pin|SPI1_CS2_Pin|SPI1_CS3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Output_Pin */
  GPIO_InitStruct.Pin = LED_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS0_Pin SPI1_CS1_Pin SPI1_CS2_Pin SPI1_CS3_Pin */
  GPIO_InitStruct.Pin = SPI1_CS0_Pin|SPI1_CS1_Pin|SPI1_CS2_Pin|SPI1_CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
