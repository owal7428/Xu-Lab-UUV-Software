/*
  ******************************************************************************
  * @file           : task_defs.c
  * @brief          : Definitions for reading and writing to peripherals
  ******************************************************************************
  */

#include "task_defs.h"



/* Definition of handles for all the tasks' communication queues */
QueueHandle_t IMUQueue;
QueueHandle_t MagQueue;
QueueHandle_t Bar30Queue;
QueueHandle_t HumidQueue;
QueueHandle_t BoardQueue;
QueueHandle_t ThrusterQueue;
QueueHandle_t ServoQueue[8];

/* Mutex lock for making sure that SPI1 communication doesn't collide */
osMutexId_t spiMutex;

/* Array to store parameters that ID each servo task to its corresponding servo */
ServoParams_t servoParams[2][4];



/*
 *  Test Task for checking flashing and power
 */
void LED_Task(void* argument)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOC, LED_OUT1_Pin);
		osDelay(50);
		HAL_GPIO_TogglePin(GPIOC, LED_OUT2_Pin);
		osDelay(50);
		HAL_GPIO_TogglePin(LED_OUT3_GPIO_Port, LED_OUT3_Pin);
		osDelay(500);
	}
}



uint8_t CalculateChecksum(uint8_t* data, size_t length)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < length; i++)
    {
        cs ^= data[i];  // XOR each byte
    }
    return cs;
}

/*
 *  Handles communication back and forth to and from the raspberry pi by assembling and deciphering packets
 */
void PiCom_Task(void * argument)
{
	uint8_t tx[PACKETSIZE];
	uint8_t rx[PACKETSIZE];
	OutPacket_t out;
	InPacket_t in;
	ThrusterCmd_t thrusterCommand;
	ServoCmd_t servoCommand[8];

	// TODO: Establish communications check with Raspberry Pi
	// TODO: Check which sensors are online for queue receiving
	// TODO: Report to Raspberry Pi which sensors are offline

	for (;;)
	{
		/* clean all communications structures to avoid sending old data */
		memset(tx, 0, PACKETSIZE);
		memset(rx, 0, PACKETSIZE);
		memset(&out, 0, sizeof(OutPacket_t));
		memset(&in, 0, sizeof(InPacket_t));
		memset(&thrusterCommand, 0, sizeof(ThrusterCmd_t));
		for (int i=0; i<8; i++) memset(&servoCommand[i], 0, sizeof(ServoCmd_t));

		/* collect new data from all sensor queues into the outgoing packet */
		out.SoP[0] = 0xAA;
		out.SoP[1] = 0x55;
		// TODO: Adjust queue timeouts in case of queues underflowing
		xQueueReceive(IMUQueue, &out.IMUCom, 1);
		xQueueReceive(MagQueue, &out.MagCom, 1);
		xQueueReceive(Bar30Queue, &out.Bar30Com, 1);
		xQueueReceive(HumidQueue, &out.HumidCom, 1);
		xQueueReceive(BoardQueue, &out.BoardCom, 1);
		out.CheckSum = CalculateChecksum((uint8_t*)&out, sizeof(OutPacket_t) - sizeof(out.CheckSum));

		/* serialize packet struct into bytes */
		memcpy(tx, &out, sizeof(OutPacket_t));

		/* perform transfer (using NSS so no need to manually pull any chips low or high) */
		HAL_SPI_TransmitReceive(&hspi2, tx, rx, PACKETSIZE, portMAX_DELAY);

		/* de-serialize byte packet into struct */
		memcpy(&in, rx, sizeof(InPacket_t));

		/* distribute commands */
		// TODO: Adjust queue timeouts in case of queue overflowing
		// TODO: Avoid sending duplicate commands to thruster or servos
		xQueueSend(ThrusterQueue, &in.ThrusterCmd, 1);
		for (int i=0; i<8; i++) xQueueSend(ServoQueue[i], &in.ServoCmd[i], 1);
	}
}



void SPI_WriteReg(uint8_t line, uint8_t chip, uint8_t reg, uint8_t data)
{
	//
}

uint8_t SPI_ReadReg(uint8_t line, uint8_t chip, uint8_t reg)
{
	//
}

void SPI_Read(uint8_t line, uint8_t chip, uint8_t reg, uint8_t* buffer, uint8_t length)
{
	//
}



bool IMU_Init(void)
{
	//
}

void IMU_Task(void *argument)
{
	IMUCom_t output = {0};
	output.acc_x = 1.0;
	output.acc_y = 2.0;
	output.acc_z = 3.0;
	output.gyro_x = 4.0;
	output.gyro_y = 5.0;
	output.gyro_z = 6.0;

	for (;;)
	{
		xQueueSend(IMUQueue, &output, 1);
		osDelay(1);
	}
}



bool Mag_Init(void)
{
	//
}

void Mag_Task(void *argument)
{
	MagCom_t output = {0};
	output.head_x = 1.0;
	output.head_y = 2.0;
	output.head_z = 3.0;

	for (;;)
	{
		xQueueSend(MagQueue, &output, portMAX_DELAY);
		osDelay(1);
	}
}



void Bar30_Init(void)
{
	//
}

void Bar30_Task(void* argument)
{
	Bar30Com_t output = {0};
	output.pressure = 1.0;
	output.water_temp = 2.0;

	for (;;)
	{
		xQueueSend(Bar30Queue, &output, portMAX_DELAY);
		osDelay(1);
	}
}



bool Humid_Init(void)
{
	//
}

void Humid_Task(void *argument)
{
	HumidCom_t output = {0};
	output.humidity = 1.0;
	output.air_temp = 2.0;

	for (;;)
	{
		xQueueSend(HumidQueue, &output, portMAX_DELAY);
		osDelay(1);
	}
}



bool Board_Init(void)
{
	//
}

void Board_Task(void *argument)
{
	BoardCom_t output = {0};
	output.board_temp = 1.0;

	for (;;)
	{
		xQueueSend(BoardQueue, &output, portMAX_DELAY);
		osDelay(1);
	}
}



uint16_t angle_to_pulse(int16_t angle)
{
	//
}

void Servo_Init(TIM_HandleTypeDef* tim, uint32_t channel)
{
	//
}

void Servo_Task(void* argument)
{
	ServoParams_t* params = (ServoParams_t*)argument;
	ServoCmd_t input;
	int8_t servo_index = 0;

	// figure out servo index from params
	if (params->tim == &htim2) servo_index = 0;
	else servo_index = 4;

	if (params->channel == TIM_CHANNEL_1) servo_index += 0;
	else if (params->channel == TIM_CHANNEL_2) servo_index += 1;
	else if (params->channel == TIM_CHANNEL_3) servo_index += 2;
	else servo_index += 3;

	for (;;)
	{
		xQueueReceive(ServoQueue[servo_index], &input, 0); // don't Interrupt wave motion, just check for news
		osDelay(1);
	}
}



uint16_t percent_to_pulse(int8_t percent)
{
	//
}

void Thruster_Init(void)
{
	//
}

void Thruster_Task(void* arugment)
{
	ThrusterCmd_t input;

	for (;;)
	{
		xQueueReceive(ThrusterQueue, &input, portMAX_DELAY); // only update at new command
		osDelay(1);
	}
}
