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
/* Array to store pre-calculated sin wave */
float sin_a[SERVO_ARRAY_SIZE];



/*
 *  Test Task for checking flashing and power
 */
void LED_Task(void* argument)
{
	for (int i=0; i<8; i++)
	{
		HAL_GPIO_TogglePin(LED_OUT3_GPIO_Port, LED_OUT3_Pin);
		osDelay(500);
	}
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



/*
 *  Step through data byte by byte and calculate xor checksum of all the data to check for corruption
 */
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
 *  Find the Start of Packet and return the index for synchronizing communication
 */
int FindSoP(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len - 1; i++)
    {
        if (buf[i] == 0xAA && buf[i + 1] == 0x55)
        {
            return i;
        }
    }
    return -1;
}

/*
 *  Handle synchronization, checksum, and packet casting
 *  	returns true if packet is found
 *  	returns false otherwise
 */
bool FindInput(uint8_t rx[PACKETSIZE], InPacket_t *in)
{
	static uint8_t rx_buffer[RX_BUFFER_SIZE];
	static size_t rx_len = 0;

	// Append new SPI data to rolling buffer
	if (rx_len + PACKETSIZE <= RX_BUFFER_SIZE)
	{
	    memcpy(&rx_buffer[rx_len], rx, PACKETSIZE);
	    rx_len += PACKETSIZE;
	}
	else
	    rx_len = 0; // Overflow protection: reset buffer

	// Try to extract a valid packet
	while (rx_len >= sizeof(InPacket_t))
	{
	    int idx = FindSoP(rx_buffer, rx_len);

	    if (idx < 0) // No SOP found, discard buffer
	    {
	        rx_len = 0;
	        break;
	    }

	    // Not enough data yet for full packet
	    if ((size_t)(idx + sizeof(InPacket_t)) > rx_len) break;

	    uint8_t *pkt_ptr = &rx_buffer[idx];

	    // Compute checksum over received packet (excluding checksum field)
	    uint8_t calc = CalculateChecksum(pkt_ptr, sizeof(InPacket_t) - sizeof(in->CheckSum));
	    uint8_t recv = pkt_ptr[sizeof(InPacket_t) - 1];

	    if (calc == recv) // Valid packet
	    {
	        memcpy(in, pkt_ptr, sizeof(InPacket_t));

	        // Remove consumed bytes
	        size_t consumed = idx + sizeof(InPacket_t);
	        memmove(rx_buffer, &rx_buffer[consumed], rx_len - consumed);
	        rx_len -= consumed;

	        return true; // process one packet per loop
	    }
	    else // Bad checksum, shift by 1 and retry
	    {
	        memmove(rx_buffer, &rx_buffer[idx + 1], rx_len - (idx + 1));
	        rx_len -= (idx + 1);
	    }
	}
	return false;
}

bool HandleInput(InPacket_t input, ThrusterCmd_t* thrusterCmd, ServoCmd_t servoCmd[8])
{
	bool valid = true;
	float thrust = input.Thrust;
	float value = input.Value;
	char* cmd = input.Command;

	// command interpretation ladder concerned with servos
	if (!strcmp(cmd, "fwd")) // forward
	{
		for (int i=0; i<8; i++)
		{
			servoCmd[i].amplitude = 80 * value;
			servoCmd[i].speed = (uint16_t)lroundf(100 * value);
			servoCmd[i].vert_offset = 0;
			if (i%2 == 0) servoCmd[i].horz_offset = 180;
			else servoCmd[i].horz_offset = 0;
		}
	}
	else if (!strcmp(cmd, "bwd")) // backward
	{
		for (int i=0; i<8; i++)
		{
			servoCmd[i].amplitude = 80 * value;
			servoCmd[i].speed = (uint16_t)lroundf(100 * value);
			servoCmd[i].vert_offset = 0;
			if (i%2 == 0) servoCmd[i].horz_offset = 180;
			else servoCmd[i].horz_offset = 0;
		}
	}
	else
		valid = false;

	thrusterCmd->thrust = (int8_t)lroundf(thrust);

	return valid;
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
	bool packet_found = false;

	// TODO: Establish communications check with Raspberry Pi
	// TODO: Check which sensors are online for queue receiving
	// TODO: Report to Raspberry Pi which sensors are offline

	/* clean all communications structures to avoid sending old data */
	memset(tx, 0, PACKETSIZE);
	memset(rx, 0, PACKETSIZE);
	memset(&out, 0, sizeof(OutPacket_t));
	memset(&in, 0, sizeof(InPacket_t));
	memset(&thrusterCommand, 0, sizeof(ThrusterCmd_t));
	for (int i=0; i<8; i++) memset(&servoCommand[i], 0, sizeof(ServoCmd_t));

	for (;;)
	{
		/* collect new data from all sensor queues into the outgoing packet */
		out.SoP[0] = 0xAA;
		out.SoP[1] = 0x55;
		xQueueReceive(IMUQueue, &out.IMUCom, 0);
		xQueueReceive(MagQueue, &out.MagCom, 0);
		xQueueReceive(Bar30Queue, &out.Bar30Com, 0);
		xQueueReceive(HumidQueue, &out.HumidCom, 0);
		xQueueReceive(BoardQueue, &out.BoardCom, 0);
		out.CheckSum = CalculateChecksum((uint8_t*)&out, sizeof(OutPacket_t) - sizeof(out.CheckSum));

		/* serialize packet struct into bytes */
		memcpy(tx, &out, sizeof(OutPacket_t));

		/* perform transfer (using NSS so no need to manually pull any chips low or high) */
		HAL_SPI_TransmitReceive(&hspi2, tx, rx, PACKETSIZE, portMAX_DELAY);

		/* de-serialize byte packet into struct and confirm checksum */
		packet_found = FindInput(rx, &in);

		/* distribute commands */
		if (packet_found)
		{
			if (HandleInput(in, &thrusterCommand, servoCommand))
			{
				xQueueSend(ThrusterQueue, &thrusterCommand, 1);
				for (int i=0; i<8; i++) xQueueSend(ServoQueue[i], &servoCommand[i], 1);
			}
		}
	}
}



/*
 *  Helper function to write to a specific register on a specific SPI line and on a specific chip select
 */
void SPI_WriteReg(uint16_t chip, uint8_t reg, uint8_t data)
{
	uint8_t tx[2];
	tx[0] = reg & 0x7F;   // write mode
	tx[1] = data;

	osMutexAcquire(spiMutex, osWaitForever);
	HAL_GPIO_WritePin(GPIOC, chip, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOC, chip, GPIO_PIN_SET);
	osMutexRelease(spiMutex);
}

/*
 *  Helper function to read from a specific register on a specific SPI line and on a specific chip select
 */
uint8_t SPI_ReadReg(uint16_t chip, uint8_t reg)
{
	uint8_t tx[2];
	uint8_t rx[2];

	tx[0] = reg | 0xC0;  // 0x80 (read) + 0x40 (auto-increment)
	tx[1] = 0x00;

	osMutexAcquire(spiMutex, osWaitForever);
	HAL_GPIO_WritePin(GPIOC, chip, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOC, chip, GPIO_PIN_SET);
	osMutexRelease(spiMutex);

	return rx[1];
}

/*
 *  Helper function to read multiple bytes of data from a specific SPI line on a specific chip select
 */
void SPI_Read(uint16_t chip, uint8_t reg, uint8_t* buffer, uint8_t length)
{
	uint8_t tx[length + 1];
	uint8_t rx[length + 1];

	tx[0] = reg | 0x80;   // read + auto increment
	memset(&tx[1], 0, length);

	osMutexAcquire(spiMutex, osWaitForever);
	HAL_GPIO_WritePin(GPIOC, chip, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOC, chip, GPIO_PIN_SET);
	osMutexRelease(spiMutex);

	memcpy(buffer, &rx[1], length);
}



/*
 *  Initialize the settings of the IMU to prepare for active reading
 */
bool IMU_Init(void)
{
	return false;
}

/*
 *  Task definition for continuous reading from IMU
 *  	SPI 1 Chip 1
 */
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



/*
 *  Initialize the settings of the Magnetometer to prepare for active reading
 *  	who_am_i = 0x30
 */
bool Mag_Init(void)
{
	SPI_WriteReg(SPI1_CS2_Pin, 0x09, 0x10); // set magnetometer
	osDelay(5);
	SPI_WriteReg(SPI1_CS2_Pin, 0x09, 0x20); // set automatic set/reset
	return (SPI_ReadReg(SPI1_CS2_Pin, 0x2F) == 0x30); // return ID check to return online/offline
}

/*
 *  Task definition for continuous reading from Magnetometer
 *  	SPI 1 Chip 2
 */
void Mag_Task(void *argument)
{
	MagCom_t output = {0};
	uint32_t raw_x, raw_y, raw_z;
	uint32_t set_x, set_y, set_z;
	int32_t field_x, field_y, field_z;
	uint8_t buffer[7];
	uint8_t set_buffer[7];

	if (!Mag_Init())
	{
		output.head_x = 666;
		output.head_y = 666;
		output.head_z = 666;
		for (;;)
		{
			xQueueSend(MagQueue, &output, 0);
			osDelay(1);
		}
	}
	osDelay(INITPAUSE);

	uint32_t tim_rst = 10;
	uint32_t timeout = tim_rst; // ms max
	for (;;)
	{
		// 1. SET
		timeout = tim_rst;
		SPI_WriteReg(SPI1_CS2_Pin, 0x09, 0x08);
		while (!(SPI_ReadReg(SPI1_CS2_Pin, 0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		// 2. MEASURE (after SET)
		timeout = tim_rst;
		SPI_WriteReg(SPI1_CS2_Pin, 0x09, 0x01);
		while (!(SPI_ReadReg(SPI1_CS2_Pin, 0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		SPI_Read(SPI1_CS2_Pin, 0x00, set_buffer, 7);

		set_x =
			((uint32_t)set_buffer[0] << 10) |
			((uint32_t)set_buffer[1] << 2)  |
			((set_buffer[6] >> 6) & 0x03);
		set_y =
			((uint32_t)set_buffer[2] << 10) |
			((uint32_t)set_buffer[3] << 2)  |
			((set_buffer[6] >> 4) & 0x03);
		set_z =
			((uint32_t)set_buffer[4] << 10) |
			((uint32_t)set_buffer[5] << 2)  |
			((set_buffer[6] >> 2) & 0x03);

		// 3. RESET
		SPI_WriteReg(SPI1_CS2_Pin, 0x09, 0x10);
		timeout = tim_rst;
		while (!(SPI_ReadReg(SPI1_CS2_Pin, 0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		// 4. MEASURE
		SPI_WriteReg(SPI1_CS2_Pin, 0x09, 0x01);
		timeout = tim_rst;
		while (!(SPI_ReadReg(SPI1_CS2_Pin, 0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		SPI_Read(SPI1_CS2_Pin, 0x00, buffer, 7); // read all 7 registers of data (0x00 - 0x06)

		// take 18 bit data
		raw_x =
			((uint32_t)buffer[0] << 10) |
			((uint32_t)buffer[1] << 2)  |
			((buffer[6] >> 6) & 0x03);
		raw_y =
			((uint32_t)buffer[2] << 10) |
			((uint32_t)buffer[3] << 2)  |
			((buffer[6] >> 4) & 0x03);
		raw_z =
			((uint32_t)buffer[4] << 10) |
			((uint32_t)buffer[5] << 2)  |
			((buffer[6] >> 2) & 0x03);

		field_x = ((int32_t)set_x - (int32_t)raw_x) / 2;
		field_y = ((int32_t)set_y - (int32_t)raw_y) / 2;
		field_z = ((int32_t)set_z - (int32_t)raw_z) / 2;

		// convert to signed gaussian units
		output.head_x = field_x / 16384.0f;
		output.head_y = field_y / 16384.0f;
		output.head_z = field_z / 16384.0f;

		// don't wait if queue is full, avoid old data piling up
		xQueueSend(MagQueue, &output, 0);
	}
}



/*
 *  Initialize the settings of the Bar30 Pressure/Temp sensor to prepare for active reading
 */
bool Bar30_Init(void)
{
	return false;
}

/*
 *  Task definition for continuous reading from Bar30 Pressure/Temp Sensor
 */
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



/*
 *  Initialize the settings of the Humidity sensor to prepare for active reading
 */
bool Humid_Init(void)
{
	return false;
}

/*
 *  Task definition for continuous reading from Humidity sensor
 */
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



/*
 *  Initialize the settings of the Board temperature sensor to prepare for active reading
 */
bool Board_Init(void)
{
	return false;
}

/*
 *  Task definition for continuous reading from Board temperature sensor
 */
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



/*
 *  Helper function to convert an angle in degrees to a bounds checked pulse width
 */
uint16_t angle_to_pulse(int16_t angle)
{
	return 0;
}

/*
 *  Initialize the positions of the servos to prepare for actuation
 */
void Servo_Init(TIM_HandleTypeDef* tim, uint32_t channel)
{
	//
}

/*
 *  Task definition for continuous actuation of the servos
 */
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



/*
 *  Helper function to convert an integer percentage from -100% to 100% to a bounds checked pulse width
 */
uint16_t percent_to_pulse(int8_t percent)
{
	return 0;
}

/*
 *  Specific initialization routine for hardware
 */
void Thruster_Init(void)
{
	//
}

/*
 *  Task definition for continuous commanding of the thruster
 */
void Thruster_Task(void* arugment)
{
	ThrusterCmd_t input;

	for (;;)
	{
		xQueueReceive(ThrusterQueue, &input, portMAX_DELAY); // only update at new command
		osDelay(1);
	}
}
