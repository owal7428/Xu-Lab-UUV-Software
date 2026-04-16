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
QueueHandle_t ServoQueue;
QueueHandle_t ThrusterQueue;

/* Mutex lock for making sure that SPI1 communication doesn't collide */
osMutexId_t spiMutex;

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
bool HandleInput(uint8_t rx[PACKETSIZE], InPacket_t *in)
{
	uint8_t rx_buffer[RX_BUFFER_SIZE];
	size_t rx_len = 0;

	// Append new SPI data to rolling buffer
	if (rx_len + PACKETSIZE <= RX_BUFFER_SIZE)
	{
	    memcpy(&rx_buffer[rx_len], rx, PACKETSIZE);
	    rx_len += PACKETSIZE;
	}
	else
	{
	    // Overflow protection: reset buffer
	    rx_len = 0;
	}

	// Try to extract a valid packet
	int packet_found = 0;
	while (rx_len >= sizeof(InPacket_t))
	{
	    int idx = FindSoP(rx_buffer, rx_len);

	    if (idx < 0)
	    {
	        // No SOP found, discard buffer
	        rx_len = 0;
	        break;
	    }

	    // Not enough data yet for full packet
	    if ((size_t)(idx + sizeof(InPacket_t)) > rx_len) break;

	    uint8_t *pkt_ptr = &rx_buffer[idx];

	    // Compute checksum over received packet (excluding checksum field)
	    uint8_t calc = CalculateChecksum(pkt_ptr, sizeof(InPacket_t) - sizeof(in->CheckSum));
	    uint8_t recv = pkt_ptr[sizeof(InPacket_t) - 1];

	    if (calc == recv)
	    {
	        // Valid packet
	        memcpy(in, pkt_ptr, sizeof(InPacket_t));
	        packet_found = 1;

	        // Remove consumed bytes
	        size_t consumed = idx + sizeof(InPacket_t);
	        memmove(rx_buffer, &rx_buffer[consumed], rx_len - consumed);
	        rx_len -= consumed;

	        break; // process one packet per loop
	    }
	    else
	    {
	        // Bad checksum, shift by 1 and retry
	        memmove(rx_buffer, &rx_buffer[idx + 1], rx_len - (idx + 1));
	        rx_len -= (idx + 1);
	    }
	}
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
	ServoCmd_t servoCommand;
	ThrusterCmd_t thrusterCommand;
	bool packet_found = false;

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
		memset(&servoCommand, 0, sizeof(ServoCmd_t));
		memset(&thrusterCommand, 0, sizeof(ThrusterCmd_t));

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

		/* de-serialize byte packet into struct and confirm checksum */
		packet_found = HandleInput(rx, &in);

		/* distribute commands */
		// TODO: Adjust queue timeouts in case of queue overflowing
		// TODO: Avoid sending duplicate commands to thruster or servos
		if (packet_found)
		{
			xQueueSend(ServoQueue, &in.ServoCmd, 1);
			xQueueSend(ThrusterQueue, &in.ThrusterCmd, 1);
		}
	}
}



/*
 *  Helper function to write to a specific register on a specific SPI line and on a specific chip select
 */
void SPI_WriteReg(uint8_t line, uint8_t chip, uint8_t reg, uint8_t data)
{
	//
}

/*
 *  Helper function to read from a specific register on a specific SPI line and on a specific chip select
 */
uint8_t SPI_ReadReg(uint8_t line, uint8_t chip, uint8_t reg)
{
	//
}

/*
 *  Helper function to read multiple bytes of data from a specific SPI line on a specific chip select
 */
void SPI_Read(uint8_t line, uint8_t chip, uint8_t reg, uint8_t* buffer, uint8_t length)
{
	//
}



/*
 *  Initialize the settings of the IMU to prepare for active reading
 */
bool IMU_Init(void)
{
	//
}

/*
 *  Task definition for continuous reading from IMU
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
 */
bool Mag_Init(void)
{
	//
}

/*
 *  Task definition for continuous reading from Magnetometer
 */
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



/*
 *  Initialize the settings of the Bar30 Pressure/Temp sensor to prepare for active reading
 */
void Bar30_Init(void)
{
	//
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
	//
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
	//
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

/* Converts servo angle to PWM pulse */
uint16_t angle_to_pulse(int16_t angle)
{
	// Clamp angle to +/- 80 degrees

	if (angle > 80) angle = 80;
	if (angle < -80) angle = -80;

	// Converts [-80, 80] to [7.5, 167.5] angle range (servo range is [0, 175])
	angle += (87.5);

	// Converts angle to [900 us, 2100 us] pulse range
	return 900 + ((uint32_t)angle * 1200) / 175;
}

#define INCREMENT_RESOLUTION 16384
#define INCREMENT_RESOLUTION_INV 0.00006103515625

/*
 *  Task definition for continuous actuation of the servos,
 *  implements a simple forward movement pattern
 */
void Servo_Task(void* argument)
{
	ServoCmd_t input;

	uint16_t stroke_pulse = 1500;
	uint16_t pitch_pulse = 2100;
	uint16_t step = 10;    // Frequency = 0.061 Hz * step
	float phase = 0;

	// Reset servos to starting positions

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, stroke_pulse);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pitch_pulse);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, stroke_pulse);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pitch_pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, stroke_pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pitch_pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, stroke_pulse);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch_pulse);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	osDelay(1000); // Give servos time to reset

	for (;;)
	{
		xQueueReceive(ServoQueue, &input, 0); // get pattern parameters

		float phase_normalized = phase * INCREMENT_RESOLUTION_INV;

		// Calculate pitch and stroke angles

		float sin_val = sinf(2 * M_PI * phase_normalized);
		float cos_val = cosf(2 * M_PI * phase_normalized);

		float stroke_angle = 80 * sin_val;
		stroke_pulse = angle_to_pulse(lroundf(stroke_angle));

		// float pitch_angle = 80 * cos_val * (2 - fabsf(cos_val)); // More sharp
		float pitch_angle = 80 * cos_val * (1.5 - 0.5 * fabsf(cos_val)); // Less sharp
		pitch_pulse = angle_to_pulse(lroundf(pitch_angle));

		// stream to all 8 servo pinouts
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, stroke_pulse);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pitch_pulse);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, stroke_pulse);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pitch_pulse);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, stroke_pulse);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pitch_pulse);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, stroke_pulse);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch_pulse);

		phase += step * input.forward;

		if (phase >= INCREMENT_RESOLUTION)
			phase -= INCREMENT_RESOLUTION; // Reset loop

		osDelay(1);
	}
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
