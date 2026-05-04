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
osMutexId_t initMutex;
osMutexId_t i2cMutex;

/* Calibration coefficients read from PROM — persist across task cycles */
static uint16_t bar30_prom[7];

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

bool DecodePacket(uint8_t rx[PACKETSIZE], InPacket_t *in)
{
	// Copy raw bytes into struct
    memcpy(in, rx, sizeof(InPacket_t));

    // Don't include last byte in checksum calculation (that byte is the checksum itself)
    uint8_t calc = CalculateChecksum((uint8_t*)in, sizeof(InPacket_t) - 1);

    if (calc != in->CheckSum)
        return false;

    if (in->SoP[0] != 0xAA || in->SoP[1] != 0x55)
        return false;

    return true;
}

/*
 *  Handles communication back and forth to and from the raspberry pi by assembling and deciphering packets
 */
void PiCom_Task(void * argument)
{
	uint8_t tx[PACKETSIZE] = {0};
	uint8_t rx[PACKETSIZE] = {0};
	OutPacket_t out = {0};
	InPacket_t in = {0};
	ServoCmd_t servoCommand;
	ThrusterCmd_t thrusterCommand;

	memset(&out, 0, sizeof(out));
	memset(&out, 0, sizeof(in));

	out.SoP[0] = 0xAA;
	out.SoP[1] = 0x55;
	out.CheckSum = CalculateChecksum((uint8_t*)&out, sizeof(OutPacket_t) - 1);

	memcpy(tx, &out, sizeof(out));

	for (;;)
	{
		// perform transfer (controller is slave device so we don't modify CS lines here)
		HAL_SPI_TransmitReceive(&hspi2, tx, rx, PACKETSIZE, portMAX_DELAY);

		/* distribute commands */
		if (DecodePacket(rx, &in))
		{
			xQueueSend(ServoQueue, &in.ServoCmd, 1);
			xQueueSend(ThrusterQueue, &in.ThrusterCmd, 1);
		}

		// Reset outgoing packet
		memset(&out, 0, sizeof(OutPacket_t));

		/* collect new data from all sensor queues into the outgoing packet */
		out.SoP[0] = 0xAA;
		out.SoP[1] = 0x55;
		xQueueReceive(IMUQueue, &out.IMUCom, 0);
		xQueueReceive(MagQueue, &out.MagCom, 0);
		xQueueReceive(Bar30Queue, &out.Bar30Com, 0);
		xQueueReceive(HumidQueue, &out.HumidCom, 0);
		xQueueReceive(BoardQueue, &out.BoardCom, 0);

		// Don't include last byte in checksum calculation
		out.CheckSum = CalculateChecksum((uint8_t*)&out, sizeof(OutPacket_t) - 1);

		/* serialize packet struct into bytes */
		memcpy(tx, &out, sizeof(OutPacket_t));	
	}
}

/* Converts servo angle to PWM pulse */
uint16_t angle_to_pulse(int16_t angle)
{
	// Clamp angle to +/- 65 degrees

	if (angle > 65) angle = 65;
	if (angle < -65) angle = -65;

	// Converts [-65, 65] to [7.5, 167.5] angle range (servo range is [0, 175])
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
	uint16_t stroke_pulse2 = 1500;
	uint16_t pitch_pulse2 = 2100;
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

		float stroke_angle = 65 * sin_val;
		float stroke_angle2 = -stroke_angle;
		stroke_pulse = angle_to_pulse(lroundf(stroke_angle));
		stroke_pulse2 = angle_to_pulse(lroundf(stroke_angle2));

		// float pitch_angle = 65 * cos_val * (2 - fabsf(cos_val)); // More sharp
		float pitch_angle = 65 * cos_val * (1.5 - 0.5 * fabsf(cos_val)); // Less sharp
		float pitch_angle2 = -pitch_angle;
		pitch_pulse = angle_to_pulse(lroundf(pitch_angle));
		pitch_pulse2 = angle_to_pulse(lroundf(pitch_angle2));

		// stream to all 8 servo pinouts
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, stroke_pulse);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pitch_pulse);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, stroke_pulse);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pitch_pulse);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, stroke_pulse2);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pitch_pulse2);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, stroke_pulse2);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pitch_pulse2);

		phase += step * input.forward;

		if (phase >= INCREMENT_RESOLUTION)
			phase -= INCREMENT_RESOLUTION; // Reset loop

		osDelay(1);
	}
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

/*
 *  Helper function to read multiple bytes of data from IMU SPI line
 */
void IMU_ReadSPI(uint8_t reg, uint8_t* buffer, uint8_t length)
{
	uint8_t tx[length + 2];  // 1 address + 1 dummy + length data
    uint8_t rx[length + 2];
	
    tx[0] = reg | 0x80;
    memset(&tx[1], 0, length + 1);
	
    osMutexAcquire(spiMutex, osWaitForever);
	
    HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_SET);
	
    osMutexRelease(spiMutex);
	
    memcpy(buffer, &rx[2], length);  // skip address echo AND dummy byte
}

/*
 *  Helper function to write to a specific register on IMU SPI line
 */
void IMU_WriteReg(uint8_t reg, uint16_t data)
{
	uint8_t tx[3];
	tx[0] = reg & 0x7F;   // write mode
	tx[1] = data & 0xFF;
	tx[2] = (data >> 8) & 0xFF;

	osMutexAcquire(spiMutex, osWaitForever);

	HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_SET);

	osMutexRelease(spiMutex);
}

/*
 *  Helper function to read from a specific register on IMU SPI line
 */
uint16_t IMU_ReadReg(uint8_t reg)
{
    uint8_t tx[3] = {reg | 0x80, 0x00, 0x00};
    uint8_t rx[3] = {0};

    osMutexAcquire(spiMutex, osWaitForever);

    HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOC, SPI1_CS1_Pin, GPIO_PIN_SET);

    osMutexRelease(spiMutex);

    return (uint16_t)((rx[1] << 8) | rx[2]);  // big-endian, LSB first
}

/*
 *  Initialize the settings of the IMU to prepare for active reading
 */
int8_t IMU_Init(void)
{
	uint8_t id, err;

	// Step 1: dummy read to switch from I3C/I2C to SPI mode
	IMU_ReadReg(0x00);
	osDelay(10);  // give interface time to settle

	// Step 2: verify chip ID
	id = IMU_ReadReg(0x00);
	if (id != 0x0048) return -1;

	// Step 3: check power status (ERR_REG bit[0] must be 0)
	err = IMU_ReadReg(0x01);
	if (err & 0x0001) return -2;  // fatal_err set

	// Step 4: configure — now safe to write
	IMU_WriteReg(0x20, 0x4029);  // ACC_CONF: normal, ±8g, 200Hz
	IMU_WriteReg(0x21, 0x4049);  // GYR_CONF: normal, ±2000dps, 200Hz

	osDelay(5);  // wait for sensor startup after mode change

	return true;
}

/*
 *  Task definition for continuous reading from IMU
 *  	SPI 1 Chip 1
 */
void IMU_Task(void *argument)
{
	IMUCom_t output = {0};
	int16_t raw_ax, raw_ay, raw_az;
	int16_t raw_gx, raw_gy, raw_gz;
	uint8_t buffer[12];

	int8_t err = IMU_Init();
	if (err < 0)
	{
		output.acc_x = SENSERROR;
		output.acc_y = SENSERROR;
		output.acc_z = SENSERROR;
		output.gyro_x = SENSERROR;
		output.gyro_y = SENSERROR;
		output.gyro_z = SENSERROR;

		for (;;)
		{
			xQueueSend(IMUQueue, &output, 0);
			osDelay(1);
		}
	}
	osDelay(INITPAUSE);

	for (;;)
	{
		IMU_ReadSPI(0x03, buffer, 12);

		raw_ax = (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]);  // LSB first
		raw_ay = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
		raw_az = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);
		raw_gx = (int16_t)((uint16_t)buffer[7] << 8 | buffer[6]);
		raw_gy = (int16_t)((uint16_t)buffer[9] << 8 | buffer[8]);
		raw_gz = (int16_t)((uint16_t)buffer[11] << 8 | buffer[10]);

		output.acc_x  = (float)raw_ax / 4096.0f;   // result in g
		output.acc_y  = (float)raw_ay / 4096.0f;
		output.acc_z  = (float)raw_az / 4096.0f;
		output.gyro_x = (float)raw_gx / 16.384f;   // result in °/s
		output.gyro_y = (float)raw_gy / 16.384f;
		output.gyro_z = (float)raw_gz / 16.384f;

		xQueueSend(IMUQueue, &output, 1);
		osDelay(1);
	}
}

/*
 *  Helper function to read multiple bytes of data from magnetometer SPI line
 */
void Mag_ReadSPI(uint16_t chip, uint8_t reg, uint8_t* buffer, uint8_t length)
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
 *  Helper function to write to a specific register on magnetometer SPI line
 */
void Mag_WriteReg(uint8_t reg, uint8_t data)
{
	uint8_t tx[2];
	tx[0] = reg & 0x7F;   // write mode
	tx[1] = data;

	osMutexAcquire(spiMutex, osWaitForever);

	HAL_GPIO_WritePin(GPIOC, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, SPI1_CS2_Pin, GPIO_PIN_SET);

	osMutexRelease(spiMutex);
}

/*
 *  Helper function to read from a specific register on magnetometer SPI line
 */
uint8_t Mag_ReadReg(uint8_t reg)
{
	uint8_t tx[2];
	uint8_t rx[2];

	tx[0] = reg | 0xC0;  // 0x80 (read) + 0x40 (auto-increment)
	tx[1] = 0x00;

	osMutexAcquire(spiMutex, osWaitForever);

	HAL_GPIO_WritePin(GPIOC, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, SPI1_CS2_Pin, GPIO_PIN_SET);

	osMutexRelease(spiMutex);

	return rx[1];
}

/*
 *  Initialize the settings of the Magnetometer to prepare for active reading
 *  	who_am_i = 0x30
 */
bool Mag_Init(void)
{
	Mag_WriteReg(0x09, 0x10); // set magnetometer
	osDelay(5);
	Mag_WriteReg(0x09, 0x20); // set automatic set/reset
	return (Mag_ReadReg(0x2F) == 0x30); // return ID check to return online/offline
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
		output.head_x = SENSERROR;
		output.head_y = SENSERROR;
		output.head_z = SENSERROR;
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
		Mag_WriteReg(0x09, 0x08);
		while (!(Mag_ReadReg(0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		// 2. MEASURE (after SET)
		timeout = tim_rst;
		Mag_WriteReg(0x09, 0x01);
		while (!(Mag_ReadReg(0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		Mag_ReadSPI(SPI1_CS2_Pin, 0x00, set_buffer, 7);

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
		Mag_WriteReg(0x09, 0x10);
		timeout = tim_rst;
		while (!(Mag_ReadReg(0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		// 4. MEASURE
		Mag_WriteReg(0x09, 0x01);
		timeout = tim_rst;
		while (!(Mag_ReadReg(0x08) & 0x01))
		{
			osDelay(1);
			if (--timeout == 0) break;
		}

		Mag_ReadSPI(SPI1_CS2_Pin, 0x00, buffer, 7); // read all 7 registers of data (0x00 - 0x06)

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

static bool Bar30_SendCmd(uint8_t cmd)
{
	osMutexAcquire(i2cMutex, osWaitForever);
	bool ok = HAL_I2C_Master_Transmit(&hi2c1, BAR30ADDR, &cmd, 1, 50) == HAL_OK;
	osMutexRelease(i2cMutex);
	return ok;
}

/*
 *  CRC-4 check, exactly as given in the datasheet (Figure 10 C code example)
 */
static uint8_t Bar30_CRC4(uint16_t prom[])
{
    uint16_t work[8];
    for (int i = 0; i < 7; i++) work[i] = prom[i];
    work[7] = 0;

    uint16_t rem = 0;
    work[0] = work[0] & 0x0FFF;   // zero out CRC nibble

    for (int cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1) rem ^= (work[cnt >> 1] & 0x00FF);
        else              rem ^= (work[cnt >> 1] >> 8);

        for (int n_bit = 8; n_bit > 0; n_bit--)
            rem = (rem & 0x8000) ? (rem << 1) ^ 0x3000 : (rem << 1);
    }
    return (rem >> 12) & 0x000F;
}

/*
 *  Read a 16-bit PROM word at address 0..6
 */
bool Bar30_ReadProm(uint8_t addr, uint16_t *out)
{
    uint8_t cmd = 0xA0 | (addr << 1);
    uint8_t buffer[2];

    osMutexAcquire(i2cMutex, osWaitForever);
    bool tx_ok = HAL_I2C_Master_Transmit(&hi2c1, BAR30ADDR, &cmd, 1, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!tx_ok) return false;

    osMutexAcquire(i2cMutex, osWaitForever);
    bool rx_ok = HAL_I2C_Master_Receive(&hi2c1, BAR30ADDR, buffer, 2, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!rx_ok) return false;

    *out = ((uint16_t)buffer[0] << 8) | buffer[1];
    return true;
}

/*
 *  Trigger a conversion and read back the 24-bit ADC result.
 *  The datasheet is explicit: do NOT read ADC during conversion — result
 *  will be 0 and the conversion is NOT stopped (Conversion Sequence section).
 */
bool Bar30_ReadAdc(uint8_t conv_cmd, uint32_t *out)
{
    uint8_t read_cmd = 0x00;
    uint8_t buffer[3];

    // Send conversion command then release — other tasks can use I2C during wait
    osMutexAcquire(i2cMutex, osWaitForever);
    bool cmd_ok = HAL_I2C_Master_Transmit(&hi2c1, BAR30ADDR, &conv_cmd, 1, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!cmd_ok) return false;

    osDelay(10);   // conversion wait — bus is free here

    osMutexAcquire(i2cMutex, osWaitForever);
    bool tx_ok = HAL_I2C_Master_Transmit(&hi2c1, BAR30ADDR, &read_cmd, 1, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!tx_ok) return false;

    osMutexAcquire(i2cMutex, osWaitForever);
    bool rx_ok = HAL_I2C_Master_Receive(&hi2c1, BAR30ADDR, buffer, 3, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!rx_ok) return false;

    *out = ((uint32_t)buffer[0] << 16) |
           ((uint32_t)buffer[1] <<  8) |
            (uint32_t)buffer[2];
    return true;
}

/*
 *  Reset the sensor, read and CRC-validate the 6 PROM calibration
 *  coefficients (C1..C6). Must succeed before any measurement is valid.
 *  (Factory Calibration + PROM Read Sequence sections)
 */
bool Bar30_Init(void)
{
    // Reset loads calibration PROM into internal registers (Reset Sequence)
    if (!Bar30_SendCmd(0x1E)) return false;
    osDelay(10);   // allow reset + PROM reload to complete

    // Read all 7 PROM words
    for (uint8_t i = 0; i <= 6; i++)
    {
        if (!Bar30_ReadProm(i, &bar30_prom[i]))
            return false;
    }

    // Validate CRC-4 in bits [15:12] of W0 (Figure 10, CRC section)
    uint8_t crc_read      = (bar30_prom[0] >> 12) & 0x0F;
    uint8_t crc_calculated = Bar30_CRC4(bar30_prom);
    if (crc_read != crc_calculated)
        return false;

    return true;
}

/*
 *  Read D1 (pressure) and D2 (temperature), apply full second-order
 *  compensation per Figure 9 (first order) and Figure 10 (second order).
 *
 *  Output pressure in mbar, temperature in °C.
 *
 *  Key datasheet intermediate types (Figure 9):
 *    dT, TEMP          → signed int32
 *    OFF, SENS         → signed int64  (intermediate products exceed 32-bit)
 *    P                 → signed int32
 */
void Bar30_Task(void *argument)
{
    Bar30Com_t output = {0};

    if (!Bar30_Init())
    {
        output.pressure   = SENSERROR;
        output.water_temp = SENSERROR;
        for (;;)
        {
            xQueueSend(Bar30Queue, &output, portMAX_DELAY);
            osDelay(1);
        }
    }

    osDelay(INITPAUSE);

    for (;;)
    {
        uint32_t D1 = 0, D2 = 0;

        if (!Bar30_ReadAdc(0x48, &D1)) goto retry;
        if (!Bar30_ReadAdc(0x58, &D2)) goto retry;

        {
            // Unpack calibration coefficients (Figure 9 notation)
            uint16_t C1 = bar30_prom[1];   // Pressure sensitivity
            uint16_t C2 = bar30_prom[2];   // Pressure offset
            uint16_t C3 = bar30_prom[3];   // Temp. coeff. of pressure sensitivity
            uint16_t C4 = bar30_prom[4];   // Temp. coeff. of pressure offset
            uint16_t C5 = bar30_prom[5];   // Reference temperature
            uint16_t C6 = bar30_prom[6];   // Temp. coeff. of temperature

            // ── First-order compensation (Figure 9) ──────────────────────
            // dT = D2 - C5 * 2^8
            int32_t  dT   = (int32_t)D2 - ((int32_t)C5 << 8);

            // TEMP = 2000 + dT * C6 / 2^23   (units: 0.01°C)
            int32_t  TEMP = 2000 + (int32_t)(((int64_t)dT * C6) >> 23);

            // OFF  = C2 * 2^16 + (C4 * dT) / 2^7
            int64_t  OFF  = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);

            // SENS = C1 * 2^15 + (C3 * dT) / 2^8
            int64_t  SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);

            // ── Second-order compensation (Figure 10) ─────────────────────
            int32_t Ti    = 0;
            int64_t OFFi  = 0;
            int64_t SENSi = 0;

            if (TEMP < 2000)
            {
                // Low temperature (< 20°C)
                Ti    = (int32_t)(3 * ((int64_t)dT * dT) >> 33);
                OFFi  = 1  * (int64_t)(TEMP - 2000) * (TEMP - 2000) >> 4;
                SENSi = 5  * (int64_t)(TEMP - 2000) * (TEMP - 2000) >> 3;

                if (TEMP < -1500)
                {
                    // Very low temperature (< -15°C)
                    OFFi  += 7  * (int64_t)(TEMP + 1500) * (TEMP + 1500);
                    SENSi += 4  * (int64_t)(TEMP + 1500) * (TEMP + 1500);
                }
            }
            else
            {
                // High temperature (>= 20°C)
                Ti    = (int32_t)(2 * ((int64_t)dT * dT) >> 37);
                OFFi  = (int64_t)(TEMP - 2000) * (TEMP - 2000) >> 4;
                SENSi = 0;
            }

            int64_t OFF2  = OFF  - OFFi;
            int64_t SENS2 = SENS - SENSi;
            int32_t TEMP2 = TEMP - Ti;

            // P2 = (D1 * SENS2 / 2^21 - OFF2) / 2^13   (units: 0.1 mbar per datasheet Fig 10)
            // Divide by 10 to get mbar
            int32_t P2 = (int32_t)((((int64_t)D1 * SENS2 >> 21) - OFF2) >> 13);

            output.water_temp = (float)TEMP2 / 100.0f;   // 0.01°C units → °C
            output.pressure   = (float)P2    / 10.0f;    // 0.1 mbar units → mbar
        }

        xQueueSend(Bar30Queue, &output, portMAX_DELAY);

retry:
        osDelay(100);
    }
}

/*
 *  Send a 16-bit command to the SHTC3 (MSB first)
 */
bool Humid_SendCmd(uint16_t cmd)
{
	uint8_t buffer[2] = { cmd >> 8, cmd & 0xFF };
	osMutexAcquire(i2cMutex, osWaitForever);
	bool ok = HAL_I2C_Master_Transmit(&hi2c1, SHTC3ADDR, buffer, 2, 50) == HAL_OK;
	osMutexRelease(i2cMutex);
	return ok;
}

/*
 *  CRC-8 verification:
 *    Polynomial 0x31, init 0xFF, no reflection, no final XOR
 */
uint8_t Humid_CRC8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

/*
 *  Initialize the settings of the Humidity sensor to prepare for active reading
 */
bool Humid_Init(void)
{
    uint8_t cmd[2] = { 0xEF, 0xC8 };
    uint8_t resp[3];

    if (!Humid_SendCmd(0x3517)) return false;  // already wrapped above
    osDelay(1);

    osMutexAcquire(i2cMutex, osWaitForever);
    bool tx_ok = HAL_I2C_Master_Transmit(&hi2c1, SHTC3ADDR, cmd, 2, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!tx_ok) return false;

    osMutexAcquire(i2cMutex, osWaitForever);
    bool rx_ok = HAL_I2C_Master_Receive(&hi2c1, SHTC3ADDR, resp, 3, 50) == HAL_OK;
    osMutexRelease(i2cMutex);
    if (!rx_ok) return false;

    if (Humid_CRC8(resp, 2) != resp[2]) return false;

    uint16_t id = ((uint16_t)resp[0] << 8) | resp[1];
    if ((id & 0x083F) != 0x0807) return false;

    Humid_SendCmd(0xB098);  // already wrapped above
    return true;
}

/*
 *  Task definition for continuous reading from Humidity sensor
 */
void Humid_Task(void *argument)
{
    HumidCom_t output = {0};
    uint8_t rx[6];

    if (!Humid_Init())
    {
        output.air_temp = SENSERROR;
        output.humidity = SENSERROR;
        for (;;)
        {
            xQueueSend(HumidQueue, &output, portMAX_DELAY);
            osDelay(1000);
        }
    }

    osDelay(INITPAUSE);

    for (;;)
    {
        bool ok = false;

        // 1. Wakeup
        if (!Humid_SendCmd(0x3517)) goto sleep_retry;
        osDelay(1);

        // 2. Trigger measurement (RH first, clock stretching disabled, normal mode)
        if (!Humid_SendCmd(0x58E0)) goto sleep_retry;
        osDelay(13);    // datasheet Table 5: max 12.1ms in normal mode, use 13ms margin

        // 3. Read 6 bytes
		osMutexAcquire(i2cMutex, osWaitForever);
		bool rx_ok = HAL_I2C_Master_Receive(&hi2c1, SHTC3ADDR, rx, 6, 50) == HAL_OK;
		osMutexRelease(i2cMutex);
		if (!rx_ok) goto sleep_retry;

        // 4. Validate both CRCs (Section 5.10)
        if (Humid_CRC8(&rx[0], 2) != rx[2]) goto sleep_retry;
        if (Humid_CRC8(&rx[3], 2) != rx[5]) goto sleep_retry;

        // 5. Convert raw values (Section 5.11) — declared BEFORE the goto target
        {
            uint16_t raw_rh = ((uint16_t)rx[0] << 8) | rx[1];
            uint16_t raw_t  = ((uint16_t)rx[3] << 8) | rx[4];

            output.humidity = 100.0f  * ((float)raw_rh / 65536.0f);
            output.air_temp = -45.0f  + 175.0f * ((float)raw_t / 65536.0f);
        }

        ok = true;
        xQueueSend(HumidQueue, &output, portMAX_DELAY);

sleep_retry:
        // Always send sleep command to minimise current draw (Table 3: ~0.3µA vs ~45µA idle)
        Humid_SendCmd(0xB098);

        // Only delay 1s on success; on failure, retry sooner
        osDelay(ok ? 1000 : 100);
    }
}

/*
 *  Initialize the settings of the Board temperature sensor to prepare for active reading
 */
bool Board_Init(void)
{
	uint32_t raw;

	// Run one conversion and check the ADC responds at all
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
		return false;

	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		HAL_ADC_Stop(&hadc1);
		return false;
	}

	raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// Sanity-check: MCP9700A output range is 100 mV to 1.75V over -40 to +125°C
	// In 12-bit ADC counts at 3.3V ref, that's roughly 124 to 2172.
	// A reading of 0 or full-scale (4095) suggests a wiring fault.
	if (raw == 0 || raw >= 4095)
		return false;

	return true;
}

/*
 *  Task definition for continuous reading from Board temperature sensor
 */
void Board_Task(void *argument)
{
	BoardCom_t output = {0};

	if (!Board_Init())
	{
		output.board_temp = SENSERROR;
		for (;;)
		{
			xQueueSend(BoardQueue, &output, portMAX_DELAY);
			osDelay(1);
		}
	}

	osDelay(INITPAUSE);

	for (;;)
	{
		if (HAL_ADC_Start(&hadc1) == HAL_OK && HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		{
			uint32_t raw  = HAL_ADC_GetValue(&hadc1);
			float    vout = ((float)raw / 4096.0) * 3.3;
			output.board_temp = (vout - 0.5) / 0.01;
		}
		HAL_ADC_Stop(&hadc1);

		xQueueSend(BoardQueue, &output, portMAX_DELAY);
		osDelay(1);
	}
}
