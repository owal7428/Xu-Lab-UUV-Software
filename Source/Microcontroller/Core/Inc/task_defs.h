/*
 * task_defs.h
 *
 *  Created on: 17 Mar 2026
 *      Author: leons
 */

#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "freertos.h"
#include "queue.h"
#include <math.h>
#include <stdbool.h>

#ifndef CORE_INC_TASK_DEFS_H_
#define CORE_INC_TASK_DEFS_H_

#define INITPAUSE 7000 // 7.0 s
#define PACKETSIZE 128 // 128 bytes



/* external definitions of connection ports */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;



/*
 *  Struct defenitions for communicating commands from the Raspberry Pi to the tasks
 */
typedef struct
{
	float amplitude;
	uint16_t speed;
	int16_t horz_offset;
	int16_t vert_offset;
} ServoCmd_t;

typedef struct
{
	int8_t thrust;
} ThrusterCmd_t;



/*
 *  Struct defenitions for communicating task data to the Raspberry Pi communication task
 */
typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} IMUCom_t;

typedef struct
{
	float head_x;
	float head_y;
	float head_z;
} MagCom_t;

typedef struct
{
	float pressure;
	float water_temp;
} Bar30Com_t;

typedef struct
{
	float humidity;
	float air_temp;
} HumidCom_t;

typedef struct
{
	float board_temp;
} BoardCom_t;



/*
 *  Struct defenition for assembling communication packet out to the Raspberry Pi
 */
typedef struct
{
	uint8_t SoP[2]; // header marking start of packet for alignment
	uint8_t pad0[2]; // already automatically added for memory padding
	IMUCom_t IMUCom;
	MagCom_t MagCom;
	Bar30Com_t Bar30Com;
	HumidCom_t HumidCom;
	BoardCom_t BoardCom;
	uint8_t CheckSum; // for detecting corruption
	uint8_t pad1; // already automatically added for memory padding
} OutPacket_t;



/*
 *  Struct defenitions for interpreting communication packets from the Raspberry Pi
 */
typedef struct
{
	uint8_t Sop[2]; // header marking start of packet for alignment
	ServoCmd_t ServoCmd[8];
	ThrusterCmd_t ThrusterCmd;
	uint8_t CheckSum;
} InPacket_t;



/* Helper struct for communicating to servos which channel and tim they are for initialization and ID */
typedef struct
{
    TIM_HandleTypeDef *tim;
    uint32_t channel;
} ServoParams_t;



/* Function definitions for tasks and helper functions */
void LED_Task(void* argument);

uint8_t CalculateChecksum(uint8_t* data, size_t length);
void PiCom_Task(void *argument);

uint16_t angle_to_pulse(int16_t angle);
void Servo_Init(TIM_HandleTypeDef* tim, uint32_t channel);
void Servo_Task(void* argument);

uint16_t percent_to_pulse(int8_t percent);
void Thruster_Init(void);
void Thruster_Task(void* arugment);

void SPI_WriteReg(uint8_t line, uint8_t chip, uint8_t reg, uint8_t data);
uint8_t SPI_ReadReg(uint8_t line, uint8_t chip, uint8_t reg);
void SPI_Read(uint8_t line, uint8_t chip, uint8_t reg, uint8_t* buffer, uint8_t length);

bool IMU_Init(void);
void IMU_Task(void *argument);

bool Mag_Init(void);
void Mag_Task(void *argument);

void Bar30_Init(void);
void Bar30_Task(void* argument);

bool Humid_Init(void);
void Humid_Task(void *argument);

bool Board_Init(void);
void Board_Task(void *argument);

#endif /* CORE_INC_TASK_DEFS_H_ */
