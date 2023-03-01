/*
 * configuration.h
 *
 *  Created on: 27 août 2017
 *      Author: Patrick
 */

#ifndef APP_CONFIGURATION_H_
#define APP_CONFIGURATION_H_

#include "stm32f7xx_hal.h"

#define CONFIGURATION_FLASH_ADDR (uint32_t*)0x080C0000

typedef enum
{
	  IMU_PITCH_REF,
	  IMU_ROLL_REF,
	  IMU_YAW_REF,
	  IMU_SPARE1,
	  IMU_SPARE2,
	  IMU_SPARE3,

	  M_ACC,
	  M_SPEED,
	  M_FILTER,
	  M_SPARE1,
	  M_SPARE2,
	  M_SPARE3,

	  CTRL_PKP,
	  CTRL_PKI,
	  CTRL_PKD,

	  CTRL_YKP,
	  CTRL_YKI,
	  CTRL_YKD,

	  CTRL_XKP,
	  CTRL_XKI,
	  CTRL_XKD,

	  CTRL_XFILTER,
	  CTRL_YFILTER,
	  CTRL_PFILTER,

	  CONFIGURATION_DATA_COUNT
} CONFIGURATION_DATA_NAME;

extern float configuration_data[CONFIGURATION_DATA_COUNT];

void HAL_Configuration_Init(void);
void HAL_Configuration_Reload(void);
void HAL_Configuration_Save(void);

#endif /* APP_CONFIGURATION_H_ */
