/*
 * imu.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_IMU_H_
#define APPLICATION_USER_HAL_IMU_H_

#include "stm32f7xx_hal.h"

void HAL_Imu_Init(void);
void HAL_Imu_Process(void);

float HAL_Imu_Pitch(void);
float HAL_Imu_Roll(void);
float HAL_Imu_Yaw(void);

enum
{
	IMU_ANGLE_PITCH,
	IMU_ANGLE_ROLL,
	IMU_ANGLE_YAW,
	IMU_ANGLE_COUNT
};

enum e_data
{
	GYR_X = 0,
	GYR_Y,
	GYR_Z,
	ACC_X,
	ACC_Y,
	ACC_Z,
	MAG_X,
	MAG_Y,
	MAG_Z,
	SENSOR_COUNT
};

float HAL_Imu_Angle(int angle);

int16_t * HAL_Imu_Raw_Sensor_Data_Offset(void);
int16_t * HAL_Imu_Raw_Sensor_Data(void);
float * HAL_Imu_Scaled_Sensor_Data(void);

void HAL_Imu_Gyr_Drift_Calibration(void);
void HAL_Imu_Mag_Calibration(void);

int32_t HAL_Imu_Get_State(void);

float fastAtan2( float y, float x);

#endif /* APPLICATION_USER_HAL_IMU_H_ */
