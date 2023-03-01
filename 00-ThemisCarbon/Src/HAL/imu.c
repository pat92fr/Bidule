/*
 * imu.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include <math.h>
#include "imu.h"
#include "stm32f7xx_hal.h"

/// ADD to linker : -specs=rdimon.specs
/// ADD to linker : -specs=rdimon.specs
/// ADD to linker : -specs=rdimon.specs

extern I2C_HandleTypeDef hi2c1;

/*****************************************************************************/

//#define M_PI 3.141592f
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ToRad(x) ((x) * 0.01745329252)  // *pi/180.0
#define ToDeg(x) ((x) * 57.2957795131)  // *180.0/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

float fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

/*****************************************************************************/
static float pitch,roll,yaw;

float HAL_Imu_Pitch()
{
	return pitch;
}

float HAL_Imu_Roll()
{
	return roll;
}

float HAL_Imu_Yaw()
{
	return yaw;
}

float HAL_Imu_Angle(int angle)
{
	switch(angle)
	{
	case IMU_ANGLE_PITCH:
		return pitch;
	case IMU_ANGLE_ROLL:
		return roll;
	case IMU_ANGLE_YAW:
		return yaw;

	}
	return 0.0;
}

/*****************************************************************************/

enum IMU_ADDR
{
	IMU_LSM6DS33_ADDR = 0xD6, // Acc+Gyro
	IMU_LIS3MDL_ADDR = 0x3C // Mag
};

enum IMU_WHO_AM_I
{
	IMU_LSM6DS33_WHO_AM_I = 0x69, // Gyro
	IMU_LIS3MDL_WHO_AM_I = 0x3D // Acc
};

enum IMU_LSM6DS33_REGISTER
{
	IMU_LSM6DS33_REGISTER_WHO_AM_I 		= 0x0F,
	IMU_LSM6DS33_REGISTER_CTRL1         = 0x10,
	IMU_LSM6DS33_REGISTER_CTRL2         = 0x11,
	IMU_LSM6DS33_REGISTER_CTRL3         = 0x12,
	IMU_LSM6DS33_REGISTER_CTRL4         = 0x13,
	IMU_LSM6DS33_REGISTER_CTRL5         = 0x14,
	IMU_LSM6DS33_REGISTER_CTRL6         = 0x15,
	IMU_LSM6DS33_REGISTER_CTRL7         = 0x16,
	IMU_LSM6DS33_REGISTER_CTRL8         = 0x17,
	IMU_LSM6DS33_REGISTER_CTRL9         = 0x18,
	IMU_LSM6DS33_REGISTER_CTRL10        = 0x19,
	IMU_LSM6DS33_REGISTER_STATUS        = 0x1E,
	IMU_LSM6DS33_REGISTER_OUT_TEMP_L    = 0x20,
	IMU_LSM6DS33_REGISTER_OUTX_L_G      = 0x22,
	IMU_LSM6DS33_REGISTER_OUTX_L_XL     = 0x28
};

enum IMU_LIS3MDL_REGISTER
{
	IMU_LIS3MDL_REGISTER_WHO_AM_I 		= 0x0F,
	IMU_LIS3MDL_REGISTER_CTRL1  		= 0x20,
	IMU_LIS3MDL_REGISTER_CTRL2  		= 0x21,
	IMU_LIS3MDL_REGISTER_CTRL3  		= 0x22,
	IMU_LIS3MDL_REGISTER_CTRL4  		= 0x23,
	IMU_LIS3MDL_REGISTER_CTRL5  		= 0x24,
	IMU_LIS3MDL_REGISTER_STATUS  		= 0x27,
	IMU_LIS3MDL_REGISTER_OUT_X_L  		= 0x28,
	IMU_LIS3MDL_REGISTER_TEMP_OUT_L     = 0x2E
};

struct t_imu_config_element
{
	int addr;
	int reg;
	uint8_t value;
};

struct t_imu_config_element imu_config[] = {
		//{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL3,0b1000000},   // reboot
		//{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL3,0b0000001},   // soft reset

		{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL9,0x38},    // ACC enabled (not necessary)
		{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL10,0x38},   // GYR enabled (not necessary)
		{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL1,0x52},   // ACC: ODR=208Hz, ±2g, 100Hz Antialiasing
		{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL2,0x52},   // GYR: ODR=208Hz, 125dps
		{IMU_LSM6DS33_ADDR,IMU_LSM6DS33_REGISTER_CTRL3,0x44},   // ACC+GYR: BDU enabled, Auto increment

		{IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_CTRL1,0b01110000},   // MAG: UHP X & Y, 10Hz,
		{IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_CTRL2,0b00000000},   // MAG: ± 4 gauss
		{IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_CTRL3,0b00000000},   // MAG: Continuous conversion mode
		{IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_CTRL4,0b00001100}    // MAG: UHP Z
};

/*****************************************************************************/

static int16_t raw_sensor_data[SENSOR_COUNT] = {0,0,0,0,0,0,0,0,0};
static int16_t raw_sensor_data_min[SENSOR_COUNT] = {0,0,0,0,0,0,-3551,-5064,1414}; // themis2
static int16_t raw_sensor_data_max[SENSOR_COUNT] = {0,0,0,0,0,0,1631,1460,6137}; // themis2
static int16_t raw_sensor_data_offset[SENSOR_COUNT] = {-245,-2000,-379,0,0,0,-960,-1802,3775}; // themis2 with mean
//static int16_t raw_sensor_data_min[SENSOR_COUNT] = {0,0,0,0,0,0,-5735,5365,6074}; // Antichavi
//static int16_t raw_sensor_data_max[SENSOR_COUNT] = {0,0,0,0,0,0,4462,5430,4195}; // Antichavi
//static int16_t raw_sensor_data_offset[SENSOR_COUNT] = {0,0,0,0,0,0,0,0,0}; // Antichavi

int16_t * HAL_Imu_Raw_Sensor_Data(void)
{
	return raw_sensor_data;
}

int16_t * HAL_Imu_Raw_Sensor_Data_Offset(void)
{
	return raw_sensor_data_offset;
}

/*****************************************************************************/

static float scaled_sensor_data[SENSOR_COUNT];

float * HAL_Imu_Scaled_Sensor_Data(void)
{
	return scaled_sensor_data;
}

static const float sensor_scale_factor[SENSOR_COUNT] = {
		125.0/32768.0*M_PI/180.0, // ± 125dps 16bits => rad/s
		125.0/32768.0*M_PI/180.0, // ± 125dps => rad/s
		125.0/32768.0*M_PI/180.0, // ± 125dps => rad/s
		2.0/32768.0, // ± 2g 16bits
		2.0/32768.0, // ± 2g
		2.0/32768.0, // ± 2g
		4.0/32768.0, // ± 4gauss 16bits
		4.0/32768.0, // ± 4gauss
		4.0/32768.0  // ± 4gauss
};
static const float  sensor_sign[SENSOR_COUNT] = {1,1,1,1,1,1,1,1,1};
static const int  sensor_order[SENSOR_COUNT] = {0,1,2,3,4,5,6,7,8};

void scale(void)
{
	for(short index=0;index<SENSOR_COUNT;++index)
	{
		scaled_sensor_data[sensor_order[index]]=(float)(raw_sensor_data[index]-raw_sensor_data_offset[index])*sensor_scale_factor[index]*sensor_sign[index];
	}
}

/*****************************************************************************/
static uint32_t calibration_iteration = 0;
#define CALIBRATION_ITERATION_MAX 1000
static int16_t raw_sensor_data_record[CALIBRATION_ITERATION_MAX][3];
static const int16_t calibration_default_value = 12000;

void HAL_Imu_Begin_Gyr_Calibration()
{

}

void HAL_Imu_Do_Gyr_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_record[calibration_iteration][index]=raw_sensor_data[GYR_X+index];
	}
}

void HAL_Imu_End_Gyr_Calibration(void)
{
	float mean[3] = {0.0,0.0,0.0};
	for(short index=0;index<3;++index)
	{
		for(short iteration=0;iteration<CALIBRATION_ITERATION_MAX;++iteration)
		{
			mean[index] = mean[index] + (float)raw_sensor_data_record[iteration][index];
		}
		raw_sensor_data_offset[GYR_X+index]= (int32_t)(mean[index]/1000.0);
	}
}

void HAL_Imu_Begin_Mag_Calibration()
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_min[MAG_X+index]=calibration_default_value;
		raw_sensor_data_max[MAG_X+index]=-calibration_default_value;
	}
}

void HAL_Imu_Do_Mag_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_min[MAG_X+index]= MIN(raw_sensor_data_min[MAG_X+index],raw_sensor_data[MAG_X+index]);
		raw_sensor_data_max[MAG_X+index]= MAX(raw_sensor_data_max[MAG_X+index],raw_sensor_data[MAG_X+index]);
	}
}

void HAL_Imu_End_Mag_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_offset[MAG_X+index]= (raw_sensor_data_max[MAG_X+index]+raw_sensor_data_min[MAG_X+index])/2;
	}
}

/*****************************************************************************/

void HAL_Imu_Process_Failure(void)
{
	while(1);
}

void HAL_Imu_Read_Sensors(void)
{
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t donnee = 0x5A;

	/// query GYR and ACC
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM6DS33_ADDR, IMU_LSM6DS33_REGISTER_STATUS, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		HAL_Imu_Process_Failure();
	}
//	if((donnee&0x04)!=0) // TEMP new data set available.
//	{
//		result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM6DS33_ADDR, IMU_LSM6DS33_REGISTER_OUT_TEMP_L | (1<<7), I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[GYR_X], 6, 10);
//		if(result!=HAL_OK)
//		{
//			HAL_Imu_Process_Failure();
//		}
//	}
	if((donnee&0x02)!=0) // GYR: X, Y, Z -axis new data set available.
	{
		result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM6DS33_ADDR, IMU_LSM6DS33_REGISTER_OUTX_L_G , I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[GYR_X], 6, 10);
		if(result!=HAL_OK)
		{
			HAL_Imu_Process_Failure();
		}
	}
	if((donnee&0x01)!=0) // ACC: X, Y, Z -axis new data set available.
	{
		result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM6DS33_ADDR, IMU_LSM6DS33_REGISTER_OUTX_L_XL , I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[ACC_X], 6, 10);
		if(result!=HAL_OK)
		{
			HAL_Imu_Process_Failure();
		}
	}
	/// query MAG
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_STATUS, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		HAL_Imu_Process_Failure();
	}
	if((donnee&0x08)!=0) // MAG: X, Y, Z -axis new data set available.
	{
		result = HAL_I2C_Mem_Read(&hi2c1, IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_OUT_X_L | (1<<7), I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[MAG_X], 6, 10);
		if(result!=HAL_OK)
		{
			HAL_Imu_Process_Failure();
		}
	}
	scale();
}

/*****************************************************************************/
enum imu_states
{
	STATE_IMU_NORMAL,
	STATE_IMU_GYR_DRIFT_CALIBRATION_START,
	STATE_IMU_GYR_DRIFT_CALIBRATION,
	STATE_IMU_MAG_CALIBRATION_START,
	STATE_IMU_MAG_CALIBRATION
};

static uint32_t state = STATE_IMU_NORMAL;


int32_t HAL_Imu_Get_State(void)
{
	return state;
}

void HAL_Imu_Gyr_Drift_Calibration(void)
{
	if(state == STATE_IMU_NORMAL)
	{
		state = STATE_IMU_GYR_DRIFT_CALIBRATION_START;
	}

}

void HAL_Imu_Mag_Calibration(void)
{
	if(state == STATE_IMU_NORMAL)
	{
		state = STATE_IMU_MAG_CALIBRATION_START;
	}

}

/*****************************************************************************/

void HAL_Imu_Init(void)
{
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t donnee = 0x5A;

	// wait for IMU boot and check presence/identification
	HAL_Delay(50);
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM6DS33_ADDR, IMU_LSM6DS33_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
	if(result!=HAL_OK)
	{
		do
		{
			HAL_Delay(500);
			result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM6DS33_ADDR, IMU_LSM6DS33_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
		}
		while(result!=HAL_OK);
	}
	if(donnee!=IMU_LSM6DS33_WHO_AM_I)
		while(1);
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		do
		{
			HAL_Delay(500);
			result = HAL_I2C_Mem_Read(&hi2c1, IMU_LIS3MDL_ADDR, IMU_LIS3MDL_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
		}
		while(result!=HAL_OK);
	}
	if(donnee!=IMU_LIS3MDL_WHO_AM_I)
		while(1);

	/// configure
	int index = 0;
	for(index=0;index<sizeof(imu_config)/sizeof(struct t_imu_config_element);++index)
	{
		result = HAL_I2C_Mem_Write(&hi2c1, imu_config[index].addr, imu_config[index].reg, I2C_MEMADD_SIZE_8BIT, &imu_config[index].value, 1, 100);
		if(result!=HAL_OK)
			while(1);
	}
	for(index=0;index<sizeof(imu_config)/sizeof(struct t_imu_config_element);++index)
	{
		result = HAL_I2C_Mem_Read(&hi2c1, imu_config[index].addr, imu_config[index].reg, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
		if(result!=HAL_OK)
			while(1);
		if(donnee != imu_config[index].value)
			while(1);
	}

	//calculate the initial quaternion
	HAL_Imu_Read_Sensors();
	//these are rough values. This calibration works a lot better if the device is kept as flat as possible
	//find the initial pitch and roll
	pitch = -ToDeg(fastAtan2(scaled_sensor_data[ACC_X],sqrt(scaled_sensor_data[ACC_Y] * scaled_sensor_data[ACC_Y] + scaled_sensor_data[ACC_Z] * scaled_sensor_data[ACC_Z]))); // Note : added - for themis ACC positioning
	roll = ToDeg(fastAtan2(-1*scaled_sensor_data[ACC_Y],sqrt(scaled_sensor_data[ACC_X] * scaled_sensor_data[ACC_X] + scaled_sensor_data[ACC_Z] * scaled_sensor_data[ACC_Z])));

//	if(scaled_sensor_data[ACC_Z] > 0)
//	{
//		if(scaled_sensor_data[ACC_X] > 0)
//		{
//			pitch = 180.0 - pitch;
//		}
//		else
//		{
//			pitch = -180.0 - pitch;
//		}
//		if(scaled_sensor_data[ACC_Y] > 0)
//		{
//			roll = -180.0 - roll;
//		}
//		else
//		{
//			roll = 180.0 - roll;
//		}
//	}

	float floatMagX = scaled_sensor_data[MAG_X];
	float floatMagY = scaled_sensor_data[MAG_Y];
	float floatMagZ = scaled_sensor_data[MAG_Z];
	//tilt compensate the compass
	float xMag = (floatMagX * cos(ToRad(pitch))) + (floatMagZ * sin(ToRad(pitch)));
	float yMag = -1 * ((floatMagX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))));

	yaw = ToDeg(fastAtan2(yMag,xMag));

	if(yaw < 0)
	{
		yaw += 360;
	}
}

void HAL_Imu_Process(void)
{
	HAL_Imu_Read_Sensors();
	//static uint32_t last_time = 0;
	//uint32_t current_time = HAL_GetTick();
	//if(current_time>=last_time+5) // 200Hz
	{
		//last_time = current_time;
		switch(state)
		{
		case STATE_IMU_NORMAL:
			{
				static float const pitch_complementary_filter_alpha = 0.006; // 1/200Hz = 0.005
				float pitch_from_acc = -ToDeg(fastAtan2(scaled_sensor_data[ACC_X],sqrt(scaled_sensor_data[ACC_Y] * scaled_sensor_data[ACC_Y] + scaled_sensor_data[ACC_Z] * scaled_sensor_data[ACC_Z]))); // Note : added - for themis ACC positioning
				pitch = (1.0-pitch_complementary_filter_alpha)*(pitch+HAL_Imu_Scaled_Sensor_Data()[GYR_Y]*180.0/M_PI*0.005) + pitch_complementary_filter_alpha*pitch_from_acc;
			}
			break;
		case STATE_IMU_GYR_DRIFT_CALIBRATION_START:
			{
				HAL_Imu_Begin_Gyr_Calibration();
				calibration_iteration = 0;
				state = STATE_IMU_GYR_DRIFT_CALIBRATION;
			}
			break;
		case STATE_IMU_GYR_DRIFT_CALIBRATION:
			{
				HAL_Imu_Do_Gyr_Calibration();
				++calibration_iteration;
				if(calibration_iteration>=CALIBRATION_ITERATION_MAX)
				{
					HAL_Imu_End_Gyr_Calibration();
					state = STATE_IMU_NORMAL;
				}
			}
			break;
		case STATE_IMU_MAG_CALIBRATION_START:
			{
				HAL_Imu_Begin_Mag_Calibration();
				calibration_iteration = 0;
				state = STATE_IMU_MAG_CALIBRATION;
			}
			break;
		case STATE_IMU_MAG_CALIBRATION:
			{
				HAL_Imu_Do_Mag_Calibration();
				++calibration_iteration;
				if(calibration_iteration>=16000)
				{
					HAL_Imu_End_Mag_Calibration();
					state = STATE_IMU_NORMAL;
				}
			}
			break;
		}
	}
}
