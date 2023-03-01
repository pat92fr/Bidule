/*
 * stepper.h
 *
 *  Created on: 8 janv. 2017
 *      Author: Patrick
 */

#ifndef STEPPER_H_
#define STEPPER_H_

/* CubeMX ------------------------------------------------------------------*/

/*
 * TIM is 16-bit
 * TIM Prescaler has to be configured for 1MHz TIMER clock
 * TIM General IT has to be activated
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  HAL Phase structures definition
  */
typedef enum
{
	HAL_Stepper_Motor_CW  = 0x01U,
	HAL_Stepper_Motor_CCW = 0x00U,
} HAL_Stepper_Motor_RotationTypeDef;

/**
  * @brief  Stepper Motor Handler Structure definition
  */
typedef struct {
	GPIO_TypeDef* STEP_GPIOx;		/*!< Specifies the GPIO port used to generate DIR signal. */
	uint16_t STEP_GPIO_Pin;			/*!< Specifies the GPIO pin used to generate DIR signal. */
	GPIO_TypeDef* DIR_GPIOx;		/*!< Specifies the GPIO port used to generate DIR signal. */
	uint16_t DIR_GPIO_Pin;			/*!< Specifies the GPIO pin used to generate DIR signal. */

	uint32_t speed;		/*!< Specifies the target speed in SPS. */
	HAL_Stepper_Motor_RotationTypeDef rotation;	/*!< Specifies the current rotation CW/CCW. */

	uint32_t period;				/*!< Specifies the period in ticks. */
	uint32_t counter;				/*!< Specifies the actuel time in ticks. */

	int32_t position;			/*!< Specifies the absolute position (steps). */
} StepperMotor_HandleTypeDef;


#ifdef __cplusplus
 extern "C" {
#endif

 void HAL_Stepper_Motor_Init(
		 TIM_HandleTypeDef * htim
 );

void HAL_Stepper_Motor_Add(
 		StepperMotor_HandleTypeDef * hstep,
		GPIO_TypeDef* STEP_GPIOx,
		uint16_t STEP_GPIO_Pin,
		GPIO_TypeDef* DIR_GPIOx,
		uint16_t DIR_GPIO_Pin
);

void HAL_Stepper_Motor_Set_Speed(
		StepperMotor_HandleTypeDef * hstep,
		int32_t speed
);

int32_t HAL_Stepper_Motor_Get_Position(
		StepperMotor_HandleTypeDef * hstep
);

void HAL_Stepper_Motor_Reset_Position(
		StepperMotor_HandleTypeDef * hstep
);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H_ */
