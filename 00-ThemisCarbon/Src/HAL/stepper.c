/*
 * stepper.c
 *
 *  Created on: 8 janv. 2017
 *      Author: Patrick
 */

#include "stepper.h"
#include "math.h"
#include "mymath.h"

/* Private variables ---------------------------------------------------------*/
#define TICK_FREQUENCY 250000.0f
#define TICK_PERIOD_MIN 5
#define TICK_PERIOD_MAX 500000
#define HAL_Stepper_Motor_max_handles 4

static TIM_HandleTypeDef * stepper_htim = 0; 	/*!< Specifies the timer used to generate STEP pulses, one for all motor. */
static StepperMotor_HandleTypeDef * HAL_Stepper_Motor_handles[HAL_Stepper_Motor_max_handles];
static uint32_t  HAL_Stepper_Motor_handles_count = 0;


/* Provate functions ----------------------------------------------------------*/

// from speed (step/s) to period (ticks)
uint32_t speed_to_period(uint32_t speed)
{
	uint32_t period = (uint32_t)(TICK_FREQUENCY / ((float)(speed)+1.0) );
	if(period>TICK_PERIOD_MAX)
		return TICK_PERIOD_MAX;
	else if(period<TICK_PERIOD_MIN)
		return TICK_PERIOD_MIN;
	else
		return period;
}


/* Public functions ----------------------------------------------------------*/

void HAL_Stepper_Motor_Init(
	TIM_HandleTypeDef *htim
)
{
	stepper_htim = htim;
	HAL_TIM_Base_Start_IT(stepper_htim);
}

void HAL_Stepper_Motor_Add(
 		StepperMotor_HandleTypeDef * hstep,
		GPIO_TypeDef* STEP_GPIOx,
		uint16_t STEP_GPIO_Pin,
		GPIO_TypeDef* DIR_GPIOx,
		uint16_t DIR_GPIO_Pin
)
{
	// set STEPPER handle
	hstep->STEP_GPIOx = STEP_GPIOx;
	hstep->STEP_GPIO_Pin = STEP_GPIO_Pin;
	hstep->DIR_GPIOx = DIR_GPIOx;
	hstep->DIR_GPIO_Pin = DIR_GPIO_Pin;

	// init STEPPER handle
	hstep->speed = 0U;
	hstep->rotation = HAL_Stepper_Motor_CW;
	hstep->period = TICK_PERIOD_MAX; 		// just reset to the lowest speed
	hstep->counter = 0U;
	hstep->position = 0;

	// finalize STEPPER handle

	// register STEPPER handle
	HAL_Stepper_Motor_handles[HAL_Stepper_Motor_handles_count++]=hstep;

	// configure DIR & STEP
	HAL_GPIO_WritePin(hstep->STEP_GPIOx,hstep->STEP_GPIO_Pin,GPIO_PIN_SET); // PullUp on Big Easy Driver
	HAL_GPIO_WritePin(hstep->DIR_GPIOx,hstep->DIR_GPIO_Pin,GPIO_PIN_RESET); // PullUp on Big Easy Driver

}

void HAL_Stepper_Motor_Set_Speed(
		StepperMotor_HandleTypeDef * hstep,
		int32_t speed
)
{
	if(speed >= 0)
	{
		hstep->speed = speed;
		hstep->rotation = HAL_Stepper_Motor_CW;
	}
	else
	{
		hstep->speed = -speed;
		hstep->rotation = HAL_Stepper_Motor_CCW;
	}
	// THEN, prepare TIMER
	hstep->period = speed_to_period(hstep->speed);
}

int32_t HAL_Stepper_Motor_Get_Position(
		StepperMotor_HandleTypeDef * hstep
)
{
	return hstep->position;
}

void HAL_Stepper_Motor_Reset_Position(
		StepperMotor_HandleTypeDef * hstep
)
{
	hstep->position = 0;
}


void process(StepperMotor_HandleTypeDef * hstep)
{
	// Rising EDGE of STEP
	// End of period ?
	if(hstep->counter>=hstep->period)
	{
		// Reset counter
		hstep->counter=0;
		// Rise
		HAL_GPIO_WritePin(hstep->STEP_GPIOx,hstep->STEP_GPIO_Pin,GPIO_PIN_SET); // PullUp on Big Easy Driver
		// Update position
		if(hstep->rotation == HAL_Stepper_Motor_CW)
			++hstep->position;
		else
			--hstep->position;
	}

	// Falling EDGE of STEP
	else if(hstep->counter==1)
	{
		// Fall
		HAL_GPIO_WritePin(hstep->STEP_GPIOx,hstep->STEP_GPIO_Pin,GPIO_PIN_RESET); // PullUp on Big Easy Driver
		// update DIR
		HAL_GPIO_WritePin(hstep->DIR_GPIOx,hstep->DIR_GPIO_Pin,hstep->rotation); // PullUp on Big Easy Driver
	}
	// INC counter
	++hstep->counter;
}

/*****************************************************************************/

// float target speed
// HAL_Stepper_Motor_RotationTypeDef target rotation

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(stepper_htim == htim)
	{
		// look for Stepper Motor Handle
		for(uint32_t index=0; index<HAL_Stepper_Motor_handles_count;++index)
		{
			StepperMotor_HandleTypeDef * hstep = HAL_Stepper_Motor_handles[index];
			process(hstep);
		}
	}
}

