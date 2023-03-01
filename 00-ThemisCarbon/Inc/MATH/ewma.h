/*
 * ewma.h
 *
 *  Created on: 2 févr. 2016
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_MATH_EWMA_H_
#define APPLICATION_USER_MATH_EWMA_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/

/* MATH Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

struct t_ewma_handler
{
    // settings
    float * alpha; // <1.0
    // state
    float mean;
};

typedef struct t_ewma_handler ewma_handler;

void reset_ewma( ewma_handler * handler );
void reset_ewma_with_non_zero_data( ewma_handler * handler, float initial_mean );

float process_ewma( ewma_handler * handler, float data );

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_MATH_EWMA_H_ */
