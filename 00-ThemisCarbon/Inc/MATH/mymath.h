/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_BASE_H
#define __MATH_BASE_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/

/* MATH Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

int32_t constrain(int32_t x, int32_t min, int32_t max);
float fconstrain(float x, float min, float max);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
float myfabs(float n);

#ifdef __cplusplus
}
#endif

#endif /* __MATH_BASE_H */




