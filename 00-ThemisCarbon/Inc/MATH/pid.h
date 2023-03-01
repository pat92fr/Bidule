/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_PID_H
#define __MATH_PID_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/

/* MATH Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

// PID with integral windup

struct t_pid_handler
{
    // settings
    float * kp;
    float * ki;
    float * kd;
    float min;
    float max;
    float alpha; // <1.0 derivative term
    // state
    float old_error;
    float old_dterm;
    float integral_error;
    uint32_t condition;
};

typedef struct t_pid_handler pid_handler;

void reset_pid( pid_handler * handler );

float process_pid( pid_handler * handler, float error );

// PID with integral window

#define PID_WIN_SIZE 128

struct t_pid_win_handler
{
    // settings
    float * kp;
    float * ki;
    float * kd;
    float min;
    float max;
    uint32_t integral_window_size;
    float alpha; // <1.0 derivative term
    // state
    float old_error;
    float old_dterm;
    float integral_window[PID_WIN_SIZE];
    uint32_t integral_window_index;
};

typedef struct t_pid_win_handler pid_win_handler;

void reset_pid_win( pid_win_handler * handler );

float process_pid_win( pid_win_handler * handler, float error );



#ifdef __cplusplus
}
#endif

#endif /* __MATH_PID_H */





