/*
 * protocol_interpreter.h
 *
 *  Created on: 29 août 2017
 *      Author: Patrick
 */

#ifndef APP_PROTOCOL_INTERPRETER_H_
#define APP_PROTOCOL_INTERPRETER_H_

#include "stm32f7xx_hal.h"
#include "serial.h"

extern uint8_t tx_protocol_buffer[128];
extern HAL_Serial_Handler com;

void APP_Protocol_Interpreter_Process();

#endif /* APP_PROTOCOL_INTERPRETER_H_ */
