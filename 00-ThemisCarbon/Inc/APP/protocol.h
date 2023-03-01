/*
 * protocol.h
 *
 *  Created on: 17 déc. 2016
 *      Author: Patrick
 */

#ifndef HAL_PROTOCOL_H_
#define HAL_PROTOCOL_H_


#ifdef __cplusplus
extern "C" {
#endif

/* HAL settings ------------------------------------------------------------------*/

enum E_PROTOCOL_CMD
{
    CMD_HELLO,

    CMD_PING,
    CMD_PONG,

    CMD_RUNNING,
    CMD_HALTED,
    CMD_POWER_SHUTDOWN,
    CMD_POWER_OK,

    CMD_QUERY_VBATT,
    CMD_VBATT,

    CMD_START,
    CMD_STOP,
    CMD_FORWARD,
    CMD_FORWARD_LONG,
    CMD_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,

    CMD_QUERY_CONFIGURATION,
    CMD_CONFIGURATION,
    CMD_SET_CONFIGURATION,

    CMD_TELEMETRY_TICK,

    CMD_UNKNOW,
    CMD_COUNT
};

/* HAL Functions ------------------------------------------------------------------*/

/// Encode a command
///  (in) Destination buffer
///  (in) CMD identifier
///  (in) Data buffer (PoD)
///  (in) Data length
/// Return :
///   CMD frame length, if CMD length matches buffer length,
///   0, if not
int protocol_encode(
    unsigned char * buffer,
    unsigned char command,
    unsigned char const * data,
    unsigned char length );

/// Decode a command
///  (in) A new received data byte
/// Return :
///   -1, if no CMD decoded
///   CMD identifier, if a valid CMD decoded
int protocol_decode(unsigned char input);

unsigned char * protocol_decode_data();

#ifdef __cplusplus
}
#endif

#endif /* HAL_PROTOCOL_H_ */
