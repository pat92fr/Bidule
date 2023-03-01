/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "protocol.h"

/* Private data --------------------------------------------------------------*/

static unsigned char const protocol_som = 0xA5;
static unsigned char const protocol_eom = 0x5A;

/// Format : A5 CMD DATA DATA DATA ... DD 5A

unsigned char const command_length[CMD_COUNT] =
{
0, //CMD_HELLO,

0, //CMD_PING,
0, //CMD_PONG,

0, //CMD_RUNNING,
0, //CMD_HALTED,
0, //CMD_SHUTDOWN,
0, //CMD_BATT_OK,

0, //CMD_QUERY_VBATT,
4, //CMD_VBATT,

0, //CMD_START,
0, //CMD_STOP,
0, //CMD_FOWARD,
0, //CMD_FOWARD_LONG,
0, //CMD_BACKWARD,
0, //CMD_TURN_LEFT,
0, //CMD_TURN_RIGHT,

0,   //CMD_QUERY_CONFIGURATION,
4*21,   //    CMD_CONFIGURATION,
4*21,    //CMD_SET_CONFIGURATION,

4*7,    //CMD_TELEMETRY_TICK,

0 //    CMD_UNKNOW,
};

/* Functions --------------------------------------------------------------*/

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
    unsigned char length )
{
    if(length==command_length[command])
    {
        *buffer++=protocol_som;
        *buffer++=command;
        for(unsigned int i=0; i<length;++i)
            *buffer++=data[i];
        *buffer++=protocol_eom;
        return length+3;
    }
    else
        return 0;
}

enum E_PROTOCOL_DECODE_INTERNAL
{
    PROTOCOL_DECODE_IDLE,
    PROTOCOL_DECODE_SOM_DECODED,
    PROTOCOL_DECODE_CMD_DECODED,
    PROTOCOL_DECODE_DATA_DECODED
};

/// Decode a command
///  (in) A new received data byte
///  (out) Data buffer (PoD), implicit size of CMD
/// Return :
///   -1, if no CMD decoded
///   CMD identifier, if a valid CMD decoded
static unsigned char decoded_data_buffer[128];

unsigned char * protocol_decode_data()
{
    return decoded_data_buffer;
};

int protocol_decode( unsigned char input )
{
    static int state = PROTOCOL_DECODE_IDLE;
    static int current_cmd = 0;
    static int remaining_data_bytes = 0;
    static int current_data_byte = 0;
    switch(state)
    {
    case PROTOCOL_DECODE_IDLE:
        {
            if(input==protocol_som)
            {
                state = PROTOCOL_DECODE_SOM_DECODED;
            }
        }
        break;
    case PROTOCOL_DECODE_SOM_DECODED:
        {
            if(input<CMD_COUNT)
            {
                current_cmd = input;
                remaining_data_bytes = command_length[input];
                current_data_byte = 0;
                if(remaining_data_bytes==0)
                {
                    state = PROTOCOL_DECODE_DATA_DECODED;
                }
                else
                {
                    state = PROTOCOL_DECODE_CMD_DECODED;
                }
            }
            else
            {
                state = PROTOCOL_DECODE_IDLE;
            }
        }
        break;
    case PROTOCOL_DECODE_CMD_DECODED:
        {
            decoded_data_buffer[current_data_byte]=input;
            ++current_data_byte;
            if(current_data_byte==remaining_data_bytes)
            {
                state = PROTOCOL_DECODE_DATA_DECODED;
            }
        }
        break;
    case PROTOCOL_DECODE_DATA_DECODED:
        {
            if(input==protocol_eom)
            {
                state = PROTOCOL_DECODE_IDLE;
                return current_cmd;
            }
            else
            {
                state = PROTOCOL_DECODE_IDLE;
            }
        }
        break;
    }
    return -1;
}
