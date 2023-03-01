/*
 * protocol_interpreter.c
 *
 *  Created on: 29 août 2017
 *      Author: Patrick
 */

#include "protocol_interpreter.h"
#include "serial.h"
#include "protocol.h"
#include "configuration.h"

static float tx_data_buffer[32];
uint8_t tx_protocol_buffer[128];
HAL_Serial_Handler com;

void APP_Protocol_Interpreter_Process()
{
	  int len = HAL_Serial_Available(&com);
	  if(len>0)
	  {
		  for(int index=0;index<len;++index)
		  {
			  int command = protocol_decode(HAL_Serial_GetChar(&com));
			  if(command>0)
			  {
				  int32_t const * ptr_data = (int32_t *)protocol_decode_data();
				  switch(command)
				  {
				  case CMD_PING:
					  {
							int enc_ret = protocol_encode(
									tx_protocol_buffer,
									CMD_PONG,
									0,
									0 );
							if(enc_ret>0)
								HAL_Serial_Write(&com, tx_protocol_buffer, enc_ret );
							HAL_Delay(1);
					  }
					  break;
				  case CMD_QUERY_CONFIGURATION:
					  {
						tx_data_buffer[0]=configuration_data[IMU_PITCH_REF];
						tx_data_buffer[1]=configuration_data[CTRL_PKP];
						tx_data_buffer[2]=configuration_data[CTRL_PKI];
						tx_data_buffer[3]=configuration_data[CTRL_PKD];
						tx_data_buffer[4]=configuration_data[CTRL_XKP];
						tx_data_buffer[5]=configuration_data[CTRL_XKI];
						tx_data_buffer[6]=configuration_data[CTRL_XKD];
						tx_data_buffer[7]=configuration_data[M_FILTER];
						tx_data_buffer[8]=configuration_data[CTRL_XFILTER];
						tx_data_buffer[9]=configuration_data[CTRL_YFILTER];
						int enc_ret = protocol_encode(
								tx_protocol_buffer,
								CMD_CONFIGURATION,
								(uint8_t const*)tx_data_buffer,
								4*21 );
						if(enc_ret>0)
							HAL_Serial_Write(&com, tx_protocol_buffer, enc_ret );
						HAL_Delay(3);
					  }
					  break;
				  case CMD_SET_CONFIGURATION:
					  {
						  float const * ptr_data_f = (float const *)ptr_data;
						  configuration_data[IMU_PITCH_REF] = ptr_data_f[0];
						  configuration_data[CTRL_PKP] = ptr_data_f[1];
						  configuration_data[CTRL_PKI] = ptr_data_f[2];
						  configuration_data[CTRL_PKD] = ptr_data_f[3];
						  configuration_data[CTRL_XKP] = ptr_data_f[4];
						  configuration_data[CTRL_XKI] = ptr_data_f[5];
						  configuration_data[CTRL_XKD] = ptr_data_f[6];
						  configuration_data[M_FILTER] = ptr_data_f[7];
						  configuration_data[CTRL_XFILTER] = ptr_data_f[8];
						  configuration_data[CTRL_YFILTER] = ptr_data_f[9];

						  HAL_Configuration_Save();
						  HAL_Configuration_Reload();

					  }
					  break;

				  }
			  }
		  }
	  }
}
