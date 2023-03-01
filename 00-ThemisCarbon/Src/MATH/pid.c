#include "pid.h"
#include "mymath.h"
#include "stdbool.h"

void reset_pid( pid_handler * handler )
{
    handler->old_error = 0.0;
    handler->old_dterm = 0.0;
    handler->integral_error = 0.0;
    handler->condition = 0;
}

float process_pid( pid_handler * handler, float error )
{
    // calculate every terms
    float pterm = error**handler->kp;
    float iterm = handler->integral_error; // do not integrate right now, update of integration depends on new condition
    //float dterm = (error-handler->old_error)**handler->kd;
    float dterm = (1.0-handler->alpha)*handler->old_dterm + handler->alpha*(error-handler->old_error)**handler->kd;
	// sum up every terms and clamp the output
    float output = pterm + iterm + dterm;
	float output_clamped =  fconstrain(output,handler->min,handler->max);
    // detect actuator saturation
	bool actuator_staturation = (output!=output_clamped);
	// detect error and output signs
	bool same_error_output_sign = ((output*error) > 0.0);
	// update this PID & calculate new condition
	handler->old_error = error;
	if(same_error_output_sign && actuator_staturation)
	{
		handler->condition = 1;
		// don t change integration (saturation detected and error same sign of output
	}
	else
	{
		handler->condition = 0;
		// update this PID according new condition
		handler->integral_error += (error**handler->ki);
	}
  	// end up the process without actuator saturation
   	return output_clamped;
}

void reset_pid_win( pid_win_handler * handler )
{
    handler->old_error = 0.0;
    handler->old_dterm = 0.0;
    for(int index=0;index<PID_WIN_SIZE;++index)
    	handler->integral_window[index] = 0.0;
    handler->integral_window_index = 0;
}

float process_pid_win( pid_win_handler * handler, float error )
{
    // calculate every terms
    float pterm = error**handler->kp;
    //float dterm = (error-handler->old_error)**handler->kd;
    float dterm = (1.0-handler->alpha)*handler->old_dterm + handler->alpha*(error-handler->old_error)**handler->kd;
    // window integral
    handler->integral_window[handler->integral_window_index] = error;
    ++handler->integral_window_index;
    if(handler->integral_window_index > handler->integral_window_size)
    	handler->integral_window_index = 0;
    float iterm = 0.0;
    for(int index=0;index<PID_WIN_SIZE;++index)
    	iterm+=handler->integral_window[index];
    iterm *= *handler->ki;
    // sum up every terms and clamp the output
    float output = pterm + iterm + dterm;
	float output_clamped =  fconstrain(output,handler->min,handler->max);
	// update this PID & calculate new condition
	handler->old_error = error;
	handler->old_dterm = dterm;
  	// end up the process without actuator saturation
   	return output_clamped;
}
