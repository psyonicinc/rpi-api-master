#ifndef HAND_H
#define HAND_H
#include "i2c-master-test.h"
#include "smoothing.h"
#include <stdint.h>

/*the following structure is a wrapper for common memory elements that need to be tracked
for low-level motion control of fingers*/
typedef struct motion_params_t
{
	float qd_set;	//the ultimate, final motor setpoint that we want to achieve
	float qd; 		//the intermediate tracking point. continuously variable, for smooth motion
	float q; 		//the finger position, measured by encoder
    float q_dot;    //finger velocity, updated periodically
	float k;		//position control (straight proportional) gain
	float gbx_ratio;	//gear ratio between motor shaft and DTC output/finger angle
}motion_params_t;

/*Parameters related to the energy-driven safety features that protect the motor from overheating when stalled*/
typedef struct safety_params_t
{
	float e_cost_cooled;	//alternative e cost heuristic that cools over time 
	float e_cost_thresh;	//threshold for energy above which the driver is to be disabled
	float e_cost;	//energy consumption heuristic 
	float abs_err_set; //the absolute value of the error between the current position and the ultimate setpoint.
	float setpoint_err_thresh; 	//setpoint error below which the motor is shut off
	uint8_t driver_disabled;	//flag indicating the disable status of a motor driver
	uint8_t cooldown_active_flag;
}safety_params_t;

/*Overall structure containing all the parameters required to perform motion control on a hand.*/
typedef struct hand_t
{
	motion_params_t mp[NUM_CHANNELS];
	safety_params_t sp[NUM_CHANNELS];
	smooth_mem_t 	sm[NUM_CHANNELS];
	float smoothing_time;	//configuration transistion time (grip period)
	float smoothing_jump_trigger; //threshold above which smoothing is engaged
	float motor_current_limit;
}hand_t;

extern volatile hand_t gl_hand;

void finger_pctl(hand_t * hand, float_format_i2c * in, float_format_i2c * out);

#endif
