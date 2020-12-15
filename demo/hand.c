#include "hand.h"

#define DEFAULT_E_COST_THRESH 10.f
#define MAX_POWER_CURRENT_THRESH	90.f
#define MID_POWER_CURRENT_THRESH	50.f

#define GBX_FINGER_RATIO_21P3		0.134497135f
#define GBX_THUMBROT_RATIO_21P3		0.53798854002f
#define GBX_FINGER_RATIO_32P49		0.0881744837f
#define GBX_THUMBROT_RATIO_32P49	0.35269793483f
/*
Initialized motor control parameters for global hand motion control structure. has extraneous data entries (specifically the ones for safety) because this is a straight
copy from the actual hand firmware
*/
volatile hand_t gl_hand = {
	.mp = {
		{.qd_set = 15.f, .qd = 15.f, .q = 0.f, .k = 3.f, .gbx_ratio = GBX_FINGER_RATIO_32P49, .q_dot = 0.f},	//index	
		{.qd_set = 15.f, .qd = 15.f, .q = 0.f, .k = 3.f, .gbx_ratio = GBX_FINGER_RATIO_32P49, .q_dot = 0.f},	//middle
		{.qd_set = 15.f, .qd = 15.f, .q = 0.f, .k = 3.f, .gbx_ratio = GBX_FINGER_RATIO_32P49, .q_dot = 0.f},	//ring
		{.qd_set = 15.f, .qd = 15.f, .q = 0.f, .k = 3.f, .gbx_ratio = GBX_FINGER_RATIO_32P49, .q_dot = 0.f},	//pinky
		{.qd_set = 15.f, .qd = 15.f, .q = 0.f, .k = 3.f, .gbx_ratio = GBX_FINGER_RATIO_32P49, .q_dot = 0.f},	//thumb flexor
		{.qd_set = -15.f, .qd = -15.f, .q = 0.f, .k = 3.f, .gbx_ratio = GBX_THUMBROT_RATIO_32P49, .q_dot = 0.f}	//thumb rotator
	},
	.sp = {
		{.cooldown_active_flag = 0, .e_cost_cooled = 0, .e_cost_thresh = DEFAULT_E_COST_THRESH, .e_cost = 0.f, .abs_err_set = 0.f, .setpoint_err_thresh = 1.5f, .driver_disabled = 0},	//index
		{.cooldown_active_flag = 0, .e_cost_cooled = 0, .e_cost_thresh = DEFAULT_E_COST_THRESH, .e_cost = 0.f, .abs_err_set = 0.f, .setpoint_err_thresh = 1.5f, .driver_disabled = 0},	//middle
		{.cooldown_active_flag = 0, .e_cost_cooled = 0, .e_cost_thresh = DEFAULT_E_COST_THRESH, .e_cost = 0.f, .abs_err_set = 0.f, .setpoint_err_thresh = 1.5f, .driver_disabled = 0},	//ring
		{.cooldown_active_flag = 0, .e_cost_cooled = 0, .e_cost_thresh = DEFAULT_E_COST_THRESH, .e_cost = 0.f, .abs_err_set = 0.f, .setpoint_err_thresh = 1.5f, .driver_disabled = 0},	//pinky
		{.cooldown_active_flag = 0, .e_cost_cooled = 0, .e_cost_thresh = DEFAULT_E_COST_THRESH, .e_cost = 0.f, .abs_err_set = 0.f, .setpoint_err_thresh = 1.5f, .driver_disabled = 0},	//thumb flexor
		{.cooldown_active_flag = 0, .e_cost_cooled = 0, .e_cost_thresh = DEFAULT_E_COST_THRESH, .e_cost = 0.f, .abs_err_set = 0.f, .setpoint_err_thresh = -1.0f, .driver_disabled = 0}	//thumb rotator
	},
	.sm = {
		{.freq = 0.001f, .offset = 0.0f, .prev_fp = 100.0f, .qd_start = 0.0f, .qd_end_prev = -1000.0f},
		{.freq = 0.001f, .offset = 0.0f, .prev_fp = 100.0f, .qd_start = 0.0f, .qd_end_prev = -1000.0f},
		{.freq = 0.001f, .offset = 0.0f, .prev_fp = 100.0f, .qd_start = 0.0f, .qd_end_prev = -1000.0f},
		{.freq = 0.001f, .offset = 0.0f, .prev_fp = 100.0f, .qd_start = 0.0f, .qd_end_prev = -1000.0f},
		{.freq = 0.001f, .offset = 0.0f, .prev_fp = 100.0f, .qd_start = 0.0f, .qd_end_prev = -1000.0f},
		{.freq = 0.001f, .offset = 0.0f, .prev_fp = 100.0f, .qd_start = 0.0f, .qd_end_prev = -1000.0f}
	},
	.smoothing_time = 0.7f,
	.smoothing_jump_trigger = 1.0f,
	.motor_current_limit = MAX_POWER_CURRENT_THRESH
};

/*Finger torque based position control with cooling based on abs_err_set*/
void finger_pctl(hand_t * hand, float_format_i2c * in, float_format_i2c * out)
{
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		hand->mp[ch].q = in->v[ch];
						
		smooth_qd(
			hand->mp[ch].qd_set, 
			hand->smoothing_time, 
			hand->smoothing_jump_trigger, 
			hand->mp[ch].q,
			&(hand->sm[ch]),
			&(hand->mp[ch].qd)
		);
		
		float tau = hand->mp[ch].k*(hand->mp[ch].qd-hand->mp[ch].q);
		if(tau > hand->motor_current_limit)
			tau = hand->motor_current_limit;
		else if(tau < -hand->motor_current_limit)
			tau = -hand->motor_current_limit;
		
		hand->sp[ch].abs_err_set = abs_f(hand->mp[ch].qd_set - hand->mp[ch].q);
		if(hand->sp[ch].abs_err_set < hand->sp[ch].setpoint_err_thresh)
			tau = 0.f;

		out->v[ch] = tau;
				
	}
}