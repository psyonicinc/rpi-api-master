#include "smoothing.h"


/*
 * INPUTS:
 *
 * USER INPUTS:
 * qd_end: 		new setpoint, to approach in a smooth fashion
 * period: 		the time in seconds it takes for the finger to track its new position
 *
 * INPUT PARAMETERS
 * q:	 		finger position
 *
 *PASS BY POINTER/REFERENCE/HELPER VARIABLES
 * start_ts, qd_start, qd_end_prev, freq
 *
 *OUTPUTS
 *qd, true setpoint for position control
 *
 */
void smooth_qd(float qd_end, float period, float thresh,
		float q,
		smooth_mem_t * sm,
		float * qd)
{
	float abs_diff = abs_f(qd_end - (sm->qd_end_prev));
	if(abs_diff > thresh)	//if the jump in setpoint value is large
	{
		sm->start_ts = get_tick();	//reset the tracking time
		sm->offset = 0.0f;
		sm->qd_start = q;			//set the start point to our current position, (in setpoint coordinates!!!)
		sm->freq = PI/period;	//precalculate so we don't have to constantly divide
	}

	if(abs_f(period - sm->prev_fp) > 0.001f)	//if the previous period is not equal to the current period
	{
		sm->offset = ((float)(get_tick() - sm->start_ts)*.001f)*(PI/(sm->prev_fp)) + sm->offset;
		sm->start_ts = get_tick();
		sm->freq = PI/period;	//if we get a new period, calculate the frequency for that period
	}

	float ft_st = ((float)(get_tick() - sm->start_ts)*.001f)*sm->freq + sm->offset;
	float t = ft_st*period*ONE_BY_PI;

	if(t <= period)
		*qd = (qd_end-sm->qd_start)*(.5f*(sin_fast(ft_st-HALF_PI)+1.0f)) + sm->qd_start;
	else
		*qd = qd_end;

	sm->qd_end_prev = qd_end;		//block entry into this routine again/update the previous value
	sm->prev_fp = period;
}
