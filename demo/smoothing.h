#ifndef SMOOTHING_H
#define SMOOTHING_H

#include "sin_math.h"
#include "m_time.h"


typedef struct smooth_mem_t
{
	uint32_t start_ts;
	float qd_start;
	float qd_end_prev;
	float freq;
	float offset;
	float prev_fp; //previous period or frequency term. 
}smooth_mem_t;

void smooth_qd(float qd_end, float period, float thresh,
		float q,
		smooth_mem_t * sm,
		float * qd);

#endif
