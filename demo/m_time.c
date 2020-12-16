#include "m_time.h"
#include <string.h>

/*Setpoint generation start time*/
struct timeval tv;
struct timeval start_time;


int64_t current_time_us(void)
{
	gettimeofday(&tv,NULL);
	return ( (tv.tv_sec - start_time.tv_sec)*1000000 + (tv.tv_usec - start_time.tv_usec) );
}

float current_time_sec(void)
{
	return ((float)current_time_us())/1000000.0f;
}

uint64_t get_tick()
{
	return (uint64_t)(current_time_us())/1000;
}

void m_time_init(void)
{
	gettimeofday(&start_time, NULL);
}
