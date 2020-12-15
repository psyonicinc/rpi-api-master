#include "m_time.h"

/*Setpoint generation start time*/
struct timeval tv;

static volatile int64_t us_offset = 0;

int64_t current_time_us(void)
{
	gettimeofday(&tv,NULL);
	return (tv.tv_sec*1000000+tv.tv_usec) - us_offset;
}

float current_time_sec(void)
{
	return ((float)current_time_us())/1000000.0f;
}

int64_t get_tick()
{
	return (current_time_us())/1000;
}

void m_time_init(void)
{
	//memset(&tv,0,sizeof(struct timeval));
	//	us_offset = current_time_us();
}
