#include "m_time.h"
#include <string.h>
#include <time.h>
/*Setpoint generation start time*/
//struct timeval tv;
//struct timeval start_time;

static long ms;
static time_t s;
static struct timespec spec;
static uint64_t start_time = 0;

uint64_t current_time_us(void)
{
	clock_gettime(CLOCK_BOOTTIME, &spec);
	s = spec.tv_sec;
	
	return (s * (uint64_t)1000000 + (spec.tv_nsec/1.0e3)) - start_time;
//	gettimeofday(&tv,NULL);
//	return ( (tv.tv_sec - start_time.tv_sec)*((uint64_t)1000000) + (tv.tv_usec - start_time.tv_usec) );
}

double current_time_sec(void)
{
	return ((double)current_time_us())/1000000.0;
}

uint64_t get_tick()
{
	return (uint64_t)(current_time_us())/1000;
}

void m_time_init(void)
{
	start_time = current_time_us();
//	gettimeofday(&start_time, NULL);
}
