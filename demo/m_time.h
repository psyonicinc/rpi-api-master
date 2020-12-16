#ifndef M_TIME_H
#define M_TIME_H

#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>


uint64_t current_time_us(void);
double current_time_sec(void);
uint64_t get_tick(void);
void m_time_init(void);

#endif
 