#include "i2c-master-test.h"
#include "i2c-err-lookup.h"
#include <stdio.h>
#include <time.h>

static struct timeval tv;

enum {PROGRAM_START, HAND_OPEN, HAND_POWER_GRASP};

/*Special type for sending grip commmands over i2c*/
typedef struct grip_cmd_t
{
	uint8_t header;
	uint8_t grip;
	uint8_t speed;
}grip_cmd_t;


/*Returns the current time in seconds*/
float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}

/*Write a grip command struct over i2c (3 bytes)*/
int write_i2c_grip(grip_cmd_t grip)
{
	uint8_t buf[3];
	buf[0] = grip.header;
	buf[1] = grip.grip;
	buf[2] = grip.speed;
	if(write(file_i2c, buf, 3) != 3)
		return -1;
	else
		return 1;
}

void main()
{
	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	uint8_t i2c_rx_buf[8];	//for reading over i2c

	float tstart = current_time_sec(&tv);	//timestamp to mark program start time
	float next_state_ts = 1.f;	//used to schedule the next state transition
	int state = PROGRAM_START;	//current state. 
	
	uint64_t cycle_count = 0;
	uint8_t cycle_chk_ready_flag = 0;
	float program_hung_ts = 20.f;
	printf("Program begin.\r\n");
	while(1)
	{
		float t = current_time_sec(&tv)-tstart;
		int comms_ok = 0;	//0 denotes ok comms
		
		/*Read the hand i2c data*/
		if(read(file_i2c, i2c_rx_buf, 8) == 8)
		{}
		else
			comms_ok |= 1 << 1;

		/*Manage what state happens next and when (relative to the execution of the current state switch). 
		Statements wrapped in these cases get called once*/
		if(t > next_state_ts)
		{
			switch(state)
			{
				case PROGRAM_START:	//program start has ENDED. transition action to next state...
				{
					if(write_i2c_grip((grip_cmd_t){0x1D, 0x00, 0xFA}) != 1)	//transistion to hand_open and send the open signal
						comms_ok |= 1;
					state = HAND_OPEN;
					next_state_ts = t+=.3f;
					break;
				}
				case HAND_OPEN:
				{
					if(write_i2c_grip((grip_cmd_t){0x1D, 0x01, 0xFA}) != 1)	//transition to hand_power_grasp and close the hand
						comms_ok |= 1;
					state = HAND_POWER_GRASP;
					next_state_ts = t + 3.f;
					break;
				}
				case HAND_POWER_GRASP:
				{		
					if(write_i2c_grip((grip_cmd_t){0x1D, 0x00, 0xFA}) != 1)	//transition to hand_open and open the hand
						comms_ok |= 1;
					state = HAND_OPEN;
					cycle_chk_ready_flag = 1;
					next_state_ts = t + 3.f;
					break;
				}
				default:
					break;
			};
			//statements not wrapped in the switch-case get called every state transition
			if(comms_ok != 0)
				print_hr_errcode(comms_ok);
		}

		/*Manage what happens while in the current state
		Statements wrapped in these cases get called continuously,
		while in the relevant state.*/
		switch(state)
		{
			case(HAND_OPEN):
			{
				if(comms_ok == 0 && i2c_rx_buf[7] == 0x17 && cycle_chk_ready_flag == 1)	//power grasp
				{
					program_hung_ts = t + 20.f;
					cycle_chk_ready_flag = 0;
					cycle_count++;
					printf("count = %llu, time = %f\r\n", cycle_count, t);
				}
				break;
			}
			default:
				break;
		};		
		if(t > program_hung_ts)
		{
			printf("error. hand has not cycled for 20 seconds\r\n");
			break;
		}
	}
	printf("exiting program\r\n");
}
