#include "i2c-master-test.h"
#include "i2c-err-lookup.h"
#include "sig-catch.h"
#include <stdio.h>
#include "m_time.h"

static struct timeval tv;

enum {PROGRAM_START, HAND_OPEN, HAND_POWER_GRASP};

/*Special type for sending grip commmands over i2c*/
typedef struct grip_cmd_t
{
	uint8_t header;
	uint8_t grip;
	uint8_t speed;
}grip_cmd_t;


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
	m_time_init();
	signal_catch_setup();
	
	FILE * tlog_fp = NULL;
	tlog_fp = fopen("log.txt", "a");
	fseek(tlog_fp, 0, SEEK_END);
	
	uint8_t i2c_rx_buf[8];	//for reading over i2c

	
	float next_state_ts = 1.f;	//used to schedule the next state transition
	int state = PROGRAM_START;	//current state. 
	
	uint64_t cycle_count = 0;
	uint8_t cycle_chk_ready_flag = 0;
	float program_hung_ts = 20.f;
	
	printf("Program begin.\r\n");
	fprintf(tlog_fp, "Program begin.\r\n");
	float tstart = current_time_sec();	//timestamp to mark program start time
	printf("start time: %f\r\n", tstart);
	fprintf(tlog_fp, "start time: %f\r\n", tstart);
	
	while(gl_run_ok)
	{
		float t = current_time_sec()-tstart;
		int comms_stat = 0;	//0 denotes ok comms
		
		/*Read the hand i2c data*/
		if(read(file_i2c, i2c_rx_buf, 8) == 8)
		{}
		else
			comms_stat |= 1 << 1;

		/*Manage what state happens next and when (relative to the execution of the current state switch). 
		Statements wrapped in these cases get called once*/
		if(t > next_state_ts)
		{
			switch(state)
			{
				case PROGRAM_START:	//program start has ENDED. transition action to next state...
				{
					if(write_i2c_grip((grip_cmd_t){0x1D, 0x00, 0xEF}) != 1)	//transistion to hand_open and send the open signal
						comms_stat |= 1;
					state = HAND_OPEN;
					next_state_ts = t+=.3f;
					break;
				}
				case HAND_OPEN:
				{
					if(i2c_rx_buf[7] == 0x17)
					{
						fprintf(tlog_fp, ", hand opened OK\r\n");
						printf(", hand opened OK\r\n");
					}
					else
					{
						fprintf(tlog_fp, ", bad open\r\n");
						printf(", bad open\r\n");
					}
					fclose(tlog_fp);
					if(write_i2c_grip((grip_cmd_t){0x1D, 0x01, 0xEF}) != 1)	//transition to hand_power_grasp and close the hand
						comms_stat |= 1;
					state = HAND_POWER_GRASP;
					next_state_ts = t + 3.f;
					break;
				}
				case HAND_POWER_GRASP:
				{		
					tlog_fp = fopen("log.txt", "a");
					fseek(tlog_fp, 0, SEEK_END);
					
					if(write_i2c_grip((grip_cmd_t){0x1D, 0x00, 0xEF}) != 1)	//transition to hand_open and open the hand
						comms_stat |= 1;
					state = HAND_OPEN;
					cycle_chk_ready_flag = 1;
					next_state_ts = t + 3.f;
					break;
				}
				default:
					break;
			};
			//statements not wrapped in the switch-case get called every state transition
			if(comms_stat != 0)
				print_hr_errcode(comms_stat);
		}

		/*Manage what happens while in the current state
		Statements wrapped in these cases get called continuously,
		while in the relevant state.*/
		switch(state)
		{
			case(HAND_OPEN):
			{
				//if(comms_stat == 0 && i2c_rx_buf[7] == 0x17 && cycle_chk_ready_flag == 1)	//power grasp
				if(comms_stat == 0 && cycle_chk_ready_flag == 1)
				{
					program_hung_ts = t + 20.f;
					cycle_chk_ready_flag = 0;
					cycle_count++;
					printf("count = %llu, time = %f", cycle_count, t);
					fprintf(tlog_fp, "count = %llu, time = %f", cycle_count, t);
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
	fclose(tlog_fp);
}
