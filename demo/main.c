#include "i2c-master-test.h"
#include "i2c-err-lookup.h"
#include <stdio.h>

const float half_pi = M_PI/2;
const float pi = M_PI;

static struct timeval tv;

/*Lookup table for finger array indexing*/
static const char * finger_name[] = {
	"index",
	"middle",
	"ring",
	"pinky",
	"thumb flex",
	"thumb rot",
};

/*Returns current time in seconds*/
float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}

/*Saturates floating point value to thresh*/
float sat_f(float in, float thresh)
{
	if(in > thresh)
		return thresh;
	else if (in < -thresh)
		return -thresh;
	else
		return in;
}

static float thumb_rotator_position = -40.f;

/*
	This function enters the api mode specified by the input argument. If you call it 
	with an invalid argument for mode, it will spin lock. It will also spin lock if there
	isn't a valid hand connected when it's called. 
	
	This function spins until it gets an i2c packet reply from a hand with the correct length (66 bytes)
	and a valid checksum (last byte in the packet). 
*/
void enter_api_mode(uint8_t mode)
{
	set_mode(mode);
	printf("prepping api...\r\n");
	pres_union_fmt_i2c pres_fmt = {0};
	float_format_i2c i2c_in = {0};
	float_format_i2c i2c_out = {0};	//this is unsafe for position mode. kludge-y 
	uint8_t disabled_stat=0;
	int rc = 7;
	while(rc != 0)	//potentially make this non-blocking...
	{
		rc = send_recieve_floats(mode, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		if(rc != 0)
			printf("waiting...\r\n");
	}
}

/*
	Spin until the hand notifies you via. the 'disabled flag' that the motors have cooled and can produce a large torque again
*/
void wait_for_cooldown(uint8_t * disabled_stat, float_format_i2c * out, float_format_i2c * in, pres_union_fmt_i2c * pres)
{
	while(*disabled_stat != 0)
	{
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			out->v[ch] = 0.f;
			int chk = (*disabled_stat >> ch) & 1;
			if(chk)
				printf("[%s cooling]", finger_name[ch]);
		}
		printf("\r\n");
		int rc = send_recieve_floats(TORQUE_CTL_MODE, out, in, disabled_stat, pres);
	}
	for(float t = current_time_sec(&tv) + 1; current_time_sec(&tv) < t;)	//extra second for good measure. need to continue sending api commands to stay in api mode
	{
		 int rc = send_recieve_floats(TORQUE_CTL_MODE, out, in, disabled_stat, pres);
	}
}

/*Opens with fixed slow velocity using torque control mode
INPUTS:
	period: amount of time to try opening. does not effect grip velocity
OUTPUTS:
	none
*/
void open_grip(float period, struct timeval * tv, uint8_t * disabled_stat, FILE * fp)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	int rc = 0;
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out.v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .1; current_time_sec(tv) < end_ts;)
	{
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		if(rc==0)
			thumb_rotator_position = i2c_in.v[THUMB_ROTATOR];
	}
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in.v[ch];
		qd[ch] = q_state_start[ch];
	}
	
	
	float cur_state_start_ts = current_time_sec(tv);
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		
		float t = current_time_sec(tv) - cur_state_start_ts;
		if(rc == 0)
		{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				if(ch == THUMB_ROTATOR)
				{
					qd[ch] = thumb_rotator_position;
					i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]), 40.f);	
				}
				else
				{
					qd[ch] = -20.f*t + q_state_start[ch];
					i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]), 40.f);								
				}
				int is_dis = (*disabled_stat >> ch) & 1;
				if(is_dis)
					i2c_out.v[ch] = 0.f;
			}

		}
		else
		{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = 0.f;	//if your last packet threw and error code, try and stop movement until you recover
		}
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		
		print_hr_errcode(rc);
		
		if(*disabled_stat & 0x1F != 0)
			break;
		usleep(10);
	}
}
/*
	Closes grip with fixed velocity.
	INPUTS:
		period: grip close attempt time. does not effect grip velocity.
	OUTPUTS:
		writes to file specified by fp filepointer, in a .csv format
*/
void close_grip(float period, struct timeval * tv, uint8_t * disabled_stat, FILE * fp)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	int rc=0;
	
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out.v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .1; current_time_sec(tv) < end_ts;)
	{
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		if(rc == 0)
			thumb_rotator_position = i2c_in.v[THUMB_ROTATOR];
	}
	if(rc != 0)
		thumb_rotator_position = -60.f;
	
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in.v[ch];
		qd[ch] = q_state_start[ch];
	}
	float cur_state_start_ts = current_time_sec(tv);
	float fspeed[NUM_CHANNELS] = {25,25,25,25,25,0};	//in degrees per second
	float e_cost[NUM_CHANNELS] = {0,0,0,0,0,0};
	float cost_update_ts=0.f;
	float sat_tau[NUM_CHANNELS] = {40.f, 40.f, 40.f, 40.f, 40.f, 40.f};
	
	
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		if(rc == 0)
		{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				float t = current_time_sec(tv) - cur_state_start_ts;
				if(ch == THUMB_ROTATOR)
				{
					qd[ch] = thumb_rotator_position;
					i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]), sat_tau[ch]);	
				}
				else
				{
					qd[ch] = fspeed[ch]*(t) + q_state_start[ch];
					i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]), sat_tau[ch]);								
				}
				int is_dis = (*disabled_stat >> ch) & 1;
				if(is_dis)
					i2c_out.v[ch] = 0.f;
			}
		}
		else
		{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = 0.f;	//if your last packet threw and error code, try and stop movement until you recover
		}


		uint8_t all_dis = 1;
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			float tau = i2c_out.v[ch];
			if(tau < 0.f)
				tau = -tau;
			e_cost[ch] += tau*.005f;
			if(e_cost[ch] > 3.1f)
				sat_tau[ch] = 0.f;
			else
				all_dis = 0;
		}
		if(all_dis)
			break;
		printf("ec[IDX]=%f\r\n",e_cost[INDEX]);
		
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		
		if(rc == 0)
		{
			for(int i =0; i< 20; i++)
				fprintf(fp, "%d,", pres_fmt.v[i]);
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				fprintf(fp, "%f, ", qd[ch]);		
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				fprintf(fp, "%f, ", i2c_out.v[ch]);
			for(int ch = 0; ch < 5; ch++)
				fprintf(fp, "%f, ", i2c_in.v[ch]);
			fprintf(fp, "%f\n", i2c_in.v[5]);
			
			if(*disabled_stat & 0x1F != 0)
				break;		
		}
		

		else
			print_hr_errcode(rc);

		usleep(10);
	}
}

/*This function performs a sinusoidal squeeze over a specified period of time with an internally chosen maximum
INPUTS:
	period: time over which to squeeze
	squeeze torque: peak torque of the sinusoidal profile
HELPER/PASS BY REFERENCE:
	tv: construct for time
	disabled_stat: status of the driver, for reference after completion of grip
OUTPUTS:
	writes to file specified by fp filepointer, in a .csv format
*/
void squeeze_grip(float period, float squeeze_torque, float squeeze_amp, struct timeval * tv, uint8_t * disabled_stat, FILE * fp)
{
	enter_api_mode(TORQUE_CTL_MODE);	//send the 
	
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	int rc = 0;

	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out.v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .05; current_time_sec(tv) < end_ts;)
	{
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		if(rc == 0)
		{
			thumb_rotator_position = i2c_in.v[THUMB_ROTATOR];
		}
	}
	if(rc != 0)
		thumb_rotator_position = -30.f;

	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in.v[ch];
		qd[ch] = q_state_start[ch];
	}
	
	printf("waiting for cooldown...\r\n");
	wait_for_cooldown(disabled_stat, &i2c_out, &i2c_in, &pres_fmt);
	printf("cooldown complete\r\n");

	float cur_state_start_ts = current_time_sec(tv);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		q_state_start[ch] = i2c_in.v[ch];
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		float t = current_time_sec(tv)-cur_state_start_ts;
		float offset_ratio = .6f;	//this factor adjusts how much the squeeze setpoint dips negative. the amount in degrees is equal to -squeeze_amp+squeeze_amp*offset_ratio*(1/(1+offset_ratio))
		const float two_pi = 2*pi;	
		/*comes in by squeeze amp, but opens less than squeeze amp. dips negative wrt. the starting position when squeeze is entered*/
		float q_des_base = (sin(two_pi*(1.f/period)*t + offset_ratio*two_pi) * squeeze_amp + offset_ratio*squeeze_amp)*(1.f/(1.f+offset_ratio));	//TODO: split this insanity into multiple lines
		if(rc == 0)
		{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				qd[ch] = q_des_base + q_state_start[ch];
				if(ch == THUMB_ROTATOR)
					i2c_out.v[ch] = sat_f(2.f*(thumb_rotator_position-i2c_in.v[ch]),40.f);
				else
					i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]),squeeze_torque);	//i2c_out.v[ch] = tau_des;			
			}
		}
		else
		{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				i2c_out.v[ch] = 0.f;	//if your last packet threw and error code, try and stop movement until you recover
		}

		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		
		if(rc == 0)
		{
			for(int i =0; i< 20; i++)
				fprintf(fp, "%d,", pres_fmt.v[i]);
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				fprintf(fp, "%f, ", qd[ch]);		
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				fprintf(fp, "%f, ", i2c_out.v[ch]);
			for(int ch = 0; ch < 5; ch++)
				fprintf(fp, "%f, ", i2c_in.v[ch]);
			fprintf(fp, "%f\n", i2c_in.v[5]);
		}

		print_hr_errcode(rc);
		usleep(10);
	}	
}


void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	FILE * fp = fopen("datalog.csv", "w");
	
	uint8_t disabled_stat = 0;	
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	float_format_i2c i2c_in;
	for(float ts = current_time_sec(&tv) + .5; current_time_sec(&tv) < ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);	//initialize position and enter API
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	
	printf("entering torque ctl...\r\n");
	enter_api_mode(TORQUE_CTL_MODE);
	printf("waiting for cooldown...\r\n");
	wait_for_cooldown(&disabled_stat, &i2c_out, &i2c_in, &pres_fmt);
	printf("cooldown complete\r\n");
	printf("disabling pres filter...\r\n");
	if(set_mode(DISABLE_PRESSURE_FILTER) == 0)
		printf("filter off\r\n");
	else
		printf("err: filter not disabled\r\n");

	printf("opening...\r\n");
	open_grip(3.f, &tv, &disabled_stat, fp);
	printf("closing...\r\n");
	close_grip(2.f, &tv, &disabled_stat, fp);
	for(int i = 0; i < 3; i++)
	{
		printf("squeezing %d...\r\n",i);
		squeeze_grip(24.f, 90.f, 17.f, &tv,&disabled_stat, fp);
	}
	printf("done!\r\n");	
	printf("opening...\r\n");
	open_grip(3.f, &tv,&disabled_stat, fp);
	
	fclose(fp);
}
