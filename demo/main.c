#include "i2c-master-test.h"
#include <stdio.h>

const float half_pi = M_PI/2;
const float pi = M_PI;

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
}
static float thumb_rotator_position = -40.f;
/*Opens with fixed slow velocity using torque control mode
INPUTS:
	period: amount of time to try opening. does not effect grip velocity
OUTPUTS:
*/
void open_grip(float period, struct timeval * tv, uint8_t * disabled_stat, FILE * fp)
{
	for(int i =0; i < 3; i++)
	{
		set_mode(DISABLE_PRESSURE_FILTER);
		usleep(1000);
	}
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	//uint8_t disabled_stat=0;
	set_mode(TORQUE_CTL_MODE);
	usleep(10000);
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out.v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .1; current_time_sec(tv) < end_ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in.v[ch];
		qd[ch] = q_state_start[ch];
	}
	send_enable_word(0x3F);
	thumb_rotator_position = i2c_in.v[THUMB_ROTATOR];
	float cur_state_start_ts = current_time_sec(tv);
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		float t = current_time_sec(tv) - cur_state_start_ts;
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
		}
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		//print_disabled_stat(disabled_stat);
		for(int i =0; i< 20; i++)
			fprintf(fp, "%d,", pres_fmt.v[i]);
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			fprintf(fp, "%f, ", qd[ch]);		
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			fprintf(fp, "%f, ", i2c_out.v[ch]);
		for(int ch = 0; ch < 5; ch++)
			fprintf(fp, "%f, ", i2c_in.v[ch]);
		fprintf(fp, "%f\n", i2c_in.v[5]);
		//fprintf(fp, "%d\n", pres_fmt.v[19]);
		
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		if(*disabled_stat & 0x1F != 0)
			break;
		usleep(10);
	}
}
/*
	Closes grip with fixed velocity.
	INPUTS:
		period: grip close attempt time. does not effect grip velocity.
*/
void close_grip(float period, struct timeval * tv, uint8_t * disabled_stat, FILE * fp)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	set_mode(TORQUE_CTL_MODE);
	usleep(10000);
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out.v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .1; current_time_sec(tv) < end_ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in.v[ch];
		qd[ch] = q_state_start[ch];
	}
	send_enable_word(0x3F);
	float cur_state_start_ts = current_time_sec(tv);
	float fspeed[NUM_CHANNELS] = {25,25,25,25,25,0};	//in degrees per second
	float e_cost[NUM_CHANNELS] = {0,0,0,0,0,0};
	float cost_update_ts=0.f;
	float sat_tau[NUM_CHANNELS] = {40.f, 40.f, 40.f, 40.f, 40.f, 40.f};
	thumb_rotator_position = i2c_in.v[THUMB_ROTATOR];
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
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
			
		}
		//if(current_time_sec(tv) >= cost_update_ts)
		//{
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				float tau = i2c_out.v[ch];
				if(tau < 0.f)
					tau = -tau;
				e_cost[ch] += tau*.005f;
				if(e_cost[ch] > 8.1f)
					sat_tau[ch] = 0.f;
			}
			//cost_update_ts = current_time_sec(tv) + .005f;	//5ms
			printf("ec[IDX]=%f\r\n",e_cost[INDEX]);
		//}
		
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		//print_disabled_stat(disabled_stat);
		for(int i =0; i< 20; i++)
			fprintf(fp, "%d,", pres_fmt.v[i]);
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			fprintf(fp, "%f, ", qd[ch]);		
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			fprintf(fp, "%f, ", i2c_out.v[ch]);
		for(int ch = 0; ch < 5; ch++)
			fprintf(fp, "%f, ", i2c_in.v[ch]);
		fprintf(fp, "%f\n", i2c_in.v[5]);

		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		if(*disabled_stat & 0x1F != 0)
			break;
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
*/
void squeeze_grip(float period, float num_squeezes, float squeeze_torque, float squeeze_amp, struct timeval * tv, uint8_t * disabled_stat, FILE * fp)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	set_mode(TORQUE_CTL_MODE);
	usleep(10000);
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out.v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .05; current_time_sec(tv) < end_ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in.v[ch];
		qd[ch] = q_state_start[ch];
	}
	send_enable_word(0x3F);
	usleep(50000);
	send_recieve_floats(DISABLE_TORQUE_VELOCITY_SAFETY, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
	send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
	usleep(50000);

	float cur_state_start_ts = current_time_sec(tv);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		q_state_start[ch] = i2c_in.v[ch];
	thumb_rotator_position = i2c_in.v[THUMB_ROTATOR];
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		float t = current_time_sec(tv)-cur_state_start_ts;
		float q_des_base = sin(2*pi*(num_squeezes/period)*t) * squeeze_amp;	//0-20-0 sinusoid over the full range of this for loop interval
		
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			qd[ch] = q_des_base + q_state_start[ch];
			if(ch == THUMB_ROTATOR)
				i2c_out.v[ch] = sat_f(2.f*(thumb_rotator_position-i2c_in.v[ch]),40.f);
			else
				i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]),squeeze_torque);	//i2c_out.v[ch] = tau_des;
		}
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		//print_disabled_stat(disabled_stat);
		for(int i =0; i< 20; i++)
			fprintf(fp, "%d,", pres_fmt.v[i]);
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			fprintf(fp, "%f, ", qd[ch]);		
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			fprintf(fp, "%f, ", i2c_out.v[ch]);
		for(int ch = 0; ch < 5; ch++)
			fprintf(fp, "%f, ", i2c_in.v[ch]);
		fprintf(fp, "%f\n", i2c_in.v[5]);
		
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		usleep(10);
	}
	send_recieve_floats(ENABLE_TORQUE_VELOCITY_SAFETY, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
	usleep(50000);
}


void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	FILE * fp = fopen("datalog.csv", "w");
	struct timeval tv;
	
	uint8_t disabled_stat = 0;	
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	float_format_i2c i2c_in;
	for(float ts = current_time_sec(&tv) + .5; current_time_sec(&tv) < ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);	//initialize position and enter API
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	
	printf("opening...\r\n");
	open_grip(10.f, &tv, &disabled_stat, fp);
	open_grip(10.f, &tv, &disabled_stat, fp);
	printf("closing...\r\n");
	close_grip(10.f, &tv, &disabled_stat, fp);
	printf("squeezing...\r\n");
	squeeze_grip(24.f, 2.75f, 40.f, 35.f, &tv,&disabled_stat, fp);
	printf("done!\r\n");	
	printf("opening...\r\n");
	open_grip(10.f, &tv,&disabled_stat, fp);
	
	fclose(fp);
}
