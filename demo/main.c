#include "i2c-master-test.h"

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

/*Opens with fixed slow velocity using torque control mode
INPUTS:
	period: amount of time to try opening. does not effect grip velocity
OUTPUTS:
*/
void open_grip(float period, struct timeval * tv, uint8_t * disabled_stat)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	//uint8_t disabled_stat=0;
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
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		float t = current_time_sec(tv) - cur_state_start_ts;
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			if(ch == THUMB_ROTATOR)
			{
				qd[ch] = -80.f;
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
void close_grip(float period, struct timeval * tv, uint8_t * disabled_stat)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
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
	float fspeed[NUM_CHANNELS] = {25,25,25,25,5,0};	//in degrees per second
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			float t = current_time_sec(tv) - cur_state_start_ts;
			if(ch == THUMB_ROTATOR)
			{
				qd[ch] = -80.f;
				i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]), 40.f);	
			}
			else
			{
				qd[ch] = fspeed[ch]*(t) + q_state_start[ch];
				i2c_out.v[ch] = sat_f(2.f*(qd[ch]-i2c_in.v[ch]), 40.f);								
			}
		}
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		//print_disabled_stat(disabled_stat);
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
void squeeze_grip(float period, float squeeze_torque, struct timeval * tv, uint8_t * disabled_stat)
{
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_in;
	float_format_i2c i2c_out;
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
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
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		float t = current_time_sec(tv)-cur_state_start_ts;
		float tau_des = (.5f*sin(2*pi*(1/period)*t - half_pi)+0.5f) * squeeze_torque;	//0-20-0 sinusoid over the full range of this for loop interval
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			if(ch == THUMB_ROTATOR)
				i2c_out.v[ch] = sat_f(2.f*(-80.f-i2c_in.v[ch]),40.f);
			else
				i2c_out.v[ch] = tau_des;
		}		
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, disabled_stat, &pres_fmt);
		//print_disabled_stat(disabled_stat);
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
	open_grip(10.f, &tv, &disabled_stat);
	open_grip(10.f, &tv, &disabled_stat);
	printf("closing...\r\n");
	close_grip(10.f, &tv, &disabled_stat);
	printf("squeezing...\r\n");
	squeeze_grip(5.f, 20.f, &tv,&disabled_stat);
	printf("done!\r\n");	
	for(float end_ts = current_time_sec(&tv)+3.f; current_time_sec(&tv)<end_ts;)
	{
		for(int ch = 0; ch < NUM_CHANNELS-1; ch++)
			printf("q[%d]=%f, ",ch, i2c_in.v[ch]);
		printf("q[%d]=%f\r\n",THUMB_ROTATOR, i2c_in.v[THUMB_ROTATOR]);
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
			i2c_out.v[ch] = 0.f;
		int rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);		
	}
	open_grip(10.f, &tv,&disabled_stat);
}
