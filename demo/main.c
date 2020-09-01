#include "i2c-master-test.h"

/*Uncomment and compile whichever control mode you would like to test.*/

//#define PRINT_PRESSURE	/*When enabled, prints the value of the pressure sensors on the index finger. */
//#define PRINT_POSITION	/*When enabled, prints the finger position in degrees/*

float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}
float sat_f(float in, float thresh)
{
	if(in > thresh)
		return thresh;
	else if (in < -thresh)
		return -thresh;
}

void print_disabled_stat(uint8_t disabled_stat)
{
	const char * name[NUM_CHANNELS] = {"index","middle","ring","pinky","thumb flexor", "thumb rotator"};
	const char * yn[2] = {"on ","off"};
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		printf("%s: %s ", name[ch], yn[((disabled_stat >> ch) & 1)] );
	printf("\r\n");		
}

void open_grip(float period, struct timeval * tv, float_format_i2c * i2c_in, float_format_i2c * i2c_out, uint8_t * disabled_stat, pres_union_fmt_i2c * pres_fmt)
{
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	//uint8_t disabled_stat=0;
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out->v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .1; current_time_sec(tv) < end_ts;)
		send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in->v[ch];
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
				i2c_out->v[ch] = sat_f(2.f*(qd[ch]-i2c_in->v[ch]), 40.f);	
			}
			else
			{
				qd[ch] = -20.f*t + q_state_start[ch];
				i2c_out->v[ch] = sat_f(2.f*(qd[ch]-i2c_in->v[ch]), 40.f);								
			}
		}
		int rc = send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
		//print_disabled_stat(disabled_stat);
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		if(*disabled_stat & 0x1F != 0)
			break;
		usleep(10);
	}
}

void close_grip(float period, struct timeval * tv, float_format_i2c * i2c_in, float_format_i2c * i2c_out, uint8_t * disabled_stat, pres_union_fmt_i2c * pres_fmt)
{
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out->v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .1; current_time_sec(tv) < end_ts;)
		send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in->v[ch];
		qd[ch] = q_state_start[ch];
	}
	send_enable_word(0x3F);
	float cur_state_start_ts = current_time_sec(tv);
	float fspeed[NUM_CHANNELS] = {25,25,25,25,5,0};
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			float t = current_time_sec(tv) - cur_state_start_ts;
			if(ch == THUMB_ROTATOR)
			{
				qd[ch] = -80.f;
				i2c_out->v[ch] = sat_f(2.f*(qd[ch]-i2c_in->v[ch]), 40.f);	
			}
			else
			{
				qd[ch] = fspeed[ch]*(t) + q_state_start[ch];
				i2c_out->v[ch] = sat_f(2.f*(qd[ch]-i2c_in->v[ch]), 40.f);								
			}
		}
		int rc = send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
		//print_disabled_stat(disabled_stat);
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		if(*disabled_stat & 0x1F != 0)
			break;
		usleep(10);
	}
}
const float half_pi = M_PI/2;
const float pi = M_PI;
void squeeze_grip(float period, struct timeval * tv, float_format_i2c * i2c_in, float_format_i2c * i2c_out, uint8_t * disabled_stat, pres_union_fmt_i2c * pres_fmt)
{
	float q_state_start[NUM_CHANNELS];
	float qd[NUM_CHANNELS];
	for(int ch = 0; ch < NUM_CHANNELS; ch ++)
		i2c_out->v[ch] = 0.f;
	for(float end_ts = current_time_sec(tv) + .05; current_time_sec(tv) < end_ts;)
		send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		q_state_start[ch] = i2c_in->v[ch];
		qd[ch] = q_state_start[ch];
	}
	send_enable_word(0x3F);
	usleep(50000);
	//set_mode(DISABLE_TORQUE_VELOCITY_SAFETY);
	send_recieve_floats(DISABLE_TORQUE_VELOCITY_SAFETY, i2c_out, i2c_in, disabled_stat, pres_fmt);
	send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
	usleep(50000);

	
	float cur_state_start_ts = current_time_sec(tv);
	for(float end_ts = current_time_sec(tv) + period; current_time_sec(tv) < end_ts;)
	{
		float t = current_time_sec(tv)-cur_state_start_ts;
		float tau_des = (.5f*sin(2*pi*(1/period)*t - half_pi)+0.5f) * 20.f;	//0-20-0 sinusoid over the full range of this for loop interval
		for(int ch = 0; ch < NUM_CHANNELS; ch++)
		{
			if(ch == THUMB_ROTATOR)
				i2c_out->v[ch] = sat_f(2.f*(-80.f-i2c_in->v[ch]),40.f);
			else
				i2c_out->v[ch] = tau_des;
		}		
		int rc = send_recieve_floats(TORQUE_CTL_MODE, i2c_out, i2c_in, disabled_stat, pres_fmt);
		//print_disabled_stat(disabled_stat);
		if(rc != 0)
			printf("I2C error code %d\r\n",rc);
		usleep(10);
	}
	send_recieve_floats(ENABLE_TORQUE_VELOCITY_SAFETY, i2c_out, i2c_in, disabled_stat, pres_fmt);
	usleep(50000);
}


void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	/*Setpoint generation start time*/
	struct timeval tv;

	/*All control modes will use the same float format struct for input and output. Initializing them here*/
	
	/*Setup for demo motion*/
	uint8_t disabled_stat = 0;
	
	int prev_phase = 0;
	int phase = 0;
		
	float q_stop[NUM_CHANNELS] = {0};
	float qd[NUM_CHANNELS] = {0};
	qd[THUMB_ROTATOR] = -80.f;

	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	float_format_i2c i2c_in;
	for(float ts = current_time_sec(&tv) + .5; current_time_sec(&tv) < ts;)
		send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);	//initialize position and enter API
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
	{
		i2c_out.v[ch] = 0;
		qd[ch] = i2c_in.v[ch];
	}

	float start_ts = current_time_sec(&tv);
	
	
	enum {OPEN, CLOSE, SQUEEZE, RELAX};
	int state = OPEN;
	int squeeze_relax_cnt = 0;
	float q_state_start[NUM_CHANNELS];
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		q_state_start[ch] = i2c_in.v[ch];
	
	printf("opening...\r\n");
	open_grip(10.f, &tv, &i2c_in, &i2c_out, &disabled_stat, &pres_fmt);
	open_grip(10.f, &tv, &i2c_in, &i2c_out, &disabled_stat, &pres_fmt);
	printf("closing...\r\n");
	close_grip(10.f, &tv, &i2c_in, &i2c_out, &disabled_stat, &pres_fmt);
	printf("squeezing...\r\n");
	squeeze_grip(5.f, &tv,&i2c_in,&i2c_out, &disabled_stat, &pres_fmt);
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
	open_grip(10.f, &tv, &i2c_in, &i2c_out, &disabled_stat, &pres_fmt);
}
