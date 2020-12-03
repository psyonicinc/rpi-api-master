#include "i2c-master-test.h"
#include "i2c-err-lookup.h"

#define PRINT_PRESSURE	/*When enabled, prints the value of the pressure sensors on the index finger. */
//#define PRINT_POSITION	/*When enabled, prints the finger position in degrees/*

float current_time_sec(struct timeval * tv)
{
	gettimeofday(tv,NULL);
	int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	return ((float)t_int)/1000000.0f;
}

const char * finger_name[] = {
	"index",
	"middle",
	"ring",
	"pinky",
	"thumb flex",
	"thumb rot",
};

/**/
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
}

void main()
{

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);
	set_grip(CHUCK_OK_GRASP_CMD,100);
	usleep(2000000);
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);

	/*Setpoint generation start time*/
	struct timeval tv;

	/*All control modes will use the same float format struct for input and output. Initializing them here*/
	
	/*Setup for demo motion*/
	uint8_t disabled_stat = 0xFF;
	
	float qd[NUM_CHANNELS];
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
	float_format_i2c current_in;
	int rc=7;
	
	set_mode(TORQUE_CTL_MODE);
	printf("prepping api...\r\n");
	while(rc != 0)
	{
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		if(rc != 0)
			printf("waiting...\r\n");
	}
	printf("api entered\r\n");
	printf("disabling pressure HPF...\r\n");
	if(set_mode(DISABLE_PRESSURE_FILTER) == 0)//example of pres filter disable
		printf("pressure filter disabled\r\n");
	else
		printf("comm failure, filter not disabled\r\n");
	printf("waiting for motor cooldown\r\n");

	wait_for_cooldown(&disabled_stat, &i2c_out, &i2c_in, &pres_fmt);
	printf("ready\r\n");
	
	for(int i = 0; i < 10; i++)
	{
		if(set_mode(EN_CURRENT_FEEDBACK_MODE) == 0)
			printf("current feedback ON\r\n");
		else
			printf("failure to enable current feedback\r\n");
		usleep(10000);
	}

	float start_ts = current_time_sec(&tv);
	float tau_thresh[NUM_CHANNELS] = {0};
	
	while(1)
	{
		if(rc == 0)
		{
			float t = current_time_sec(&tv)-start_ts;
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				float qd = 50.f*(.5f*sin(3*t+(float)(5-ch)*3.1415f/6)+.5f) + 10.f;
				if(ch == THUMB_ROTATOR)
					qd = -qd;
				
				/*Create a cooldown handler rule (stop the finger if it has triggered the 'hot' flag*/
				int chk = (disabled_stat >> ch) & 1;
				if(chk)
				{
					tau_thresh[ch] = 0.f;
					printf("[%s hot]", finger_name[ch]);
				}
				else
					tau_thresh[ch] = 90.f;
				
				/*Perform torque based position control*/
				float tau = 1.f*(qd-i2c_in.v[ch]);
				if(tau > tau_thresh[ch])
					tau = tau_thresh[ch];
				else if(tau < -tau_thresh[ch])
					tau = -tau_thresh[ch];
				i2c_out.v[ch] = tau;
			}
			printf("\r\n");
			
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
				printf("[%.1d: %2.2f]", ch, current_in.v[ch]*10.f);
		}
		
		//rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		rc = send_receieve_current_mode(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &current_in);

		print_hr_errcode(rc);
	}
}
