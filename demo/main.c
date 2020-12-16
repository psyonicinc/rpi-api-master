#include "i2c-master-test.h"
#include "i2c-err-lookup.h"
#include "sig-catch.h"
#include "m_time.h"
#include "hand.h"

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
	m_time_init();
	signal_catch_setup();
	
	printf("%lld",get_tick());
	usleep(2000000);
	/*Setup for demo motion*/
	uint8_t disabled_stat = 0xFF;
	
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
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
	usleep(3000000);	//delay for printf visibility
	printf("waiting for motor cooldown\r\n");
	wait_for_cooldown(&disabled_stat, &i2c_out, &i2c_in, &pres_fmt);
	printf("ready\r\n");
		
	
	
	enum {OPEN, CLOSE};
	
	int state = 0;
	float next_state_ts = 1.f;
	float start_ts = current_time_sec();
	gl_hand.smoothing_time = 1.f;
	
	while(gl_run_ok)
	{
		float t = current_time_sec() - start_ts;
		if(t > next_state_ts)
		{
			int next_state = 0;
			switch(state)
			{
				case OPEN:
					gl_hand.mp[INDEX].qd_set = 50.f;
					
					next_state = CLOSE;
					next_state_ts = t + 3.f;
					break;
				case CLOSE:
					gl_hand.mp[INDEX].qd_set = 15.f;
					
					next_state = OPEN;
					next_state_ts = t+3.0f;
					break;
				default: 
					break;
			}
			state = next_state;
		}
		switch(state)
		{
			
		};
		
		if(rc == 0)
			finger_pctl((hand_t*)&gl_hand, &i2c_in, &i2c_out);
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		print_hr_errcode(rc);
	}
}
