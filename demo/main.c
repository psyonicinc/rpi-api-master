#include "i2c-master-test.h"
#include "i2c-err-lookup.h"
#include "sig-catch.h"
#include "m_time.h"
#include "hand.h"


#define PRINT_PRESSURE	/*When enabled, prints the value of the pressure sensors on the index finger. */
//#define PRINT_POSITION	/*When enabled, prints the finger position in degrees/*


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
		
	/*Setup for demo motion*/
	uint8_t disabled_stat = 0xFF;
	
	float qd[NUM_CHANNELS];
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
					next_state_ts = t + 1.5f;
					break;
				case CLOSE:
					gl_hand.mp[INDEX].qd_set = 15.f;
					
					next_state = OPEN;
					next_state_ts = t+1.5f;
					break;
				default: 
					break;
			}
			state = next_state;
		}
		
		if(rc == 0)
		{
			
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				gl_hand.mp[ch].q = i2c_in.v[ch];
								
				smooth_qd(
					gl_hand.mp[ch].qd_set, 
					gl_hand.smoothing_time, 
					gl_hand.smoothing_jump_trigger, 
					gl_hand.mp[ch].q,
					(smooth_mem_t *)(&(gl_hand.sm[ch])),
					(float *)(&(gl_hand.mp[ch].qd))
				);
				
				float tau = gl_hand.mp[ch].k*(gl_hand.mp[ch].qd-gl_hand.mp[ch].q);
				if(tau > gl_hand.motor_current_limit)
					tau = gl_hand.motor_current_limit;
				else if(tau < -gl_hand.motor_current_limit)
					tau = -gl_hand.motor_current_limit;
				
				gl_hand.sp[ch].abs_err_set = abs_f(gl_hand.mp[ch].qd_set - gl_hand.mp[ch].q);
				if(gl_hand.sp[ch].abs_err_set < gl_hand.sp[ch].setpoint_err_thresh)
					tau = 0.f;

				i2c_out.v[ch] = tau;
						
				//printf("[%.2f]", tau);
				printf("[%.2f]", gl_hand.sp[ch].abs_err_set);
			}
			printf("\r\n");
		}
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		print_hr_errcode(rc);
	}
}
