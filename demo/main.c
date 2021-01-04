#include "i2c-master-test.h"
#include "i2c-err-lookup.h"
#include "m_time.h"

//#define PRINT_PRESSURE	/*When enabled, prints the value of the pressure sensors on the index finger. */
#define CABLE_TEST
//#define PRINT_POSITION	/*When enabled, prints the finger position in degrees/*

// float current_time_sec(struct timeval * tv)
// {
	// gettimeofday(tv,NULL);
	// int64_t t_int = (tv->tv_sec*1000000+tv->tv_usec);
	// return ((float)t_int)/1000000.0f;
// }

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
{	FILE *log; //log file init of file and cleaning
	time_t now;
	int clock, secsPastMidnight, hours, minutes, seconds;
	log = fopen("log.txt","a");
	time(&now);
			clock = now - 18000;
			secsPastMidnight = clock % 86400;
		hours = (secsPastMidnight / 3600)-1;
		if (hours == -1)	hours = 23;
			minutes = (secsPastMidnight % 3600) / 60;
			seconds = secsPastMidnight % 60;
	fprintf(log, "%02d:%02d:%02d Beginning of the test \n", hours, minutes, seconds);
	printf("%02d:%02d:%02d Beginning of the test \n", hours, minutes, seconds);
	fclose(log);

	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	/*Quick example of pre-programmed grip control (i.e. separate control mode from torque, velocity and position control)*/
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);
	set_grip(POWER_GRASP_CMD,100);
	usleep(2000000);
	set_grip(GENERAL_OPEN_CMD,100);
	usleep(2000000);

	/*Setpoint generation start time*/
	struct timeval tv;

		int connect_flag[5];
		for (int cnt = 0; cnt<5; cnt++)
			connect_flag[cnt]=1;
		int prev_phase = 0;
		int phase = 0;
		uint32_t cable_data[10];
		uint32_t old_cable_data[10];
		uint32_t gl_fall_diff = 0;
		uint32_t gl_rise_diff = 0;
		for (int i=0; i<10; ++i){
			cable_data[i]=0;
			old_cable_data[i]=0;
		}

	/*All control modes will use the same float format struct for input and output. Initializing them here*/

	/*Setup for demo motion*/
	uint8_t disabled_stat = 0xFF;

	float qd[NUM_CHANNELS];
	pres_union_fmt_i2c pres_fmt;
	float_format_i2c i2c_out;
	for(int ch = 0; ch < NUM_CHANNELS; ch++)
		i2c_out.v[ch] = 0;
	float_format_i2c i2c_in;
	int rc=7;
	int state[NUM_CHANNELS]={0};
	unsigned int counter[NUM_CHANNELS]={0};

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

	float start_ts = current_time_sec();
	float tau_thresh[NUM_CHANNELS] = {0};
	while(1)
	{
		if(rc == 0)
		{
			float t = current_time_sec()-start_ts;
			for(int ch = 0; ch < NUM_CHANNELS; ch++)
			{
				float qd = 105.f*(.5f*sin(2.f*t+(float)(5-ch)*3.1415f/6)+.5f) + 5.f;
				if(ch == THUMB_ROTATOR)
					qd = -qd;

				/*Create a cooldown handler rule (stop the finger if it has triggered the 'hot' flag*/
				int chk = (disabled_stat >> ch) & 1;
				if(chk)
				{
					tau_thresh[ch] = 0.f;
					if (ch == 2)
					printf("[%s hot]", finger_name[ch]);
				}
				else
					tau_thresh[ch] = 110.f;
				/*Perform torque based position control*/
				float tau = 1.f*(qd-i2c_in.v[ch]);
				//printf ("%f \n", tau);
				if(tau > tau_thresh[ch])
					tau = tau_thresh[ch];
				else if(tau < -tau_thresh[ch])
					tau = -tau_thresh[ch];
				i2c_out.v[ch] = tau;
				if (tau < -0.5) state[ch] = 3;
				if (tau > 0.5) state[ch] = 1;
				if (state[ch] == 1 && tau >= -0.5 && tau <= 0.5){
					state[ch] = 2;
					counter[ch]++;
				}
				if (state[ch] == 3 && tau >= -0.5 && tau <= 0.5) state[ch] = 0;

				//printf("%d\n", counter);

			}
		//	printf("\r\n");

			/*
			Pressure Indices:
			Index: 	0-3
			Middle: 4-7
			Ring: 	8-11
			Pinky: 	12-15
			Thumb: 	16-19

			Note that the The pressure range is NOT normalized (i.e. will range from 0-0xFFFF).
			*/

			#ifdef CABLE_TEST

				int i, pidx;
				for(pidx = 0; pidx < 19; pidx+=2){
						cable_data[pidx/2]=((uint32_t)pres_fmt.v[pidx+1] << 16) +  pres_fmt.v[pidx];
						//printf("%x ", cable_data[pidx/2]);
				}
				//printf("\n");

				for (i = 0; i <10; i+=2){
						int finger = i/2;
						gl_fall_diff = cable_data[i]-old_cable_data[i];
						gl_rise_diff = cable_data[i+1]-old_cable_data[i+1];
						if (gl_fall_diff>0x1000 || gl_rise_diff> 0x1000)
						  {//printf("Glitch");

							old_cable_data[i] = cable_data [i];
							old_cable_data[i+1] = cable_data[i+1];
							gl_fall_diff = cable_data[i]-old_cable_data[i];
							gl_rise_diff = cable_data[i+1]-old_cable_data[i+1];}

						// else if (gl_fall_diff!=0 || gl_rise_diff!=0){
						// 	printf("Fall %d\n", gl_fall_diff);
						// 	printf("Rise %d\n", gl_rise_diff);
						// 	old_cable_data[i] = cable_data [i];
						// 	old_cable_data[i+1] = cable_data[i+1];
						// 	gl_fall_diff = cable_data[i]-old_cable_data[i];
						// 	gl_rise_diff = cable_data[i+1]-old_cable_data[i+1];
						// }

						if (gl_fall_diff>0&&connect_flag[finger]==1){
							time(&now);
		    					clock = now - 18000;
		    					secsPastMidnight = clock % 86400;
		   					hours = (secsPastMidnight / 3600)-1;
								if (hours == -1)	hours = 23;
		    					minutes = (secsPastMidnight % 3600) / 60;
		    					seconds = secsPastMidnight % 60;
							connect_flag[finger] = 0;
					 		log = fopen("log.txt","a");
					 		fprintf(log, "%02d:%02d:%02d ", hours, minutes, seconds);
							fprintf(log, "Cycle %d ", counter[finger]);
					 		if (state[finger]==1)
					 		 {fprintf(log, "Finger %d disconnected during closing\n", finger+1);
								//printf("Finger %d disconnected during closing\n", finger+1);
								}
					 		else if (state[finger]==3)
					 			{fprintf(log, "Finger %d disconnected during opening\n", finger+1);
								//printf("Finger %d disconnected during opening\n", finger+1);
								}
							else if (state[finger]==0)
								{fprintf(log, "Finger %d disconnected while open \n", finger+1);
								//printf("Finger %d disconnected while open \n", finger+1);
								}
							else
							{fprintf(log, "Finger %d disconnected while closed \n", finger+1);
							//printf("Finger %d disconnected while closed \n", finger+1);
							}
					 		fclose(log);
							old_cable_data[i]=cable_data[i];
							old_cable_data[i+1]=cable_data[i+1];
						}

						else if (gl_rise_diff>0&&connect_flag[finger]==0){
							connect_flag[finger] = 1;
					 		log = fopen("log.txt","a");
							time(&now);
		    					clock = now - 18000;
		   					secsPastMidnight = clock % 86400;
		    					hours = (secsPastMidnight / 3600)-1;
									if (hours == -1)	hours = 23;
		    					minutes = (secsPastMidnight % 3600) / 60;
		    					seconds = secsPastMidnight % 60;
							fprintf(log, "%02d:%02d:%02d ", hours, minutes, seconds);
							fprintf(log, "Cycle %d ", counter[finger]);
					 		if (state[finger]==1)
					 		 {fprintf(log, "Finger %d connected during closing\n", finger+1);
								//printf("Finger %d connected during closing\n", finger+1);
								}
					 		else if (state[finger]==3)
					 			{fprintf(log, "Finger %d connected during opening\n", finger+1);
								//printf("Finger %d connected during opening\n", finger+1);
								}
							else if (state[finger]==0)
								{fprintf(log, "Finger %d connected while open \n", finger+1);
								//printf("Finger %d connected while open \n", finger+1);
								}
							else
							{fprintf(log, "Finger %d connected while closed \n", finger+1);
							//printf("Finger %d connected while closed \n", finger+1);
							}
	 				 		fclose(log);
							old_cable_data[i]=cable_data[i];
							old_cable_data[i+1]=cable_data[i+1];
						}

				}

			#endif

			// #ifdef PRINT_PRESSURE
			// 	int finger_idx = PINKY;
			// 	uint8_t pb_idx = 4*finger_idx;
			// 	if(pb_idx > 16)
			// 		pb_idx = 16;
			// 	int pidx = 0;
			// 	for(pidx = 0; pidx < 3; pidx++)
			// 		printf("%.3f, ",(float)pres_fmt.v[pb_idx+pidx]/6553.5f);
			// 	printf("%.3f\r\n",(float)pres_fmt.v[pb_idx+pidx]/6553.5f);
			//
			// #elif defined PRINT_POSITION	//Print the position
			// 	int ch;
			// 	for(ch = 0; ch < NUM_CHANNELS-1; ch++)
			// 		printf("q[%d] = %f, ",ch,i2c_in.v[ch]);
			// 	printf("q[%d] = %f\r\n",ch,i2c_in.v[ch]);
			// #else
			// 	const char * yn[2] = {"on ","off"};
			// 	for(int ch = 0; ch < NUM_CHANNELS; ch++)
			// 		printf("%s: %s ", finger_name[ch], yn[((disabled_stat >> ch) & 1)] );
			// 	printf("\r\n");
			// #endif
		}
		rc = send_recieve_floats(TORQUE_CTL_MODE, &i2c_out, &i2c_in, &disabled_stat, &pres_fmt);
		print_hr_errcode(rc);
	}
}
