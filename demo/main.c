#include "i2c-master-test.h"
#include <stdio.h>


void main()
{
	open_i2c(0x50);	//Initialize the I2C port. Currently default setting is 100kHz clock rate

	uint8_t i2c_rx_buf[8];
	while(1)
	{
		if(read(file_i2c, i2c_rx_buf, 8) == 8)
		{
			printf("d = ");
			for(int i =0; i < 8; i++)
				printf("[%d]", i2c_rx_buf[i]);
			printf("\r\n");
		}
		else
			printf("error\r\n");		
	}
}
