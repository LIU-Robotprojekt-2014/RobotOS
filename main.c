#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#include <stm32f4xx_gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4_discovery.h"
#include <math.h>
#include "platform.h"
#include "sensors.h"
#include "bluetooth.h"


int main(void) {

	SystemInit();
	init_platform();
	init_sensors2();
	InitializeLEDs();
	init_rotary();



	while(1) {
		process_sensors();
		process_platform();
		if(i>1500){
			set_stop();
			i=0;
		}
	}

