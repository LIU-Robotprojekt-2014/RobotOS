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

	//SystemInit();
	init_platform();
	init_sensors2();
	InitializeLEDs();
	init_rotary();
	init_bluetooth();
	setLeftCalSpeed(0.945);
	//startLeft();
	//startRight();
	startForward(2000);
	while(1) {
		process_sensors();
		process_platform();
		process_bluetooth();


		/*if(i>0 && i<600){
			set_forward();
		}
		if(i>600 && i<900){
			set_left(100,100);
		}
		if(i>900 && i<1200){
			set_right(100,100);
		}
		if(i>1200 && i<1800){
			set_backward(100,100);
		}
		if(i>18){
			set_stop();
		} */
	}
}
