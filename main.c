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
#include "PID.h"


int main(void) {
	InitializeLEDs();

	init_sensors();
	init_platform();
	init_PID();
	init_bluetooth();
	setLeftCalSpeed(0.945);

	startForward(10000,10,1);
	while(1) {
		process_sensors();
		process_platform();
		process_PID();
		process_bluetooth();
	}
}
