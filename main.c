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

void Delay(__IO uint32_t nCount) {
  while(nCount--){}
}

int main(void) {

  //init_platform();
  //init_sensors2();
  init_bluetooth();
  uint32_t i = 0;
  while(1) {
	  process_bluetooth();
	  i++;
	  if(i>1000) {
		  i = 0;
	  }
	  /*
	  set_forward();
	  process_platform();
	  set_backward();
	  process_platform();
	  set_left();
	  process_platform();
	  set_right();
	  process_platform();
	  set_stop();
	  process_platform();
	  */
  }

}

/**************************************************************************************/
