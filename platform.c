#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"

MotorPlatform MP;

void init_platform(void) {
	MP._speed = 100;
	MP._state = 0;
	MP._flags = 0;

	MP._left_side._calibrate_speed 	= 1;
	MP._left_side._state 			= 0;
	MP._left_side._forward_pin 		= GPIO_Pin_11;
	MP._left_side._backward_pin 	= GPIO_Pin_9;

	MP._right_side._calibrate_speed = 1;
	MP._right_side._state 			= 0;
	MP._right_side._forward_pin 	= GPIO_Pin_10;
	MP._right_side._backward_pin 	= GPIO_Pin_12;

	GPIO_InitTypeDef  GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//GPIO_InitStruct.GPIO_Pin configures the pins that will be used.
	//In this case we will use the LED's off of the discovery board which are on
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;

	//PIO_InitStruct.GPIO_Mode configures the pin mode the options are as follows
	// GPIO_Mode_IN (Input Mode)
	// GPIO_Mode_OUT (Output Mode)
	// GPIO_Mode_AF (Alternate Function)
	// GPIO_Mode_AN (Analog Mode)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

	//GPIO_InitStruct.GPIO_Speed configures the clock speed, options are as follows
	// GPIO_Speed_2MHz
	// GPIO_Speed_25MHz
	// GPIO_Speed_50MHz
	// GPIO_Speed_100MHz
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	//GPIO_InitStruct.GPIO_OType configures the pin type, options are as follows
	// GPIO_OType_PP (Push/Pull)
	// GPIO_OType_OD (Open Drain)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

	//Configures pullup / pulldown resistors on pin, options are as follows
	// GPIO_PuPd_NOPULL (Disables internal pullup and pulldown resistors)
	// GPIO_PuPd_UP (Enables internal pullup resistors) (~40K (30-50k))
	// GPIO_PuPd_DOWN (Enables internal pulldown resistors) (~40K (30-50k))
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

	//This finally passes all the values to the GPIO_Init function
	//which takes care of setting the corresponding bits.
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);

}

void go_forward(void) {
	stop();
	GPIO_SetBits(GPIOE, MP._left_side._forward_pin|MP._right_side._forward_pin);
}

void go_backward(void) {
	stop();
	GPIO_SetBits(GPIOE, MP._left_side._backward_pin|MP._right_side._backward_pin);
}

void turn_left(void) {
	stop();
	GPIO_SetBits(GPIOE, MP._left_side._forward_pin|MP._right_side._backward_pin);
}

void turn_right(void) {
	stop();
	GPIO_SetBits(GPIOE, MP._left_side._backward_pin|MP._right_side._forward_pin);
}

void stop(void) {
	GPIO_ResetBits(GPIOE, MP._left_side._forward_pin|MP._right_side._forward_pin|MP._left_side._backward_pin|MP._right_side._backward_pin);
}
