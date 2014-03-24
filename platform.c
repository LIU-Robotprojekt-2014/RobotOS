#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"

MotorPlatform MP;
/*
 * States:
 * 0: Stop
 * 1: Forwards
 * 2: Backwards
 * 3: Left
 * 4: Right
 *
 */

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
void process_platform() {

	switch(MP._state){
		case(0):
			_stop();
			break;
		case(1):
			_go_forward();
			break;
		case(2):
			_go_backward();
			break;
		case(3):
			_turn_left();
			break;
		case(4):
			_turn_right();
			break;
	}
}

int set_forward() {
	MP._state = 1;
	if(MP._state == 1) {
		return 0;
	}
	return -1;
}
int set_backward() {
	MP._state = 2;
	if(MP._state == 2) {
		return 0;
	}
	return -1;
}
int set_left() {
	MP._state = 3;
	if(MP._state == 3) {
		return 0;
	}
	return -1;
}
int set_right() {
	MP._state = 4;
	if(MP._state == 4) {
		return 0;
	}
	return -1;
}
int set_stop() {
	MP._state = 0;
	if(MP._state == 0) {
		return 0;
	}
	return -1;
}

void _go_forward(void) {
	_stop();
	GPIO_SetBits(GPIOE, MP._left_side._forward_pin|MP._right_side._forward_pin);
}

void _go_backward(void) {
	_stop();
	GPIO_SetBits(GPIOE, MP._left_side._backward_pin|MP._right_side._backward_pin);
}

void _turn_left(void) {
	_stop();
	GPIO_SetBits(GPIOE, MP._left_side._forward_pin|MP._right_side._backward_pin);
}

void _turn_right(void) {
	_stop();
	GPIO_SetBits(GPIOE, MP._left_side._backward_pin|MP._right_side._forward_pin);
}

void _stop(void) {
	GPIO_ResetBits(GPIOE, MP._left_side._forward_pin|MP._right_side._forward_pin|MP._left_side._backward_pin|MP._right_side._backward_pin);
}
