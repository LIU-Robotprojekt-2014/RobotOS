#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"
#include "sensors.h"
#include <stm32f4xx_tim.h>
#include <stdio.h>


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
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  int change=0;


void TIM_Config(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); */

  /* Connect TIM3 pins to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
}

void PWM_Config(int period)
{
  uint16_t PrescalerValue = 0;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void PWM_SetDC(uint16_t channel,uint16_t dutycycle)
{
  if (channel == 1)
  {
    TIM3->CCR1 = dutycycle;
  }
  else if (channel == 2)
  {
    TIM3->CCR2 = dutycycle;
  }
  else if (channel == 3)
  {
    TIM3->CCR3 = dutycycle;
  }
  else if (channel == 4)
    {
      TIM3->CCR4 = dutycycle;
    }
}

void init_platform(void) {
	MP._Lspeed = 100;
	MP._Rspeed = 100;
	MP._state = 0;
	MP._flags = 0;
	istop=0;

	MP._left_side._calibrate_speed 	= 1;
	MP._left_side._state 			= 0;
	MP._left_side._forward_pin 		= GPIO_Pin_11;
	MP._left_side._backward_pin 	= GPIO_Pin_9;

	MP._right_side._calibrate_speed = 1;
	MP._right_side._state 			= 0;
	MP._right_side._forward_pin 	= GPIO_Pin_10;
	MP._right_side._backward_pin 	= GPIO_Pin_12;

	uint16_t pulse_width = 0;
	/* TIM Configuration */
	TIM_Config();
	/* PWM Configuration */
	PWM_Config(4000);

	//PC6 kanal 1
	//PC7 kanal 2
	//PC8 kanal 3
	//PC9 kanal 4
}

void process_platform() {

	if(i>=istop){
		set_stop();
	}

	if(change==1){
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
			default:
				_stop();
				break;
		}
		change=0;
	}
}

int set_forward(int ls, int rs) {
	change=1;
	MP._Lspeed = ls;
	MP._Rspeed = rs;
	MP._state = 1;
	if(MP._state == 1) {
		return 0;
	}
	return -1;
}
int set_backward(int ls, int rs) {
	change=1;
	MP._Lspeed = ls;
	MP._Rspeed = rs;
	MP._state = 2;
	if(MP._state == 2) {
		return 0;
	}
	return -1;
}
int set_left(int ls, int rs) {
	change=1;
	MP._Lspeed = ls;
	MP._Rspeed = rs;
	MP._state = 3;
	if(MP._state == 3) {
		return 0;
	}
	return -1;
}
int set_right(int ls, int rs) {
	change=1;
	MP._Lspeed = ls;
	MP._Rspeed = rs;
	MP._state = 4;
	if(MP._state == 4) {
		return 0;
	}
	return -1;
}
int set_stop() {
	change=1;
	MP._state = 0;
	if(MP._state == 0) {
		return 0;
	}
	return -1;
}

void _go_forward(void) {
	_stop();
	PWM_SetDC(2,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(3,40*MP._Rspeed*MP._right_side._calibrate_speed);
}

void _go_backward(void) {
	_stop();
	PWM_SetDC(4,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(1,40*MP._Rspeed*MP._right_side._calibrate_speed);

}

void _turn_left(void) {
	_stop();
	PWM_SetDC(4,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(3,40*MP._Rspeed*MP._right_side._calibrate_speed);

}

void _turn_right(void) {
	_stop();
	PWM_SetDC(2,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(1,40*MP._Rspeed*MP._right_side._calibrate_speed);
}


void _stop(void) {
	PWM_SetDC(1,0);
	PWM_SetDC(2,0);
	PWM_SetDC(3,0);
	PWM_SetDC(4,0);

}

void startForward(int distance){
	i=0;
	istop=distance;
	set_forward(100,100);
}

void startLeft(void){
	i=0;
	istop=210;
	set_left(100,100);
}

void startRight(void){
	i=0;
	istop=237;
	set_right(100,100);
}

void setLeftCalSpeed( float c){
	MP._left_side._calibrate_speed 	= c;
}

void setRightCalSpeed( float c){
	MP._right_side._calibrate_speed = c;
}

void InitializeLEDs()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpioStructure;
    gpioStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpioStructure);

    GPIO_WriteBit(GPIOD, GPIO_Pin_12 | GPIO_Pin_13, Bit_RESET);
}
