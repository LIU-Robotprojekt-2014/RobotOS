#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"
#include "sensors.h"
#include <stm32f4xx_tim.h>
#include <stdio.h>
#include "PID.h"
#include "order.h"


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

void PWM_SetDC(uint16_t channel,uint16_t dutycycle) {
  if (channel == 1) {
	  TIM3->CCR1 = dutycycle;
  }
  else if (channel == 2) {
	  TIM3->CCR2 = dutycycle;
  }
  else if (channel == 3) {
	  TIM3->CCR3 = dutycycle;
  }
  else if (channel == 4) {
	  TIM3->CCR4 = dutycycle;
  }
}

void init_platform(void) {
	MP._Lspeed = 100;
	MP._Rspeed = 100;
	MP._state = 0;
	MP._flags = 0;


	MP._left_side._forward_pin 		= GPIO_Pin_11;
	MP._left_side._backward_pin 	= GPIO_Pin_9;
	MP._left_side._calibrate_speed 	= 0.945;
	MP._left_side._state 			= 0;

	MP._right_side._forward_pin 	= GPIO_Pin_10;
	MP._right_side._backward_pin 	= GPIO_Pin_12;
	MP._right_side._calibrate_speed = 1;
	MP._right_side._state 			= 0;

	MP._rotary_driver_state			= 0;
	MP.rotary_driver_ticks			= 0;
	MP.rotary_driver_target_ticks	= 0;

	//Orders
	MP.completed_order = 0;


	istop=0;
	orderComplete=0;
	timer=0;

	//uint16_t pulse_width = 0;
	/* TIM Configuration */
	TIM_Config();
	/* PWM Configuration */
	PWM_Config(4000);

}

void process_platform() {
	if(getOrderID() != MP.completed_order) {
		if(!(getOrderState()&ORDER_ACTIVE)) {
			switch(getOrderType()) {
				case(ORDER_TYPE_FORWARD):
					orderStartForward();
					break;
				case(ORDER_TYPE_LEFT_TURN):
					orderStartLeftTurn();
					break;
				case(ORDER_TYPE_RIGHT_TURN):
					orderStartRightTurn();
					break;
			}
		} else {
			if(!(MP._rotary_driver_state&ROTARY_DRIVER_ACTIVE)) {
				//<=290

				if(MP._state == PLATFORM_FORWARD) {
					if(!checkFrontRight() && (getOrderCurrentTicks() >= getOrderTargetTicks()*0.4)) {
						deactivatePID();
						set_forward(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
					}
					if(getOrderTargetTicks() <= cmtoticks(25)) {
						deactivatePID();
						set_forward(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
						if(getOrderCurrentTicks() >= getOrderTargetTicks()) {
							set_stop();
							setOrderDone();
						}
					} else {
						if(getOrderCurrentTicks() >= getOrderTargetTicks()*0.7) {
							if((!checkBackRight() && !checkFrontRight()) || !checkBackLeft()) {
								set_stop();
								if(getOrderLengthToWall() == 9.5) {
									rotaryDriverStartCM(4);
								} else {
									rotaryDriverStartCM(13);
								}

							}
						}
					}
				}

				if(MP._state == PLATFORM_LEFT || MP._state == PLATFORM_RIGHT) {
					if(getOrderCurrentTicks() >= getOrderTargetTicks()) {
						set_stop();
						setOrderDone();
					}
				}
			} else {
				if(rotaryDriverDone()) {
					set_stop();
					setOrderDone();
					rotaryDriverStop();
				}
			}
			}
		}

	if(change==1){
		switch(MP._state){
			case(PLATFORM_STOP):
				_stop();
				break;
			case(PLATFORM_FORWARD):
				_go_forward();
				break;
			case(PLATFORM_BACKWARD):
				_go_backward();
				break;
			case(PLATFORM_LEFT):
				_turn_left();
				break;
			case(PLATFORM_RIGHT):
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
	MP._originalLspeed=ls;
	MP._originalRspeed=rs;
	MP._state = PLATFORM_FORWARD;
	if(MP._state == PLATFORM_FORWARD) {
		return 0;
	}
	return -1;
}
int set_backward(int ls, int rs) {
	change=1;
	MP._Lspeed = ls;
	MP._Rspeed = rs;
	MP._originalLspeed=ls;
	MP._originalRspeed=rs;
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
	MP._originalLspeed=ls;
	MP._originalRspeed=rs;
	MP._state = PLATFORM_LEFT;
	if(MP._state == PLATFORM_LEFT) {
		return 0;
	}
	return -1;
}
int set_right(int ls, int rs) {
	change=1;
	MP._Lspeed = ls;
	MP._Rspeed = rs;
	MP._originalLspeed=ls;
	MP._originalRspeed=rs;
	MP._state = PLATFORM_RIGHT;
	if(MP._state == PLATFORM_RIGHT) {
		return 0;
	}
	return -1;
}
int set_stop() {
	change=1;
	MP._state = PLATFORM_STOP;
	if(MP._state == PLATFORM_STOP) {
		return 0;
	}
	return -1;
}

void _go_forward(void) {
	_stop();
	PWM_SetDC(PLATFORM_LEFT_FORWARD ,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(PLATFORM_RIGHT_FORWARD ,40*MP._Rspeed*MP._right_side._calibrate_speed);
}

void _go_backward(void) {
	_stop();
	PWM_SetDC(PLATFORM_LEFT_BACKWARD,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(PLATFORM_RIGHT_BACKWARD,40*MP._Rspeed*MP._right_side._calibrate_speed);

}

void _turn_left(void) {
	_stop();
	PWM_SetDC(PLATFORM_LEFT_BACKWARD ,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(PLATFORM_RIGHT_FORWARD ,40*MP._Rspeed*MP._right_side._calibrate_speed);

}

void _turn_right(void) {
	_stop();
	PWM_SetDC(PLATFORM_LEFT_FORWARD ,40*MP._Lspeed*MP._left_side._calibrate_speed);
	PWM_SetDC(PLATFORM_RIGHT_BACKWARD ,40*MP._Rspeed*MP._right_side._calibrate_speed);
}


void _stop(void) {
	PWM_SetDC(1,0);
	PWM_SetDC(2,0);
	PWM_SetDC(3,0);
	PWM_SetDC(4,0);

}

void startForward(int distance, float distanceToWall, int ordNr){
	// 3000i = 2700cm
	// 1cm = 3000/2700 i = 1.111111111 = 10/9
	i=0;
	istop=distance*10/9;
	setPIDValue(distanceToWall+IR_SENSOR_OFFSET);
	activePID();
	set_forward(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
	activateRotary();
	orderNr=ordNr;
}

void orderStartForward(void) {
	setPIDValue((getOrderLengthToWall()/10.0)+IR_SENSOR_OFFSET);
	resetPIDIntergrator();
	if(getOrderTargetTicks() >= cmtoticks(25)) {
		activePID();
	}
	set_forward(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
	//activateRotary();
	setOrderActive();
}

void orderStartLeftTurn(void) {
	set_left(MOTOR_DEFAULT_TURN_SPEED,MOTOR_DEFAULT_TURN_SPEED);
	setOrderTargetTicks(MOTOR_LEFT_TICKS);
	setOrderActive();
}

void orderStartRightTurn(void) {
	set_right(MOTOR_DEFAULT_TURN_SPEED,MOTOR_DEFAULT_TURN_SPEED);
	setOrderTargetTicks(MOTOR_RIGHT_TICKS);
	setOrderActive();
}

void startLeft(){
	i=0;
	istop=210;
	set_left(MOTOR_DEFAULT_SPEED,MOTOR_DEFAULT_SPEED);
}

void startRight(){
	i=0;
	istop=260;
	set_right(MOTOR_DEFAULT_SPEED,MOTOR_DEFAULT_SPEED);
}

void setLeftCalSpeed(float c) {
	MP._left_side._calibrate_speed 	= c;
}

void setRightCalSpeed(float c) {
	MP._right_side._calibrate_speed = c;
}

void platformPID(float left, float right) {
	MP._Lspeed = MP._originalLspeed + left;
	MP._Rspeed = MP._originalLspeed + right;
}

void setChange(int value) {
	change = value;
}

uint8_t getMotorState(void) {
	return MP._state;
}

int isComplete(void){
	return orderComplete;
}


void rotaryDriverStart(uint16_t ticks) {
	rotaryDriverReset();
	set_forward(MP._originalLspeed, MP._originalLspeed);
	MP._rotary_driver_state |= ROTARY_DRIVER_ACTIVE;
	MP.rotary_driver_target_ticks = ticks;
	setChange(1);
}

void rotaryDriverStartCM(uint16_t cm) {
	rotaryDriverReset();
	set_forward(MP._originalLspeed, MP._originalLspeed);
	MP._rotary_driver_state |= ROTARY_DRIVER_ACTIVE;
	MP.rotary_driver_target_ticks = 9.71*(cm)-37.47;
	setChange(1);
}

uint8_t rotaryDriverDone(void) {
	if(MP.rotary_driver_ticks >= MP.rotary_driver_target_ticks) {
		return 1;
	}
	return 0;
}

void rotaryDriverStop(void) {
	rotaryDriverCancel();
}

void rotaryDriverCancel(void) {
	MP._rotary_driver_state &= ~(ROTARY_DRIVER_ACTIVE);
}

void rotaryDriverReset(void) {
	MP._rotary_driver_state = 0;
	MP.rotary_driver_ticks = 0;
	MP.rotary_driver_target_ticks = 0;
}

void rotaryDriverTick(void) {
	if(MP._rotary_driver_state&ROTARY_DRIVER_ACTIVE) {
		MP.rotary_driver_ticks++;
	}
}

float cmtoticks(float cm) {
	return (9.71*(cm)-37.47);
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


/*
				if(getOrderTargetTicks() <= 290) {
					if(getOrderCurrentTicks() >= getOrderTargetTicks()) {
						set_stop();
						deactivatePID();
						setOrderDone();
					}
				} else {
					if(getOrderCurrentTicks() >= getOrderTargetTicks()*0.6) {
						if((!checkBackRight() && !checkFrontRight()) || !checkBackLeft()) {
							set_stop();
							rotaryDriverStartCM(10);
						}
					}
					if(getOrderCurrentTicks() >= getOrderTargetTicks()*0.8) {
						if(!checkRightWall()) {
							if(!checkBackRight()) {
								//deactivatePID();
								set_stop();
								rotaryDriverStartCM(1);
							}
						}
					} else {
						activePID();
					}
					*/
