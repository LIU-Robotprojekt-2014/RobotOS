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

	//Heading
	MP.heading_state = PLATFORM_HEADING_YNEG;
	MP.x_max 		 = 664; //682
	MP.y_max 		 = 308;
	MP.x_pos 		 = 0;
	MP.y_pos 		 = 154;

	//Adjust
	MP.adjust_state = 0;

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
			if(MP._rotary_driver_state&ROTARY_DRIVER_ACTIVE) {
				if(rotaryDriverDone()) {
					deactivatePID();
					set_stop();
					setOrderDone();
					MP.completed_order = getOrderID();
					rotaryDriverStop();
					rotaryDriverReset();
					MP.adjust_state = 0;
				}
			} else {
				if(MP._state == PLATFORM_FORWARD) {
					if(getOrderTargetTicks() < cmtoticks(PLATFORM_PID_THRESHOLD)) {
						deactivatePID();
						//set_forward(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
						if(getOrderCurrentTicks() >= getOrderTargetTicks()) {
							set_stop();
							setOrderDone();
							MP.completed_order = getOrderID();
							MP.adjust_state = 0;
						}
					} else {

						if(!MP.adjust_state&PLATFORM_ADJUST_DONE) {
							if(getOrderLengthToWall() < 7.0) {
								if(getOrderCurrentTicks() >= getOrderTargetTicks()*0.40 && getOrderCurrentTicks() <= getOrderTargetTicks()*0.60) {
									platformFineAdjust();
								}
							}
						}

						if(getOrderCurrentTicks() >= getOrderTargetTicks()*0.6) {

							if(!checkFrontRight()) {
								deactivatePID();
								set_forward(MP._originalLspeed, MP._originalRspeed);
							}

							if((!checkBackRight() && !checkFrontRight()) || (!checkBackLeft() && !checkFrontLeft())) {
								set_stop();
								deactivatePID();
								if(getOrderLengthToWall() >= 7.0) {
									rotaryDriverStartCM(7);
								} else {
									rotaryDriverStartCM(9);
								}

							}
						}
					}
				} else if(MP._state == PLATFORM_LEFT || MP._state == PLATFORM_RIGHT) {
					deactivatePID();
					if(getOrderCurrentTicks() >= getOrderTargetTicks()) {
						set_stop();
						setOrderDone();
						MP.completed_order = getOrderID();
					}
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
void orderStartForward(void) {
	setPIDValue(getOrderLengthToWall()+IR_SENSOR_OFFSET);
	resetPIDIntergrator();

	movePlatformCM(MP.order_length);

	if(isPlatformMovingIntoWall()) {
		setOrderTargetTicks(getOrderTargetTicks()-cmtoticks(5));
	}

	if(getOrderTargetTicks() >= cmtoticks(PLATFORM_PID_THRESHOLD)) {
		if(getOrderLengthToWall() < 8.0) {
			setPIDSmall();
		} else {
			setPIDWide();
		}
		activePID();
	} else {
		deactivatePID();
	}

	if(getOrderLengthToWall() < 8.0 || MP.order_length < 30) {
		set_forward(MOTOR_REDUCED_SPEED, MOTOR_REDUCED_SPEED);
	} else {
		set_forward(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
	}

	//activateRotary();
	setOrderActive();
}
void orderStartLeftTurn(void) {
	set_left(MOTOR_DEFAULT_TURN_SPEED,MOTOR_DEFAULT_TURN_SPEED);
	setOrderTargetTicks(MOTOR_LEFT_TICKS);
	setOrderActive();
	switch(MP.heading_state) {
		case(PLATFORM_HEADING_XNEG):
			MP.heading_state = PLATFORM_HEADING_YNEG;
			break;
		case(PLATFORM_HEADING_XPOS):
			MP.heading_state = PLATFORM_HEADING_YPOS;
			break;
		case(PLATFORM_HEADING_YNEG):
			MP.heading_state = PLATFORM_HEADING_XPOS;
			break;
		case(PLATFORM_HEADING_YPOS):
			MP.heading_state = PLATFORM_HEADING_XNEG;
			break;
	}
}
void orderStartRightTurn(void) {
	set_right(MOTOR_DEFAULT_TURN_SPEED,MOTOR_DEFAULT_TURN_SPEED);
	setOrderTargetTicks(MOTOR_RIGHT_TICKS);
	setOrderActive();
	switch(MP.heading_state) {
		case(PLATFORM_HEADING_XNEG):
			MP.heading_state = PLATFORM_HEADING_YPOS;
			break;
		case(PLATFORM_HEADING_XPOS):
			MP.heading_state = PLATFORM_HEADING_YNEG;
			break;
		case(PLATFORM_HEADING_YNEG):
			MP.heading_state = PLATFORM_HEADING_XNEG;
			break;
		case(PLATFORM_HEADING_YPOS):
			MP.heading_state = PLATFORM_HEADING_XPOS;
			break;
	}
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
	/*
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
	*/
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
	set_forward(MOTOR_DEFAULT_TURN_SPEED, MOTOR_DEFAULT_TURN_SPEED);
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
void setOrderLengthCM(uint16_t length) {
	MP.order_length = length;
}
void changePlatformHeading(uint8_t heading) {
	MP.heading_state = heading;
}
void movePlatformCM(uint16_t length) {
	switch(MP.heading_state) {
		case(PLATFORM_HEADING_XNEG):
				MP.x_pos -= length;
				break;
		case(PLATFORM_HEADING_XPOS):
				MP.x_pos += length;
				break;
		case(PLATFORM_HEADING_YNEG):
				MP.y_pos -= length;
				break;
		case(PLATFORM_HEADING_YPOS):
				MP.y_pos += length;
				break;
	}
}
uint8_t isPlatformMovingIntoWall(void){
	if((MP.x_pos >= MP.x_max) && (MP.heading_state == PLATFORM_HEADING_XPOS)) {
		return 1;
	}
	if ((MP.x_pos <= 0) && (MP.heading_state == PLATFORM_HEADING_XNEG)) {
		return 1;
	}
	return 0;
}
uint8_t isInOuterLane(void){
	return 0;
}

void platformFineAdjust(void) {
	float diff = 0.0;
	uint16_t ticks = getOrderCurrentTicks();
	if(checkRightWall()) {
		//_stop();
		process_sensors();
		diff = getIRSensorReadingCM(IR_SENSOR_RF)-getIRSensorReadingCM(IR_SENSOR_RB);

		if(diff <= -MOTOR_ADJUSTMENT_CM || diff >= MOTOR_ADJUSTMENT_CM) {
			while(diff <= -MOTOR_ADJUSTMENT_CM || diff >= MOTOR_ADJUSTMENT_CM) {
				process_sensors();

				if(!checkRightWall()) {
					break;
				}

				diff = getIRSensorReadingCM(IR_SENSOR_RF)-getIRSensorReadingCM(IR_SENSOR_RB);
				if(diff >= MOTOR_ADJUSTMENT_CM ) {
					_stop();
					PWM_SetDC(PLATFORM_LEFT_FORWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._left_side._calibrate_speed);
					PWM_SetDC(PLATFORM_RIGHT_BACKWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._right_side._calibrate_speed);
				} else if ( diff <= -MOTOR_ADJUSTMENT_CM) {
					_stop();
					PWM_SetDC(PLATFORM_LEFT_BACKWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._left_side._calibrate_speed);
					PWM_SetDC(PLATFORM_RIGHT_FORWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._right_side._calibrate_speed);
				} else {
					break;
				}

			}
			MP.adjust_state |= PLATFORM_ADJUST_DONE;
			setOrderCurrentTicks(ticks);
			resetPIDIntergrator();
			_stop();
			setChange(1);
			deactivatePID();
		}

	}
	/*
	else if(checkLeftWall()) {
		_stop();
		process_sensors();
		diff = getIRSensorReadingCM(IR_SENSOR_LF)-getIRSensorReadingCM(IR_SENSOR_LB);
		while(diff < -MOTOR_ADJUSTMENT_CM || diff > MOTOR_ADJUSTMENT_CM) {
			process_sensors();
			if(!checkLeftWall()) {
				break;
			}
			diff = getIRSensorReadingCM(IR_SENSOR_RF)-getIRSensorReadingCM(IR_SENSOR_RB);
			if(diff > MOTOR_ADJUSTMENT_CM ) {
				PWM_SetDC(PLATFORM_LEFT_BACKWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._left_side._calibrate_speed);
				PWM_SetDC(PLATFORM_RIGHT_FORWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._right_side._calibrate_speed);
			} else if( diff < -MOTOR_ADJUSTMENT_CM) {
				PWM_SetDC(PLATFORM_LEFT_FORWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._left_side._calibrate_speed);
				PWM_SetDC(PLATFORM_RIGHT_BACKWARD ,40*MOTOR_ADJUSTMENT_SPEED*MP._right_side._calibrate_speed);
			}
		}
		_stop();
	}
	*/

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

void tickCurrentDelayMS(){
	currentDelayMS++;
}

void platformDelay(int targetDelayMS){
		currentDelayMS=0;
		while(currentDelayMS < targetDelayMS){

		}
}
