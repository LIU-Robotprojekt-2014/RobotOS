#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"
#include "PID.h"
#include "sensors.h"
#include <stm32f4xx_tim.h>
#include <stdio.h>
#include <stm32f4xx_tim.h>

PID PID_Motor;

void init_PID(void) {
	PID_Motor._input			= 0;
	PID_Motor._value 			= 20;
	PID_Motor._interval 		= 100;
	PID_Motor._Kp 				= 5;
	PID_Motor._Ki 				= 1;
	PID_Motor._Kd 				= 15;
	PID_Motor._error 			= 0;
	PID_Motor._previous_error 	= 0;
	PID_Motor._proportional	 	= 0;
	PID_Motor._integrator 		= 0;
	PID_Motor._derivator 		= 0;
	PID_Motor._state 			= 0;
	PID_Motor._timer_count 		= 0;
	PID_Motor.output			= 0;
	setupTimer(PID_Motor._interval);
}


void process_PID(void) {
	/*
	if(!getMotorState()&PLATFORM_FORWARD) {
		PID_Motor._state &= ~(PID_ACTIVE);
	} else {
		PID_Motor._state |= PID_ACTIVE;
	}
	*/

	if(PID_Motor._state&PID_ACTIVE) {
		if(PID_Motor._state&PID_TAKE_SAMPLE) {
			PID_Motor._state &= ~(PID_TAKE_SAMPLE);
			addInputValue(HFsensor);
		}

		if(PID_Motor._state&PID_TIMER_DONE) {
			PID_Motor._state &= ~(PID_TIMER_DONE);
			calculatePID();

			//DEBUG
			if(PID_Motor.output > 0){
				platformPID(-PID_Motor.output,0);
			}
			if(PID_Motor.output < 0){
				platformPID(0,PID_Motor.output);
			}
			setChange(1);
			resetInput();
		}
	}
}

void setupTimer(float interval) {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 100; // 1 MHz down to 1 KHz (1 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 78 ; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void setPIDValue(float val) {
	if(val >= WALL_DISTANCE_MINIMUM && val <= WALL_DISTANCE_MAXIMUM) {
		PID_Motor._value = val;
	} else {
		PID_Motor._value = DEFAULT_WALL_DISTANCE;
	}
}

int ab = 0;
void addInputValue(float value) {
	PID_Motor._input += value/10;
	ab++;
}

void resetInput(void) {
	PID_Motor._input = 0;
	ab = 0;
}

void calculatePID(void) {
	/*
	Error = Set_Distance - Range
	if(abs(Error) < 1) {
	    Error = 0
	}
	P = Error
	I = I + Error*0.1
	D = (Error - Error_prev)/0.1

	PID_value = (Kp*P) + (Ki*I) + (Kd*D)
	Error_prev = Error
	#Thruster = Error + PID_value
	if(PID_value > 16000) {
	    PID_value = 16000
	} elseif (PID_value < -16000) {
	    PID_value = -16000
	}
	Thruster = PID_value
	*/
	if(PID_Motor._state&PID_RESET_INTEGRATOR) {
		PID_Motor._state &= ~(PID_RESET_INTEGRATOR);
		PID_Motor._integrator = 0;
	}


	PID_Motor._error = PID_Motor._value - PID_Motor._input;

	if(abs(PID_Motor._error) < PID_MINIMUM_ERROR) {
		PID_Motor._error = 0;
	}

	PID_Motor._proportional	= PID_Motor._error;
	PID_Motor._integrator 	+= PID_Motor._error*PID_SAMPLE_TIME;
	PID_Motor._derivator 	= (PID_Motor._error-PID_Motor._previous_error)/PID_SAMPLE_TIME;

	PID_Motor.output = PID_Motor._Kp*PID_Motor._proportional;
	PID_Motor.output += PID_Motor._Ki*PID_Motor._integrator;
	PID_Motor.output += PID_Motor._Kd*PID_Motor._derivator;

	if(PID_Motor.output >= PID_UPPER_LIMIT) {
		PID_Motor.output = PID_UPPER_LIMIT;
	} else if(PID_Motor.output <= PID_LOWER_LIMIT) {
		PID_Motor.output = PID_LOWER_LIMIT;
	}

	PID_Motor._previous_error = PID_Motor._error;
}

void activePID(void) {
	PID_Motor._state |= PID_ACTIVE;
}
void deactivatePID(void) {
	PID_Motor._state &= ~(PID_ACTIVE);
}

int16_t getPIDOutput(void) {
	return PID_Motor.output;
}

//TODO: Temp for rotary timer
uint32_t rot_ms = 0;
#define ROT_MS 1000

uint32_t ir_ms = 0;
#define IR_MS 1500

//Interrupt routines
void TIM2_IRQHandler(void) {
	if(PID_Motor._state&PID_RESET_TIMER_COUNT) {
			PID_Motor._state &= ~(PID_RESET_TIMER_COUNT);
			PID_Motor._timer_count = 0;
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		PID_Motor._timer_count++;


		rot_ms++;
		if(rot_ms >= ROT_MS) {
			sendRotaryTick();
			rot_ms = 0;
		}

		ir_ms++;
		if(ir_ms >= IR_MS) {
			//sendIRSensors();
			ir_ms = 0;
		}

		//Take sample every 10ms

		if(PID_Motor._timer_count%10 == 0) {
			PID_Motor._state |= PID_TAKE_SAMPLE;
		}


		//Take sample every ms
		//PID_Motor._state |= PID_TAKE_SAMPLE;

		if(PID_Motor._timer_count >= PID_Motor._interval) {
			PID_Motor._state |= PID_TIMER_DONE;
			PID_Motor._state |= PID_RESET_TIMER_COUNT;
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
