#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"
#include "PID.h"
#include "sensors.h"
#include "order.h"
#include "bluetooth.h"
#include <stm32f4xx_tim.h>
#include <stdio.h>
#include <stm32f4xx_tim.h>
#include <math.h>

PID PID_Motor;

PID_Params PID_P_Small;
PID_Params PID_P_Wide;

void init_PID(void) {
	PID_Motor._input			= 0;
	PID_Motor._value 			= 20;
	PID_Motor._interval 		= 10;
	PID_Motor._Kp 				= 10;
	PID_Motor._Ki 				= 1;
	PID_Motor._Kd 				= 0;

	PID_Motor._error 			= 0;
	PID_Motor._previous_error 	= 0;
	PID_Motor._proportional	 	= 0;
	PID_Motor._integrator 		= 0;
	PID_Motor._derivator 		= 0;
	PID_Motor._state 			= 0;
	PID_Motor._timer_count 		= 0;
	PID_Motor.output			= 0;
	PID_Motor.values_to_mean 	= 0;

	PID_P_Small._Kp = 6;
	PID_P_Small._Ki = 0.1;
	PID_P_Small._Kd = 0.1;

	PID_P_Wide._Kp = 15;
	PID_P_Wide._Ki = 0.1;
	PID_P_Wide._Kd = 1;
	setupTimer(PID_Motor._interval);
}

void process_PID(void) {
	if(PID_Motor._state&PID_ACTIVE) {
		if(PID_Motor._state&PID_TIMER_DONE) {
			PID_Motor._state &= ~(PID_TIMER_DONE);
			calculatePID();
			if(PID_Motor.output > 0){
				platformPID(-PID_Motor.output,0);
			} else if(PID_Motor.output < 0){
				platformPID(0,PID_Motor.output);
			}

			setChange(1);
			resetInput();
			PID_Motor.output = 0;
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
	PID_Motor._value = val;
}
void addInputValue(float value) {
	PID_Motor._input += value;
	PID_Motor.values_to_mean++;
}
void resetInput(void) {
	PID_Motor._input = 0;
	PID_Motor.values_to_mean = 0;
}
void calculatePID(void) {
	if(PID_Motor._state&PID_RESET_INTEGRATOR) {
		PID_Motor._state &= ~(PID_RESET_INTEGRATOR);
		PID_Motor._integrator = 0;
	}

	if(checkFrontRight()) {
		PID_Motor._input = getIRSensorReadingCM(IR_SENSOR_RF);
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
	} else {
		PID_Motor.output = 0;
		//return;
	}


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

void resetPIDIntergrator(void) {
	PID_Motor._state |= PID_RESET_INTEGRATOR;
}

void setPIDParameters(float Kp,float Ki, float Kd) {
	PID_Motor._Kp = Kp;
	PID_Motor._Ki = Ki;
	PID_Motor._Kd = Kd;
}

void setPIDWideParameters(float Kp,float Ki, float Kd) {
	PID_P_Wide._Kp = Kp;
	PID_P_Wide._Ki = Ki;
	PID_P_Wide._Kd = Kd;
}

void setPIDSmallParameters(float Kp,float Ki, float Kd) {
	PID_P_Small._Kp = Kp;
	PID_P_Small._Ki = Ki;
	PID_P_Small._Kd = Kd;
}

void setPIDWide(void) {
	PID_Motor._Kp = PID_P_Wide._Kp;
	PID_Motor._Ki = PID_P_Wide._Ki;
	PID_Motor._Kd = PID_P_Wide._Kd;
}

void setPIDSmall(void) {
	PID_Motor._Kp = PID_P_Small._Kp;
	PID_Motor._Ki = PID_P_Small._Ki;
	PID_Motor._Kd = PID_P_Small._Kd;
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
		/*
		rot_ms++;
		if(rot_ms >= ROT_MS) {
			//sendRotaryTick();
			rot_ms = 0;
		}
		*/
		/*
		ir_ms++;
		if(ir_ms >= IR_MS) {
			sendIRSensors();
			ir_ms = 0;
		}
		*/
		tickOrderDelay();

		//Take sample every 10ms
		if(PID_Motor._state&PID_ACTIVE) {
			PID_Motor._timer_count++;
			//PID_Motor._state |= PID_TAKE_SAMPLE;
			if(PID_Motor._timer_count >= PID_Motor._interval) {
				PID_Motor._state |= PID_TIMER_DONE;
				PID_Motor._state |= PID_RESET_TIMER_COUNT;
			}
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
