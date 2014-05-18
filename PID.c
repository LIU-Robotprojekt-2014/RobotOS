#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "platform.h"
#include "PID.h"
#include "sensors.h"
#include "order.h"
#include <stm32f4xx_tim.h>
#include <stdio.h>
#include <stm32f4xx_tim.h>
#include <math.h>

PID PID_Motor;
PID PID_Angular;
PID PID_Distance;

float PID_Output_Sum = 0;

void init_PID(void) {
	/*
	 * "Good"
	 * P = 2
	 * I = 0.01
	 * D = 5
	 */
	PID_Motor._input			= 0;
	PID_Motor._value 			= 20;
	PID_Motor._interval 		= 10;
	PID_Motor._Kp 				= 6;
	PID_Motor._Ki 				= 0;
	PID_Motor._Kd 				= 10;
	PID_Motor._error 			= 0;
	PID_Motor._previous_error 	= 0;
	PID_Motor._proportional	 	= 0;
	PID_Motor._integrator 		= 0;
	PID_Motor._derivator 		= 0;
	PID_Motor._state 			= 0;
	PID_Motor._timer_count 		= 0;
	PID_Motor.output			= 0;
	PID_Motor.values_to_mean 	= 0;

	_init_PID_Angular();

	setupTimer(PID_Motor._interval);
}

void _init_PID_Angular(void) {
	PID_Angular._input			= 0;
	PID_Angular._value 			= 0;
	PID_Angular._interval 		= 10;
	PID_Angular._Kp 			= 10;
	PID_Angular._Ki 			= 1;
	PID_Angular._Kd 			= 1;
	PID_Angular._error 			= 0;
	PID_Angular._previous_error	= 0;
	PID_Angular._proportional 	= 0;
	PID_Angular._integrator		= 0;
	PID_Angular._derivator 		= 0;
	PID_Angular._state 			= 0;
	PID_Angular._timer_count 	= 0;
	PID_Angular.output			= 0;
	PID_Angular.values_to_mean 	= 0;
}
void _init_PID_Distance(void) {

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

		//checkFrontRight()&&checkBackRight()
		if(checkRightWall()) {
			/*
			if(PID_Motor._state&PID_TAKE_SAMPLE) {
				PID_Motor._state &= ~(PID_TAKE_SAMPLE);

				if(checkBackRight()) {
					addInputValue(getIRSensorReading(IR_SENSOR_RB));
				} else if (checkFrontRight()) {
					addInputValue(getIRSensorReading(IR_SENSOR_RF));
				}


			}

			if(checkBackRight()) {
				addInputValue(getIRSensorReading(IR_SENSOR_RB));
			} else if (checkFrontRight()) {
				addInputValue(getIRSensorReading(IR_SENSOR_RF));
			}
			*/



			if(PID_Motor._state&PID_TIMER_DONE) {
				PID_Motor._state &= ~(PID_TIMER_DONE);

				if(checkBackRight() && checkFrontRight()) {
					calculatePIDAngular();
				}
				calculatePID();

				PID_Output_Sum = PID_Motor.output+PID_Angular.output;
				if(PID_Output_Sum > PID_UPPER_LIMIT) {
					PID_Output_Sum = PID_UPPER_LIMIT;
				} else if (PID_Output_Sum < PID_LOWER_LIMIT) {
					PID_Output_Sum = PID_LOWER_LIMIT;
				}


				//DEBUG
				if(PID_Output_Sum > 0){
					platformPID(-PID_Output_Sum,0);
				}
				if(PID_Output_Sum < 0){
					platformPID(0,PID_Output_Sum);
				}
				setChange(1);
				resetInput();
				PID_Motor.output = 0;
				PID_Angular.output = 0;
			}
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

	//PID_Motor._input /= PID_Motor.values_to_mean;
	if(checkBackRight()) {
		PID_Motor._error = PID_Motor._value - getIRSensorReadingCM(IR_SENSOR_RB) + IR_SENSOR_OFFSET;
	} else if (checkFrontRight()) {
		PID_Motor._error = PID_Motor._value - getIRSensorReadingCM(IR_SENSOR_RF) + IR_SENSOR_OFFSET;
	}

	//PID_Motor._error = PID_Motor._value - PID_Motor._input;

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


void calculatePIDAngular(void) {
	if(PID_Angular._state&PID_RESET_INTEGRATOR) {
		PID_Angular._state &= ~(PID_RESET_INTEGRATOR);
		PID_Angular._integrator = 0;
	}

	//PID_Angular._input /= PID_Angular.values_to_mean;
	PID_Angular._error = getIRSensorReading(IR_SENSOR_RF)-getIRSensorReading(IR_SENSOR_RB);
	//PID_Angular._error = LatestReading[IR_SENSOR_RB]-LatestReading[IR_SENSOR_RF];
	//PID_Angular._error = PID_Angular._value - PID_Angular._input;

	if(abs(PID_Angular._error) < 2) {
		PID_Angular._error = 0;
	}

	PID_Angular._proportional	= PID_Angular._error;
	PID_Angular._integrator 	+= PID_Angular._error*PID_SAMPLE_TIME;
	PID_Angular._derivator 	= (PID_Angular._error-PID_Angular._previous_error)/PID_SAMPLE_TIME;

	PID_Angular.output = PID_Angular._Kp*PID_Angular._proportional;
	PID_Angular.output += PID_Angular._Ki*PID_Angular._integrator;
	PID_Angular.output += PID_Angular._Kd*PID_Angular._derivator;

	if(PID_Angular.output >= PID_UPPER_LIMIT) {
		PID_Angular.output = PID_UPPER_LIMIT;
	} else if(PID_Angular.output <= PID_LOWER_LIMIT) {
		PID_Angular.output = PID_LOWER_LIMIT;
	}
	PID_Angular._previous_error = PID_Angular._error;
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
