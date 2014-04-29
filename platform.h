#ifndef PLATFORM_H
#define PLATFORM_H

typedef struct Motor {
	float _calibrate_speed;
	uint8_t _state; //Private
	uint16_t _forward_pin;
	uint16_t _backward_pin;
} Motor;

typedef struct MotorPlatform {
   uint8_t _state; //Private
   uint8_t _Lspeed; //Private (1-100)
   uint8_t _Rspeed;
   uint8_t _flags; //
   Motor _left_side;
   Motor _right_side;
} MotorPlatform;


void TIM_Config(void);
void PWM_Config(int period);
void PWM_SetDC(uint16_t channel,uint16_t dutycycle);
void init_platform(void);
void process_platform();
int set_forward(int ls, int rs);
int set_backward(int ls, int rs);
int set_left(int ls, int rs);
int set_right(int ls, int rs);
int set_stop();
void _go_forward(void);
void _go_backward(void);
void _turn_left(void);
void _turn_right(void);
void _stop(void);
void setLeftCalSpeed( float c);
void setRightCalSpeed( float c);
void InitializeLEDs();

#endif
