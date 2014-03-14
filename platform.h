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
   uint8_t _speed; //Private (1-100)
   uint8_t _flags; //
   Motor _left_side;
   Motor _right_side;
} MotorPlatform;



void init_platform(void);
void go_forward(void);
void go_backward(void);
void turn_left(void);
void turn_right(void);
void stop(void);

#endif
