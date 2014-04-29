#ifndef PID_H
#define PID_H

#define PID_TIMER_DONE 0x01
#define PID_RESET_TIMER 0x02
#define PID_RESET_INTEGRATOR 0x04
#define PID_RESET_PID 0x08

typedef struct PID {
	float _value; //Desirable value
	float _interval; //Sampling interval for timer
	float _Kp;
	float _Ki;
	float _Kd;
	float _error;
	float _previous_error;
	float _proportional;
	float _integrator;
	float _derivator;
	uint8_t _state; //Private
} PID;

void init_PID(void);
void process_PID(void);
void setupTimer(void);
void setValue(float val);

#endif
