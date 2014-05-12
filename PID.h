#ifndef PID_H
#define PID_H

#define PID_TIMER_DONE 0x01
#define PID_RESET_TIMER_COUNT 0x02
#define PID_RESET_INTEGRATOR 0x04
#define PID_TAKE_SAMPLE 0x08
#define PID_ACTIVE	0x10

#define DEFAULT_WALL_DISTANCE 10
#define WALL_DISTANCE_MINIMUM 1
#define WALL_DISTANCE_MAXIMUM 9999

#define PID_LOWER_LIMIT -70
#define PID_UPPER_LIMIT 70

#define PID_SAMPLE_TIME 0.1
#define PID_MINIMUM_ERROR 1

typedef struct PID {
	float _input;
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
	uint16_t _timer_count;
	float output;

} PID;

void init_PID(void);
void process_PID(void);
void setupTimer(float interval);
void setPIDValue(float val);
void addInputValue(float value);
void resetInput(void);
void calculatePID(void);

#endif
