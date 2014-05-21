#ifndef PID_H
#define PID_H

#define PID_TIMER_DONE 0x01
#define PID_RESET_TIMER_COUNT 0x02
#define PID_RESET_INTEGRATOR 0x04
#define PID_TAKE_SAMPLE 0x08
#define PID_ACTIVE	0x10

#define DEFAULT_WALL_DISTANCE 13
#define WALL_DISTANCE_MINIMUM 1
#define WALL_DISTANCE_MAXIMUM 9999

#define PID_LOWER_LIMIT -60
#define PID_UPPER_LIMIT 60

#define PID_SAMPLE_TIME 0.01
#define PID_MINIMUM_ERROR 1.0

#define PID_ANGULAR_SENSOR_DIST 9

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
	uint16_t values_to_mean;
	float output;
	uint8_t lost_found_state;
} PID;

typedef struct PID_Params {
	float _Kp;
	float _Ki;
	float _Kd;
} PID_Params;


void init_PID(void);
void _init_PID_Angular(void);
void _init_PID_Distance(void);
void process_PID(void);
void setupTimer(float interval);
void setPIDValue(float val);
void addInputValue(float value);
void resetInput(void);
void calculatePID(void);
void activePID(void);
void deactivatePID(void);
int16_t getPIDOutput(void);
void resetPIDIntergrator(void);

void setPIDParameters(float Kp,float Ki, float Kd);
void setPIDWideParameters(float Kp,float Ki, float Kd);
void setPIDSmallParameters(float Kp,float Ki, float Kd);
void setPIDWide(void);
void setPIDSmall(void);

#endif
