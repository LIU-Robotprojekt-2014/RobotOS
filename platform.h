#ifndef PLATFORM_H
#define PLATFORM_H

#ifndef FLIP_MOTORS
#define PLATFORM_LEFT_FORWARD 1
#define PLATFORM_RIGHT_FORWARD 4
#define PLATFORM_LEFT_BACKWARD 3
#define PLATFORM_RIGHT_BACKWARD 2
#else
#define PLATFORM_LEFT_FORWARD 3
#define PLATFORM_RIGHT_FORWARD 2
#define PLATFORM_LEFT_BACKWARD 1
#define PLATFORM_RIGHT_BACKWARD 4
#endif

#define MOTOR_DEFAULT_SPEED 100
#define MOTOR_DEFAULT_TURN_SPEED 100
#define MOTOR_REDUCED_SPEED 85
#define MOTOR_ADJUSTMENT_SPEED 30

#define MOTOR_ADJUSTMENT_CM 1

#define PLATFORM_PID_THRESHOLD 30

//TODO: RECALIBRATE (GO LOWER)
//#define MOTOR_LEFT_TICKS 255
//#define MOTOR_RIGHT_TICKS 270
//#define MOTOR_LEFT_TICKS 258
//#define MOTOR_RIGHT_TICKS 290
#define MOTOR_LEFT_TICKS 235
#define MOTOR_RIGHT_TICKS 230
//Good right turn 246, 240 for "under turn"

#define PLATFORM_STOP 0
#define PLATFORM_FORWARD 1
#define PLATFORM_BACKWARD 2
#define PLATFORM_LEFT 3
#define PLATFORM_RIGHT 4

//Platform moving states
#define PLATFORM_HEADING_XPOS 1
#define PLATFORM_HEADING_XNEG 2
#define PLATFORM_HEADING_YPOS 3
#define PLATFORM_HEADING_YNEG 4

#define ROTARY_DRIVER_ACTIVE 0x01
#define ROTARY_DRIVER_NODE_TICK 80

#define PLATFORM_ADJUST_DONE 0x01

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
   uint8_t _originalLspeed;
   uint8_t _originalRspeed;
   uint8_t _flags; //
   Motor _left_side;
   Motor _right_side;

   uint8_t _rotary_driver_state;
   uint16_t rotary_driver_ticks;
   uint16_t rotary_driver_target_ticks;

   uint16_t order_length;

   uint8_t order_state;
   uint16_t current_order; //Set to 0 when done
   uint16_t completed_order;

   //Software heading
   uint16_t x_pos;
   uint16_t y_pos;
   uint16_t x_max;
   uint16_t y_max;
   uint8_t heading_state;

   //Adjust
   uint8_t adjust_state;

} MotorPlatform;


int orderComplete;
int orderNr;
int timer;
int currentDelayMS;

void TIM_Config(void);
void PWM_Config(int period);
void PWM_SetDC(uint16_t channel,uint16_t dutycycle);
void init_platform(void);
//void init_PID(void);
//void doPID(void);
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

void orderStartForward(void);
void orderStartLeftTurn(void);
void orderStartRightTurn(void);

void setLeftCalSpeed( float c);
void setRightCalSpeed( float c);

void platformPID(float left, float right);
void setChange(int value);

uint8_t getMotorState(void);

int isComplete(void);

void rotaryDriverStart(uint16_t ticks);
void rotaryDriverStartCM(uint16_t cm);
uint8_t rotaryDriverDone(void);
void rotaryDriverStop(void);
void rotaryDriverCancel(void);
void rotaryDriverReset(void);
void rotaryDriverTick(void);

void InitializeLEDs();

float cmtoticks(float cm);

void setOrderLengthCM(uint16_t length);
void changePlatformHeading(uint8_t heading);
void movePlatformCM(uint16_t length);
uint8_t isPlatformMovingIntoWall(void);
uint8_t isInOuterLane(void);

void platformFineAdjust(void);

void platformDelay(int targetDelayMS);
void tickCurrentDelayMS();

#endif
