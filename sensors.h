#ifndef SENSORS_H
#define SENSORS_H

#define IR_SENSORS 4
#define IR_SENSOR_LF 2
#define IR_SENSOR_LB 0
#define IR_SENSOR_RF 3
#define IR_SENSOR_RB 1

#define IR_SENSOR_LOWER_LIMIT 8
#define IR_SENSOR_UPPER_LIMIT 60

#define IR_SENSOR_OFFSET 12

#define IR_SENSOR_WALL_RIGHT_LIMIT 30 //Below value: got wall. Above value: no wall
#define IR_SENSOR_WALL_LEFT_LIMIT 40  //Below value: got wall. Above value: no wall
#define IR_SENSOR_GOT_WALL_RF 0x01 //Right front
#define IR_SENSOR_GOT_WALL_RB 0x02 //Right back
#define IR_SENSOR_GOT_WALL_LF 0x04 //Left front
#define IR_SENSOR_GOT_WALL_LB 0x08 //Left back
#define IR_SENSOR_SEND 0x10

#define DISABLE_LF


#define IR_SENSOR_STATE_GOT_WALL 0x01

#define ROTARY_SEND 0x01
#define ROTARY_CLEAR 0x02
#define ROTARY_ACTIVE 0x04



typedef struct Infrared {
	float _latest_reading;
	float _calibration;
	uint8_t state;
} Infrared;

typedef struct Rotary {
	double _accumulated_distance;
	double _total_accumulated_distance;
	float _calibration;
	uint8_t state;
	uint32_t tick;
	char _message_buffer[60];
} Rotary;


typedef struct Sensors {
	Infrared IR[IR_SENSORS];
	Rotary ROT;
	uint8_t ir_state;
} Sensors;


void init_sensors(void);
void init_rotary(void);
void setupIR(void);
void setupRotary(void);
void process_sensors(void);
void getVal(uint32_t arr[]);
float getIRSensorReading(uint8_t sensorID);
float getIRSensorReadingCM(uint8_t sensorID);
void checkWallDistance(void);
char* toArray(int number);
char* revStr(char *str);

void sendIRSensors(void);
void sendRotaryTick(void);

void activateRotary(void);
void deactivateRotarty(void);

uint8_t checkFrontLeft(void);
uint8_t checkFrontRight(void);
uint8_t checkBackRight(void);
uint8_t checkBackLeft(void);
uint8_t checkLeftWall(void);
uint8_t checkRightWall(void);

void EXTI0_IRQHandler(void);
uint32_t i;
uint32_t itot;
float istop;
float HFsensor;
float HBsensor;
float wall;
#endif
