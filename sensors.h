#ifndef SENSORS_H
#define SENSORS_H

#define IR_SENSORS 4

typedef struct Infrared {
	float _latest_reading;
	float _calibration;
} Infrared;

typedef struct Rotary {
	double _accumulated_distance;
	double _total_accumulated_distance;
	float _calibration;
} Rotary;


typedef struct Sensors {
	Infrared IR[IR_SENSORS];
	Rotary ROT;
} Sensors;




void init_sensors(void);
void init_sensors2(void);
void process_sensors(void);
void getVal(uint32_t arr[]);



#endif
