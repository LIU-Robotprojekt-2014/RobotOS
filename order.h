#ifndef ORDER_H
#define ORDER_H

#define ORDER_TYPE_FORWARD 0x01
#define ORDER_TYPE_LEFT_TURN 0x02
#define ORDER_TYPE_RIGHT_TURN 0x04

#define ORDER_ACTIVE 0x01
#define ORDER_USE_ROTARY_DRIVER 0x02
#define ORDER_DONE 0x04

typedef struct Order {
	uint8_t state;
	uint8_t order_type;
	uint16_t orderID;
	uint16_t target_ticks;
	uint16_t current_ticks;
	float length_to_wall;
} Order;

void setOrderTypeForward(void);
void setOrderTypeLeftTurn(void);
void setOrderTypeRightTurn(void);
uint8_t getOrderType(void);
void setOrderID(uint16_t order);
void setOrderTargetTicks(uint16_t len);
uint16_t getOrderID(void);
uint8_t checkOrderDone(void);
uint8_t checkOrderActive(void);
void resetOrder(void);
void setOrderDone(void);
void setOrderActive(void);
void setOrderInactive(void);
void setOrderLengthToWall(float len);
float getOrderLengthToWall(void);
uint8_t getOrderState(void);
uint16_t getOrderTargetTicks(void);
uint16_t getOrderCurrentTicks(void);
void tickOrder(void);


#endif
