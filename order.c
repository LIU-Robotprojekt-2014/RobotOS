#include <stm32f4xx.h>
#include <misc.h>
#include <stdio.h>
#include <stdlib.h>
#include "order.h"

Order RobotOrder;

void setOrderTypeForward(void) {
	RobotOrder.order_type = ORDER_TYPE_FORWARD;
}

void setOrderTypeLeftTurn(void) {
	RobotOrder.order_type = ORDER_TYPE_LEFT_TURN;
}

void setOrderTypeRightTurn(void) {
	RobotOrder.order_type = ORDER_TYPE_RIGHT_TURN;
}

uint8_t getOrderType(void) {
	return RobotOrder.order_type;
}

void setOrderID(uint16_t order) {
	RobotOrder.orderID = order;
}

void setOrderTargetTicks(uint16_t len) {
	RobotOrder.target_ticks = len;
}

uint16_t getOrderID(void) {
	return RobotOrder.orderID;
}

uint8_t checkOrderDone(void) {
	if(RobotOrder.state&ORDER_ACTIVE) {
		if(RobotOrder.state&ORDER_DONE) {
			return 1;
		}
	}
	return 0;
}

uint8_t checkOrderActive(void) {
	if(RobotOrder.state&ORDER_ACTIVE) {
		return 1;
	}
	return 0;
}

void resetOrder(void) {
	RobotOrder.current_ticks 	= 0;
	RobotOrder.target_ticks		= 0;
	RobotOrder.orderID 			= 0;
	RobotOrder.order_type 		= 0;
	RobotOrder.state 			= 0;
	RobotOrder.length_to_wall	= 0;
}

void setOrderDone(void) {
	RobotOrder.state |= ORDER_DONE;
}

void setOrderActive(void) {
	RobotOrder.state |= ORDER_ACTIVE;
}

void setOrderInactive(void) {
	RobotOrder.state &= ~(ORDER_ACTIVE);
}

void setOrderLengthToWall(float len) {
	RobotOrder.length_to_wall = len;
}

float getOrderLengthToWall(void) {
	return RobotOrder.length_to_wall;
}

uint8_t getOrderState(void) {
	return RobotOrder.state;
}

uint16_t getOrderTargetTicks(void) {
	return RobotOrder.target_ticks;
}

uint16_t getOrderCurrentTicks(void) {
	return RobotOrder.current_ticks;
}

void tickOrder(void) {
	if(RobotOrder.state&ORDER_ACTIVE) {
		RobotOrder.current_ticks++;
	}
}
