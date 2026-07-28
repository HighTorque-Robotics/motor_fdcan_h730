#ifndef _TEST_MOTOR_H
#define _TEST_MOTOR_H



#include "motor_control.h"
#include "fdcan.h"


void test_motor_control(const uint8_t id);
void test_motor_cycle(port_t portx, data_type_t type, uint8_t id);

#endif

