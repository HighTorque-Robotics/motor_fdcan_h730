#ifndef _MOTOR_CONFIG_H
#define _MOTOR_CONFIG_H


#include "motor.h"
#include "livelybot_fdcan.h"


uint8_t motor_pos_reset(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id);


#endif
