#ifndef _MOTOR_CONFIG_H
#define _MOTOR_CONFIG_H


#include "motor.h"
#include "livelybot_fdcan.h"


uint8_t motor_pos_reset(port_t portx, const uint8_t id);
uint8_t motor_conf_write(port_t portx, const uint8_t id);
uint8_t motor_set_id(port_t portx, const uint8_t old_id, const uint8_t new_id);

#endif
