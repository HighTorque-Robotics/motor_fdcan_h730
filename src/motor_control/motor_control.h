#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H


#include "livelybot_fdcan.h"
#include "convert.h"


#define  MOTOR_DATA_TYPE_FLAG  TURNS


void motor_set_dq_vlot(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float volt);
void motor_set_dq_current(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float cur);
void motor_set_pos(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos);
void motor_set_vel(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float vel);
void motor_set_tqe(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float tqe, const motor_type_t motor_type);
void motor_set_pos_vel(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos, const float vel);
void motor_set_pos_vel_MAXtqe(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                              const float pos, const float vel, const float tqe, const motor_type_t motor_type);
void motor_set_pos_velmax_acc(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos, const float vel, const float acc);
void motor_set_vel_acc(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float vel, const float acc);
void motor_set_pos_vel_tqe_kp_kd(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                                 const float pos, const float vel, const float tqe, const float kp, const float kd, const motor_type_t motor_type);

void motor_set_state(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id);


#endif
