#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H


#include "livelybot_fdcan.h"
#include "convert.h"


void motor_set_dq_vlot(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float volt);
void motor_set_dq_current(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float cur);
void motor_set_pos(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos);
void motor_set_vel(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float vel);
void motor_set_tqe(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float tqe);
void motor_set_pos_vel(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos, const float vel);
void motor_set_pos_vel_MAXtqe(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                              const float pos, const float vel, const float tqe);
void motor_set_pos_velmax_acc(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos, const float vel, const float acc);
void motor_set_vel_acc(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float vel, const float acc);
void motor_set_pos_vel_tqe_kp_kd(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                                const float pos, const float vel, const float tqe, const float kp, const float kd);
void motor_set_pos_vel_tqe_kp_kd_2(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                                const float pos, const float vel, const float tqe, const float kp, const float kd);

void motor_get_state_send(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id);
void mootr_get_version(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id);

#endif
