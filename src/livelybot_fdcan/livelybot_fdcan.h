#ifndef _LIBELYBOT_FDCAN_H
#define _LIBELYBOT_FDCAN_H


#include "main.h"
#include "convert.h"


/* 各个数据类型的无限制 */
#define  NAN_FLOAT  NAN
#define  NAN_INT32  0x80000000
#define  NAN_INT16  0x8000
#define  NAN_INT8   0x80



#define  MODE_POSITION              0X8080
#define  MODE_VELOCITY              0X8081
#define  MODE_TORQUE                0X8082
#define  MODE_VOLTAGE               0X8083
#define  MODE_CURRENT               0X8084
#define  MODE_OUT_TIME              0X8085

#define  MODE_POS_VEL_TQE           0X8090
#define  MODE_POS_VEL_TQE_KP_KD     0X8093
#define  MODE_POS_VEL_TQE_KP_KI_KD  0X8098
#define  MODE_POS_VEL_KP_KD         0X809E
#define  MODE_POS_VEL_TQE_RKP_RKD   0X80A3
#define  MODE_POS_VEL_RKP_RKD       0X80A8
#define  MODE_POS_VEL_ACC           0X80AD



/* dq 电压模式 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float volt);
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t volt);
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t volt);

/* dq 电流模式 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float current);
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t current);
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t current);

/* 力矩控制 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque);
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque);
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque);

/* 位置、速度和力矩控制 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque);
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque);
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque);

/* 位置 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos);
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos);
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos);

/* 速度 */
void set_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel);
void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel);
void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel);

/* 超时时间 */
void set_out_time_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t);

/* 位置、速度、力矩、PD控制 */
void set_pos_vel_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd);
void set_pos_vel_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd);
void set_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

/* 停止位置、速度、力矩、PD控制 */
void set_stoppos_vel_tqe_kp_kd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t stop_pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

/* 速度、速度限制 */
void set_vel_velmax_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t vel_max);

/* 位置、速度、加速度限制（梯形控制） */
void set_pos_velmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc);
void set_pos_velmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc);
void set_pos_velmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc);

/* 速度、加速度控制 */
void set_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc);
void set_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc);
void set_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc);

/* vfoc固定模式 */
void set_vfoc_lock(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vol);

/* 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样) */
void timed_return_motor_status_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t_ms);

/* 重设零点 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 保存设置 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 电机停止 */
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 电机刹车 */
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 读取电机状态 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);


#endif
