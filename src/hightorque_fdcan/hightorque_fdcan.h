#ifndef _HIGHTORQUE_FDCAN_H
#define _HIGHTORQUE_FDCAN_H


#include "main.h"
#include "convert.h"


/* CAN ID 帧头 (接收端 bit[15]=0; 发送控制帧由 fdcan_send 自动置 bit[15]=1) */
/* bits[18]=CAN MIT, bits[17:16]=数据类型(与 data_type_t 枚举值一致), bit[15]=控制/返回区分 */

/* ---- 数据类型 (由 convert.h 的 data_type_t 枚举左移16位派生, 只改枚举即可同步) ---- */
#define  ID_PREFIX_TINT16_NOHDR     ((uint32_t)TINT16_NOHDR << 16)  // bits[17:16]=00
#define  ID_PREFIX_TINT16           ((uint32_t)TINT16       << 16)  // bits[17:16]=01
#define  ID_PREFIX_TINT32           ((uint32_t)TINT32       << 16)  // bits[17:16]=10
#define  ID_PREFIX_TFLOAT           ((uint32_t)TFLOAT       << 16)  // bits[17:16]=11

/* dq 电压模式 (d=0, q=实际电压) */
void hightorque_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float d, float q);
void hightorque_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t d, int32_t q);
void hightorque_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t d, int16_t q);

/* dq 电流模式 (d=0, q=实际电流) */
void hightorque_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float d, float q);
void hightorque_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t d, int32_t q);
void hightorque_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t d, int16_t q);

/* 力矩控制 */
void hightorque_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque);
void hightorque_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque);
void hightorque_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque);

/* 位置、速度和力矩控制 */
void hightorque_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque);
void hightorque_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque);
void hightorque_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque);

/* 位置 */
void hightorque_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos);
void hightorque_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos);
void hightorque_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos);

/* 速度 */
void hightorque_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel);
void hightorque_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel);
void hightorque_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel);

/* 位置、速度、力矩、PD控制（真运控模式） */
void hightorque_pos_vel_tqe_kp_kd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd);
void hightorque_pos_vel_tqe_kp_kd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd);
void hightorque_pos_vel_tqe_kp_kd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);


/* 位置、速度、加速度限制（梯形控制） */
void hightorque_pos_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc);
void hightorque_pos_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc);
void hightorque_pos_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc);

/* 速度、加速度控制 */
void hightorque_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc);
void hightorque_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc);
void hightorque_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc);

/* 周期请求电机状态返回 (TINT16 发送: 0x03 0x00 0x05 <查询码> + 4字节微秒, t_us=0 停止周期返回) */
void hightorque_request_timed_return(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, uint32_t t_us);

/* 重设零点 */
void hightorque_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 保存设置 */
void hightorque_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 重启电机 */
void hightorque_reset(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 更改电机ID */
void hightorque_id(FDCAN_HandleTypeDef *fdcanHandle, uint8_t old_id, uint8_t new_id);

/* 电机停止 */
void hightorque_stop_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void hightorque_stop_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void hightorque_stop_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 电机刹车 */
void hightorque_brake_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void hightorque_brake_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void hightorque_brake_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 读取电机状态 */
void hightorque_request_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void hightorque_request_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void hightorque_request_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 查询电机固件版本 */
void hightorque_request_fw_version(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 查询电机型号 */
void hightorque_request_model(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

/* 查询电机硬件版本号 */
void hightorque_request_hw_version(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);


#endif
