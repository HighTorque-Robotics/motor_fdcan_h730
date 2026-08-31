#ifndef _MOTOR_MANY_H
#define _MOTOR_MANY_H


#include "convert.h"
#include "my_fdcan.h"
#include "motor.h"


#define  MANY_PORT_SIZE   MOTOR_PORT_NUM  // 通道数量
#define  MANY_MOTOR_SIZE  MOTOR_MAX_NUM  // 一拖多模式下，每个CAN通道控制的电机数量，取值范围为(0，30]


#if MANY_MOTOR_SIZE > 0 && MANY_MOTOR_SIZE <= 30
#define  MANY_DATA_BUF_MAX_LEN   (MANY_MOTOR_SIZE * sizeof(many_pos_vel_tqe_kp_kd_s))
#else
#error "MANY_MOTOR_SIZE value out of range error!!!"
#endif



typedef enum __packed// 一拖多模式码
{
    MANY_MODE_POSITION    = 0x80,        // 位置控制 (每电机2字节)
    MANY_MODE_VELOCITY    = 0x81,        // 速度控制 (每电机2字节)
    MANY_MODE_TORQUE      = 0x82,        // 力矩控制 (每电机2字节)
    MANY_MODE_VOLTAGE     = 0x83,        // DQ 电压控制 (每电机2字节)
    MANY_MODE_CURRENT     = 0x84,        // DQ 电流控制 (每电机2字节)
    MANY_MODE_STOP        = 0x85,        // 停止 (每电机1字节: 非0启用, 0不启用)
    MANY_MODE_BRAKE       = 0x86,        // 刹车 (每电机1字节: 非0启用, 0不启用)
    MANY_MODE_RESET       = 0x87,        // 电机软重启 (每电机1字节: 非0启用, 0不启用)
    MANY_MODE_REZERO      = 0x88,        // 电机重置零位 (每电机1字节: 非0启用, 0不启用)
    MANY_MODE_VEL_ACC     = 0x90,        // 速度 + 加速度 (块 0x90~0x91, 2帧)
    MANY_MODE_POS_VEL_TQE = 0x92,        // 位置 + 速度 + 前馈力矩 (块 0x92~0x94, 3帧)
    MANY_MODE_POS_VEL_ACC = 0x95,        // 位置 + 速度 + 加速度(梯形) (块 0x95~0x97, 3帧)
    MANY_MODE_POS_VEL_TQE_KP_KD = 0x98,// MIT 运控 (位置+速度+力矩+Kp+Kd)
    MANY_MODE_MIT_END     = 0x9C,        // MIT 运控模式 CAN ID 基址上限
} many_mode_t;



#pragma pack(1)
typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
} many_pos_vel_tqe_s;

typedef struct
{
    int16_t vel;
    int16_t acc;
} many_vel_acc_s;

typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t acc;
} many_pos_vel_acc_s;

typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
    int16_t kp;
    int16_t kd;
} many_pos_vel_tqe_kp_kd_s;

typedef struct
{
    union
    {
        int16_t position[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t velocity[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t torque[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t voltage[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t current[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        uint8_t stop[MANY_MOTOR_SIZE];        // 停止: 每电机1字节, 非0启用, 0不启用
        uint8_t brake[MANY_MOTOR_SIZE];       // 刹车: 每电机1字节, 非0启用, 0不启用
        uint8_t reset[MANY_MOTOR_SIZE];       // 软重启: 每电机1字节, 非0启用, 0不启用
        uint8_t rezero[MANY_MOTOR_SIZE];      // 重置零位: 每电机1字节, 非0启用, 0不启用
        many_vel_acc_s vel_acc[MANY_DATA_BUF_MAX_LEN / sizeof(many_vel_acc_s)];
        many_pos_vel_tqe_s pos_vel_tqe[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_s)];
        many_pos_vel_acc_s pos_vel_acc[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_acc_s)];
        many_pos_vel_tqe_kp_kd_s pos_vel_tqe_kp_kd[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_kd_s)];
        uint8_t data[MANY_DATA_BUF_MAX_LEN];
        int16_t data16[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
    };
    uint8_t mode;
} many_data_s, *p_many_data_s;
#pragma pack()



void motor_many_dq_volt(port_t portx, const uint8_t id, const float vol);
void motor_many_dq_current(port_t portx, const uint8_t id, const float cur);
void motor_many_pos(port_t portx, const uint8_t id, const float pos);
void motor_many_vel(port_t portx, const uint8_t id, const float vel);
void motor_many_tqe(port_t portx, const uint8_t id, const float tqe);
void motor_many_stop(port_t portx, const uint8_t id, const uint8_t enable);
void motor_many_brake(port_t portx, const uint8_t id, const uint8_t enable);
void motor_many_reset(port_t portx, const uint8_t id, const uint8_t enable);
void motor_many_rezero(port_t portx, const uint8_t id, const uint8_t enable);
void motor_many_pos_vel_MAXtqe(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe);
void motor_many_vel_acc(port_t portx, const uint8_t id, const float vel, const float acc);
void motor_many_pos_vel_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc);
void motor_many_pos_vel_tqe_kp_kd_2(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd);

void motor_many_send(port_t portx, many_request_type_t request_type);

#endif
