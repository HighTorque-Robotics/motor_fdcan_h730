#ifndef _LIBELYBOT_FDCAN_H
#define _LIBELYBOT_FDCAN_H


#include "main.h"


#define  POS_FLAG  0
// #define  POS_REZERO
// #define  MOTOR_STOP
#define  MOTOR_BRAKE
#define  READ_MOTOR_FLAG   3



#define  NAN_FLOAT  NAN
#define  NAN_INT32  0x80000000
#define  NAN_INT16  0x8000
#define  NAN_INT8   0x80

// 此处根据电机的个数和ID号，自定义电机的名字
typedef enum
{
    MOTOR1 = 1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
    MOTOR6,
    MOTOR7,
    MOTOR8,
    MOTOR9,
    MOTOR10,
    MOTOR11,
    MOTOR12,
    MOTOR13,
    MOTOR14,
    MOTOR15,
    MOTOR16
} motor_e;


#if READ_MOTOR_FLAG == 1
#define MOTOR_SIZE 24
typedef float motor_state_type;
#elif READ_MOTOR_FLAG == 2
#define MOTOR_SIZE 24
typedef int32_t motor_state_type;
#elif READ_MOTOR_FLAG == 3
#define MOTOR_SIZE 24
typedef int16_t motor_state_type;
#endif


typedef struct
{
    uint32_t id;
    motor_state_type position;
    motor_state_type velocity;
    motor_state_type torque;
} motor_state_s;

typedef struct
{
    union
    {
        motor_state_s motor;
        uint8_t data[MOTOR_SIZE];
    };
} motor_state_t;


extern motor_state_t motor_state;
extern uint8_t motor_read_flag;


/* dq 电压模式 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float volt);
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t volt);
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t volt);

/* dq 电流模式 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float current);
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t current);
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t current);

/* 力矩控制 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float torque);
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t torque);
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t torque);

/* 位置、速度和力矩控制 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float torque);
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t torque);
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t torque);

/* 位置 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos);
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos);
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos);

/* 速度 */
void set_val_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val);
void set_val_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val);
void set_val_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val);

/* 位置、速度、力矩、PD控制 */
void set_pos_val_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float tqe, float kp, float kd);
void set_pos_val_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t tqe, float rkp, float rkd);
void set_pos_val_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t tqe, float rkp, float rkd);

/* 速度、速度限制 */
void set_val_valmax_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t vel_max);

/* 位置、速度限制 */
void set_pos_valmax_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float vel_max);
void set_pos_valmax_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t vel_max);
void set_pos_valmax_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t vel_max);

/* 速度、加速度控制 */
void set_val_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val, float acc);
void set_val_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val, int32_t acc);
void set_val_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t acc);

/* 重设零点 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* 保存设置 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* 电机停止 */
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* 电机刹车 */
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* 读取电机状态 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);


#endif
