#ifndef _LIBELYBOT_FDCAN_H
#define _LIBELYBOT_FDCAN_H


#include "main.h"




/*
*   MOTOR_MODEL 用于选择电机型号，用于修正输入力矩。
*   MOTOR_MODEL = 5046
*   MOTOR_MODEL = 4538
*   MOTOR_MODEL = 50471  5047单极
*   MOTOR_MODEL = 50472  5047双极
*/
#define  MOTOR_MODEL  5046

#define  MOTOR_5046_TQE_CORRECT_FLOAT    1.2037
#define  MOTOR_5046_TQE_CORRECT_INT32    1.2038
#define  MOTOR_5046_TQE_CORRECT_INT16    1.2001

#define  MOTOR_4538_TQE_CORRECT_FLOAT    1.1925
#define  MOTOR_4538_TQE_CORRECT_INT32    1.2090
#define  MOTOR_4538_TQE_CORRECT_INT16    1.2029

#define  MOTOR_5047_1_TQE_CORRECT_FLOAT  1
#define  MOTOR_5047_1_TQE_CORRECT_INT32  1
#define  MOTOR_5047_1_TQE_CORRECT_INT16  1

#define  MOTOR_5047_2_TQE_CORRECT_FLOAT  1
#define  MOTOR_5047_2_TQE_CORRECT_INT32  1
#define  MOTOR_5047_2_TQE_CORRECT_INT16  1


/*
*   POS_FLAG 用于测试位置模式，共三种模式，三种模式只是数据类型不同，效果都是 -0.5转~0.5转 来回旋转。
*   POS_FLAG = 1  float
*   POS_FLAG = 2  int32
*   POS_FLAG = 3  int8
*/
#define  POS_FLAG  0


/*
*   READ_MOTOR_FLAG 用于改变读取电机状态的数据类型
*   READ_MOTOR_FLAG = 1  float
*   READ_MOTOR_FLAG = 2  int32
*   READ_MOTOR_FLAG = 3  int8
*/
#define  READ_MOTOR_FLAG   1


/*
*   POS_REZERO 用于测试电机的重置零点功能。
*   需测试此功能将 POS_REZERO 解注释即可。
*   效果是电机上电 3 秒后将当前位置设为零点
*   注意：需让电机停止后再重置零位，否则无效
*/
// #define  POS_REZERO


/*
*   MOTOR_STOP 用于测试电机停止功能。
*   需测试此功能将 MOTOR_STOP 解注释即可。
*   效果是电机上电 3 秒后将停止。
*   注意：需配合电机控制函数使用，启用 MOTOR_STOP 宏不会改变任何电机控制函数。
*/
// #define  MOTOR_STOP


/*
*   MOTOR_BRAKE 用于测试电机刹车功能功能。
*   需测试此功能将 MOTOR_BRAKE 解注释即可。
*   效果是电机上电 3 秒后将刹车。
*   注意：需配合电机控制函数使用，启用 MOTOR_BRAKE 宏不会改变任何电机控制函数。
*/
// #define  MOTOR_BRAKE


/* 各个数据类型的无限制 */
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
#define MOTOR_SIZE 18
typedef float motor_state_type;
#elif READ_MOTOR_FLAG == 2
#define MOTOR_SIZE 18
typedef int32_t motor_state_type;
#elif READ_MOTOR_FLAG == 3
#define MOTOR_SIZE 10
typedef int16_t motor_state_type;
#endif


typedef struct
{
    motor_state_type mode;
    motor_state_type position;
    motor_state_type velocity;
    motor_state_type torque;
    uint16_t id;
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

/* 位置、速度、加速度限制（梯形控制） */
void set_pos_valmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float vel_max, float acc);
void set_pos_valmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t vel_max, int32_t acc);
void set_pos_valmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t vel_max, int16_t acc);

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
