#ifndef _LIBELYBOT_FDCAN_H
#define _LIBELYBOT_FDCAN_H


#include "main.h"




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
*   READ_MOTOR_FLAG = 3  int16
*/
#define  READ_MOTOR_FLAG   3


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



#define  MODE_POSITION              0X8080
#define  MODE_VELOCITY              0X8081
#define  MODE_TORQUE                0X8082
#define  MODE_VOLTAGE               0X8083
#define  MODE_CURRENT               0X8084

#define  MODE_POS_VEL_TQE           0X8090
#define  MODE_POS_VEL_TQE_KP_KD     0X8093
#define  MODE_POS_VEL_TQE_KP_KI_KD  0X8098
#define  MODE_POS_VEL_KP_KD         0X809E
#define  MODE_POS_VEL_TQE_RKP_RKD   0X80A3
#define  MODE_POS_VEL_RKP_RKD       0X80A8
#define  MODE_POS_VEL_ACC           0X80AD



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

/* 位置、速度、力矩、PD控制 */
void set_pos_vel_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd);
void set_pos_vel_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd);
void set_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd);

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

/* 一拖多 位置控制 */
void set_many_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t pos2, int16_t pos3, int16_t pos4);

/* 一拖多 速度控制 */
void set_many_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t vel1, int16_t vel2, int16_t vel3, int16_t vel4);

/* 一拖多 力矩控制 */
void set_many_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t tqe1, int16_t tqe2, int16_t tqe3, int16_t tqe4);

/* 一拖多 电压控制 */
void set_many_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t volt1, int16_t volt2, int16_t volt3, int16_t volt4);

/* 一拖多 电流控制 */
void set_many_current_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

/* 一拖多 电机位置、速度、力矩控制 */
void set_many_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t tqe1, int16_t pos2, int16_t vel2, int16_t tqe2);

/* 一拖多 电机位置、速度、力矩、PD控制 */
void set_many_pos_vel_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t tqe1, int16_t rkp1, int16_t rkd1, int16_t pos2, int16_t vel2, int16_t tqe2, int16_t rkp2, int16_t rkd2);

/* 一拖多 电机位置、速度、PD控制 */
void set_many_pos_vel_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t rkp1, int16_t rkd1, int16_t pos2, int16_t vel2, int16_t rkp2, int16_t rkd2);

/* 一拖多 电机位置、速度、加速度控制 （梯形控制） */
void set_many_pos_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, int16_t pos1, int16_t vel1, int16_t acc1, int16_t pos2, int16_t vel2, int16_t acc2);

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
