#include "livelybot_fdcan.h"
#include "my_fdcan.h"
#include "motor.h"



//static void print_data(uint8_t *data, uint16_t len)
//{
//    for (int i = 0; i < len; i++)
//    {
//        printf("%d\r\n", &data[i]);
//    }
//    printf("\r\n\r\n");
//}


/**
 * @brief 电压控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param d d轴电压，例：0.0 -> 0v（通常设为 0）
 * @param q q轴电压，例：0.3 -> 0.3v
 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float d, float q)
{
//                     dq电压模式  query  d轴电压           q轴电压
    static uint8_t cmd[] = {MOTOR_MODE_VOLT, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电压控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param d d轴电压，单位：0.001V（通常设为 0）
 * @param q q轴电压，单位：0.001V
 */
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t d, int32_t q)
{
//                     dq电压模式  query  d轴电压           q轴电压
    static uint8_t cmd[] = {MOTOR_MODE_VOLT, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电压控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param d d轴电压，单位：0.1V（通常设为 0）
 * @param q q轴电压，单位：0.1V
 */
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t d, int16_t q)
{
//                     dq电压模式  query  d轴电压    q轴电压
    static uint8_t cmd[] = {MOTOR_MODE_VOLT, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00,0x00, 0x00, 0x00};
    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[4], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param d d轴电流，例：0.0 -> 0A（通常设为 0）
 * @param q q轴电流，例：0.3 -> 0.3A
 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float d, float q)
{
//                      dq电流模式  query  d轴电流           q轴电流
    static uint8_t cmd[] = {MOTOR_MODE_CUR, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param d d轴电流，单位：0.001A（通常设为 0）
 * @param q q轴电流，单位：0.001A
 */
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t d, int32_t q)
{
//                     dq电流模式  query  d轴电流           q轴电流
    static uint8_t cmd[] = {MOTOR_MODE_CUR, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param d d轴电流，单位：0.1A（通常设为 0）
 * @param q q轴电流，单位：0.1A
 */
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t d, int16_t q)
{
//                     dq电流模式 query  d轴电流    q轴电流
    static uint8_t cmd[] = {MOTOR_MODE_CUR, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[4], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param torque 力矩（单位见文档）
 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float torque)
{
//                          Mode  query  位置
    static uint8_t cmd[] = {MOTOR_MODE_TQE, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 * @param torque 力矩（单位见文档）
 */
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t torque)
{
 //                      力矩模式  query  力矩
    static uint8_t cmd[] = {MOTOR_MODE_TQE, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 * @param torque 力矩（单位见文档）
 */
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t torque)
{
 //                      力矩模式  query  力矩
    static uint8_t cmd[] = {MOTOR_MODE_TQE, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，float型
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param torque 最大力矩（单位见文档）
 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float torque)
{
 //           位置速度最大力矩模式  query  位置                   速度                     力矩  
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_TQE, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int32型
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param torque 最大力矩（单位见文档）
 */
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t torque)
{

 //           位置速度最大力矩模式  query  位置                   速度                     力矩  
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_TQE, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int16型
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param torque 最大力矩（单位见文档）
 */
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t torque)
{

 //           位置速度最大力矩模式  query  位置       速度        力矩  
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_TQE, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos)
{
 //                      位置模式  query  位置      
    static uint8_t cmd[] = {MOTOR_MODE_POS, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置。
 */
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos)
{
 //                      位置模式  query  位置   
    static uint8_t cmd[] = {MOTOR_MODE_POS, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机位置控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 */
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos)
{
 //                      位置模式  query 位置   
    static uint8_t cmd[] = {MOTOR_MODE_POS, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 1 转/秒，如 vel = 0.1 -> 0.1 转/秒
 */
void set_vel_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel)
{
 //                      速度模式  query 速度 
    static uint8_t cmd[] = {MOTOR_MODE_VEL, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 */
void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 */
void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 真运控模式 float (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param tqe 前馈力矩：（单位见文档）
 * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 1 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_float_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd)
{

    static uint8_t cmd[] = {MOTOR_MODE_MIT, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &tqe, sizeof(tqe));
    my_memcpy(&cmd[14], &kp, sizeof(kp));
    my_memcpy(&cmd[18], &kd, sizeof(kd));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 真运控模式 int32 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.001 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 0.001 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_int32_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd)
{

    static uint8_t cmd[] = {MOTOR_MODE_MIT, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &tqe, sizeof(tqe));
    my_memcpy(&cmd[14], &kp, sizeof(kp));
    my_memcpy(&cmd[18], &kd, sizeof(kd));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 真运控模式 int16 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kp * 0.1 (Mkd 表示电机内部 kd)
 */
void set_pos_vel_tqe_kp_kd_int16_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
{

    static uint8_t cmd[] = {MOTOR_MODE_MIT, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &tqe, sizeof(tqe));
    my_memcpy(&cmd[8], &kp, sizeof(kp));
    my_memcpy(&cmd[10], &kd, sizeof(kd));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、速度限幅控制（如果 vel > vel_max，则用 vel_max） int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param vel_max 速度限幅：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 */
void set_vel_velmax_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t vel_max)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[4], &vel_max, sizeof(vel_max));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制） float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel_max 速度限制，单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param acc 加速度，单位：1 转/秒^2
 */
void set_pos_velmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel_max, float acc)
{

    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_ACC, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[10], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制） int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel_max 速度限制：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param acc 加速度：单位 0.00001 转/秒^2，如 acc = 50000 表示 0.5 转/秒^2
 */
void set_pos_velmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel_max, int32_t acc)
{

    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_ACC, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[10], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制） int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 * @param vel_max 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param acc 加速度：单位 0.001 转/秒^2，如 acc = 100 表示 0.1 转/秒^2
 */
void set_pos_velmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc)
{
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_ACC, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 float
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度，单位： 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param acc 加速度，单位：1 转/秒^2
 */
void set_vel_acc_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float vel, float acc)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL_ACC, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param acc 加速度：单位 0.001 转/秒^2，如 vel = 500 表示 0.5 转/秒^2
 */
void set_vel_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel, int32_t acc)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL_ACC, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度、加速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param acc 加速度：单位 0.01 转/秒^2，如 vel = 40 表示 0.4 转/秒^2
 */
void set_vel_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel, int16_t acc)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL_ACC, MANY_GET_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[4], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 设置电机超时时间，电机超过超时时间没接受到新指令，电机进入刹车模式
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param t 电机超时时间，单位：1ms
 */
void set_out_time_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t)
{
    static uint8_t cmd[] = {0x05, 0x1F, 0x00, 0x00};

    my_memcpy(&cmd[2], &t, sizeof(int16_t));

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


// /**
//  * @brief 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样)
//  *          1. 周期返回电机位置、速度、力矩数据。
//  *          2. 返回数据格式和使用  0x17，0x01  指令获取的格式一样
//  *          3. 周期单位为 ms。
//  *          4. 最小周期为 1ms，最大周期 32767ms。
//  *          5. 如需停止周期返回数据，将周期给 0 即可，或者给电机断电。
//  * @param id 电机ID
//  * @param t 返回周期（单位：ms）
//  * @param 此函数暂未开放使用
//  */
// void timed_return_motor_status_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t t_ms)
// {
//     static uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

//     *(int16_t *)&tdata[3] = t_ms;

//     fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, tdata, sizeof(tdata));
// }


/**
 * @brief 重设电机零位
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x03};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 保存电机设置
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x02};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机软重启
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_reset_int8(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x01};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 更改电机ID
 * @param fdcanHandle &hfdcanx
 * @param old_id 当前电机ID
 * @param new_id 新电机ID
 */
void set_motor_id(FDCAN_HandleTypeDef *fdcanHandle, uint8_t old_id, uint8_t new_id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x04, 0x00};

    cmd[3] = new_id;

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | old_id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {MOTOR_MODE_STOP, MANY_GET_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_STOP, MANY_GET_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_STOP, MANY_GET_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_BRAKE, MANY_GET_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_BRAKE, MANY_GET_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_BRAKE, MANY_GET_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 float，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, MANY_GET_MODE_FLAUT_POS_VEL_TQE};
    fdcan_send(fdcanHandle, ID_TITLE_FLOAT | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int32，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, MANY_GET_MODE_FLAUT_POS_VEL_TQE};
    fdcan_send(fdcanHandle, ID_TITLE_INT32 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int16，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, MANY_GET_MODE_FLAUT_POS_VEL_TQE};
    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机固件版本
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_version_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, 0x04};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机硬件版本
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_hardware_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, 0x05};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, (uint8_t *)cmd, sizeof(cmd));
}



/**
 * @brief 查询电机型号
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 */
void read_motor_model(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, 0x07};

    fdcan_send(fdcanHandle, ID_TITLE_INT16 | id, (uint8_t *)cmd, sizeof(cmd));
}

