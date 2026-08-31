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
    static uint8_t cmd[] = {MOTOR_MODE_VOLT, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_VOLT, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_VOLT, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};
    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[4], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_CUR, QUERY_MODE_FLAUT_CD_CQ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_CUR, QUERY_MODE_FLAUT_CD_CQ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[6], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_CUR, QUERY_MODE_FLAUT_CD_CQ, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[4], &q, sizeof(q));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_TQE, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_TQE, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_TQE, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_TQE, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_TQE, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_TQE, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &torque, sizeof(torque));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_VEL, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int32
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 */
void set_vel_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t vel)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机速度控制 int16
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 */
void set_vel_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t vel)
{

    static uint8_t cmd[] = {MOTOR_MODE_VEL, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief MIT模式 float (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩) (Mkp 表示电机内部 kp, kd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 1 圈，如 pos = 0.5 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 1 转/秒，如 vel = 0.5 表示 0.5 转/秒
 * @param tqe 前馈力矩：（单位见文档）
 */
void set_pos_vel_tqe_kp_kd_float_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, float pos, float vel, float tqe, float kp, float kd)
{

    static uint8_t cmd[] = {MOTOR_MODE_MIT, QUERY_MODE_FLAUT_POS_VEL_TQE,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00
                           };

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &tqe, sizeof(tqe));
    my_memcpy(&cmd[14], &kp, sizeof(kp));
    my_memcpy(&cmd[18], &kd, sizeof(kd));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief MIT模式 int32 (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩) (kp 表示电机内部 kp, kd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.00001 圈，如 pos = 50000 表示转到 0.5 圈的位置
 * @param vel 速度：单位 0.00001 转/秒，如 vel = 50000 表示 0.5 转/秒
 * @param tqe 前馈力矩（单位见文档）
 */
void set_pos_vel_tqe_kp_kd_int32_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int32_t pos, int32_t vel, int32_t tqe, int32_t kp, int32_t kd)
{

    static uint8_t cmd[] = {MOTOR_MODE_MIT, QUERY_MODE_FLAUT_POS_VEL_TQE,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00
                           };

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel, sizeof(vel));
    my_memcpy(&cmd[10], &tqe, sizeof(tqe));
    my_memcpy(&cmd[14], &kp, sizeof(kp));
    my_memcpy(&cmd[18], &kd, sizeof(kd));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief MIT模式 int16 (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩) (kp 表示电机内部 kp, kd 表示电机内部 kd)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param tqe 前馈力矩（单位见文档）
 */
void set_pos_vel_tqe_kp_kd_int16_2(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
{

    static uint8_t cmd[] = {MOTOR_MODE_MIT, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &tqe, sizeof(tqe));
    my_memcpy(&cmd[8], &kp, sizeof(kp));
    my_memcpy(&cmd[10], &kd, sizeof(kd));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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

    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_ACC, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[10], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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

    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_ACC, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[6], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[10], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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
    static uint8_t cmd[] = {MOTOR_MODE_POS_VEL_ACC, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
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

    static uint8_t cmd[] = {MOTOR_MODE_VEL_ACC, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
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

    static uint8_t cmd[] = {MOTOR_MODE_VEL_ACC, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
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

    static uint8_t cmd[] = {MOTOR_MODE_VEL_ACC, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[4], &acc, sizeof(acc));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 周期请求电机状态返回 (TINT16)
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 * @param t_us 周期时间, 单位: 1us, 4字节小端; 填 0 表示停止周期返回
 * @note 发送: 0x03 0x00 0x05 <查询码> + 4字节微秒
 *       返回数据格式由 cmd[3] 查询码决定, 当前 QUERY_MODE_FLAUT_POS_VEL_TQE(0x0B)
 *       即 模式/错误/位置/速度/力矩; 可换 0x0C(含温度)/0x0D(DQ电流)/0x0E(无模式)
 */
void request_motor_state(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id, uint32_t t_us)
{
    static uint8_t cmd[] = {0x03, 0x00, 0x05, QUERY_MODE_FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[4], &t_us, sizeof(uint32_t));

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 重设电机零位
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x03};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 保存电机设置
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x02};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机软重启
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_reset_int8(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x03, 0x03, 0x01};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 更改电机ID
 * @param fdcanHandle &hfdcanx
 * @param old_id 当前电机ID
 * @param new_id 新电机ID
 */
void set_motor_id(FDCAN_HandleTypeDef *fdcanHandle, uint8_t old_id, uint8_t new_id)
{
    /* new_id 限定在 1~126 (127 = BROADCAST_ID 广播地址, 不可作为电机 ID) */
    if (new_id < MOTOR_ID_MIN || new_id > MOTOR_ID_MAX)
    {
        MOTOR_ERR();
        new_id = (new_id < MOTOR_ID_MIN) ? MOTOR_ID_MIN : MOTOR_ID_MAX;
    }

    static uint8_t cmd[] = {0x03, 0x03, 0x04, 0x00};

    cmd[3] = new_id;

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | old_id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {MOTOR_MODE_STOP, QUERY_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_STOP, QUERY_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机停止
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_stop_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_STOP, QUERY_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_BRAKE, QUERY_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_BRAKE, QUERY_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void set_motor_brake_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{

    static uint8_t cmd[] = {MOTOR_MODE_BRAKE, QUERY_MODE_FLAUT_POS_VEL_TQE};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 float，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, QUERY_MODE_FLAUT_POS_VEL_TQE};
    fdcan_send(fdcanHandle, ID_PREFIX_TFLOAT | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int32，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, QUERY_MODE_FLAUT_POS_VEL_TQE};
    fdcan_send(fdcanHandle, ID_PREFIX_TINT32 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int16，状态、位置、速度、转矩、错误码
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, QUERY_MODE_FLAUT_POS_VEL_TQE};
    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机固件版本
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_version_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, 0x04};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机硬件版本
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void read_motor_hardware_int16(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, 0x05};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}



/**
 * @brief 查询电机型号
 * @param fdcanHandle &hfdcanx
 * @param id 电机ID
 */
void read_motor_model(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x00, 0x07};

    fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}

