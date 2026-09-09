#include "motor_many.h"
#include <stdio.h>

many_data_s many_data_port[MANY_PORT_SIZE][MANY_DATA_BUF_MAX_LEN];


p_many_data_s motor_get_many_pointer(port_t portx)
{
    if (portx < 1 || portx > MANY_PORT_SIZE)
    {
        MOTOR_ERR();
        return NULL;
    }

    return many_data_port[portx - 1];
}


/**
 * @brief 一拖多 DQ 电压模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V
 */
void motor_many_dq_volt(port_t portx, const uint8_t id, const float vol)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const int16_t vol_raw = vol_float2int(vol, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_VOLTAGE)
    {
        p_many_data->mode = MANY_MODE_VOLTAGE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->voltage[index] = vol_raw;
}


/**
 * @brief 一拖多 DQ 电流模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A
 */
void motor_many_dq_current(port_t portx, const uint8_t id, const float cur)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const int16_t cur_raw = cur_float2int(cur, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_CURRENT)
    {
        p_many_data->mode = MANY_MODE_CURRENT;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->current[index] = cur_raw;
}


/**
 * @brief 一拖多 位置模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_pos(port_t portx, const uint8_t id, const float pos)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_raw = pos_float2int(pos_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_POSITION)
    {
        p_many_data->mode = MANY_MODE_POSITION;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->position[index] = pos_raw;
}


/**
 * @brief 一拖多 速度模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_vel(port_t portx, const uint8_t id, const float vel)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_raw = vel_float2int(vel_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_VELOCITY)
    {
        p_many_data->mode = MANY_MODE_VELOCITY;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->velocity[index] = vel_raw;
}


/**
 * @brief 一拖多 力矩模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（Nm）
 */
void motor_many_tqe(port_t portx, const uint8_t id, const float tqe)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const int16_t tqe_raw = tqe_float2int(tqe, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_TORQUE)
    {
        p_many_data->mode = MANY_MODE_TORQUE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->torque[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->torque[index] = tqe_raw;
}


/**
 * @brief 一拖多 停止模式 (CAN ID 0x8085)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param enable 1-启用, 0-不启用
 */
void motor_many_stop(port_t portx, const uint8_t id, const uint8_t enable)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_STOP)
    {
        p_many_data->mode = MANY_MODE_STOP;
        for (int i = 0; i < MANY_MOTOR_SIZE; i++)
        {
            p_many_data->stop[i] = 0;
        }
    }

    p_many_data->stop[index] = enable;
}


/**
 * @brief 一拖多 刹车模式 (CAN ID 0x8086)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param enable 1-启用, 0-不启用
 */
void motor_many_brake(port_t portx, const uint8_t id, const uint8_t enable)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_BRAKE)
    {
        p_many_data->mode = MANY_MODE_BRAKE;
        for (int i = 0; i < MANY_MOTOR_SIZE; i++)
        {
            p_many_data->brake[i] = 0;
        }
    }

    p_many_data->brake[index] = enable;
}


/**
 * @brief 一拖多 电机软重启模式 (CAN ID 0x8087)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param enable 1-启用, 0-不启用
 */
void motor_many_reset(port_t portx, const uint8_t id, const uint8_t enable)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_RESET)
    {
        p_many_data->mode = MANY_MODE_RESET;
        for (int i = 0; i < MANY_MOTOR_SIZE; i++)
        {
            p_many_data->reset[i] = 0;
        }
    }

    p_many_data->reset[index] = enable;
}


/**
 * @brief 一拖多 电机重置零位模式 (CAN ID 0x8088)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param enable 1-启用, 0-不启用
 */
void motor_many_rezero(port_t portx, const uint8_t id, const uint8_t enable)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_REZERO)
    {
        p_many_data->mode = MANY_MODE_REZERO;
        for (int i = 0; i < MANY_MOTOR_SIZE; i++)
        {
            p_many_data->rezero[i] = 0;
        }
    }

    p_many_data->rezero[index] = enable;
}





/**
 * @brief 一拖多 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（Nm）
 */
void motor_many_pos_vel_MAXtqe(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);


    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);

    const int16_t pos_raw = pos_float2int(pos_turns, TINT16);
    const int16_t vel_raw = vel_float2int(vel_turns, TINT16);
    const int16_t tqe_raw = tqe_float2int(tqe, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_POS_VEL_TQE)
    {
        p_many_data->mode = MANY_MODE_POS_VEL_TQE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_raw;
    p_many_data->pos_vel_tqe[index].vel = vel_raw;
    p_many_data->pos_vel_tqe[index].tqe = tqe_raw;
}


/**
 * @brief 一拖多 速度加速度模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_vel_acc(port_t portx, const uint8_t id, const float vel, const float acc)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);

    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);

    const int16_t vel_raw = vel_float2int(vel_turns, TINT16);
    const int16_t acc_raw = acc_float2int(acc_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_VEL_ACC)
    {
        p_many_data->mode = MANY_MODE_VEL_ACC;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->vel_acc[index].vel = vel_raw;
    p_many_data->vel_acc[index].acc = acc_raw;
}


/**
 * @brief 位置、速度、加速度模式（梯形控制）
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_pos_vel_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);

    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);

    const int16_t pos_raw = pos_float2int(pos_turns, TINT16);
    const int16_t vel_raw = vel_float2int(vel_turns, TINT16);
    const int16_t acc_raw = acc_float2int(acc_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_POS_VEL_ACC)
    {
        p_many_data->mode = MANY_MODE_POS_VEL_ACC;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->pos_vel_acc[index].pos = pos_raw;
    p_many_data->pos_vel_acc[index].vel = vel_raw;
    p_many_data->pos_vel_acc[index].acc = acc_raw;
}

/**
 * @brief MIT模式 (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（Nm）
 * @param kp 单位可为牛米每转（Nm/rev）、牛米每弧度（Nm/rad）、或牛米每度（Nm/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param kd 单位可为牛米秒每转（Nm·s/rev）、牛米秒每弧度（Nm·s/rad）、或牛米秒每度（Nm·s/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_pos_vel_tqe_kp_kd(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);

    /* 单位转换成转 */
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float kp_turns = conv_from_turns(kp, MOTOR_DATA_TYPE_FLAG);
    const float kd_turns = conv_from_turns(kd, MOTOR_DATA_TYPE_FLAG);

    /* float -> int */
    const float pos_raw = pos_float2int(pos_turns, TINT16);
    const float vel_raw = vel_float2int(vel_turns, TINT16);
    const float tqe_raw = tqe_float2int(tqe, TINT16);
    const float kp_raw = pid_float2int(kp_turns, TINT16);
    const float kd_raw = pid_float2int(kd_turns, TINT16);

    const uint16_t index = id - 1;

    if (p_many_data->mode != MANY_MODE_POS_VEL_TQE_KP_KD)
    {
        p_many_data->mode = MANY_MODE_POS_VEL_TQE_KP_KD;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->data16[i] = 0x8000;  // NAN_INT16
        }
    }

    p_many_data->pos_vel_tqe_kp_kd[index].pos = pos_raw;
    p_many_data->pos_vel_tqe_kp_kd[index].vel = vel_raw;
    p_many_data->pos_vel_tqe_kp_kd[index].tqe = tqe_raw;
    p_many_data->pos_vel_tqe_kp_kd[index].kp = kp_raw;
    p_many_data->pos_vel_tqe_kp_kd[index].kd = kd_raw;
}


static uint8_t get_mode_data_len(uint8_t mode)
{
    switch(mode)
    {
    case MANY_MODE_POSITION:
    case MANY_MODE_VELOCITY:
    case MANY_MODE_TORQUE:
    case MANY_MODE_VOLTAGE:
    case MANY_MODE_CURRENT:
        return 2;
    case MANY_MODE_STOP:
    case MANY_MODE_BRAKE:
    case MANY_MODE_RESET:
    case MANY_MODE_REZERO:
        return 1;
    case MANY_MODE_VEL_ACC:
        return 4;
    case MANY_MODE_POS_VEL_TQE:
    case MANY_MODE_POS_VEL_ACC:
        return 6;
    case MANY_MODE_POS_VEL_TQE_KP_KD:
        return 10;
    }

    return 0;
}


static uint8_t get_fdcan_len(uint16_t len)
{
    uint32_t dlc = 0;

    if (len <= 8)
    {
        dlc = len;
    }
    else if (len <= 12)
    {
        dlc = 12;
    }
    else if (len <= 16)
    {
        dlc = 16;
    }
    else if (len <= 20)
    {
        dlc = 20;
    }
    else if (len <= 24)
    {
        dlc = 24;
    }
    else if (len <= 32)
    {
        dlc = 32;
    }
    else if (len <= 48)
    {
        dlc = 48;
    }
    else
    {
        dlc = 64;
    }

    return dlc;
}




/**
 * @brief 一拖多 发送 (切分成多帧, 每帧最多 60 字节, ID 递增, query 填帧尾)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param request_type 决定电机返回帧包含的信息
 */
void motor_many_send(port_t portx, motor_query_t request_type)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    /* 一拖多固定 TINT16,  ID_PREFIX_TINT16, fdcan_send 自动置 bit[15]=1 */

    uint8_t id = p_many_data->mode;          /* 一拖多模式下 bits[6:0] = 模式块编号 */
    uint8_t *p_data = p_many_data->data;     /* 当前模式打包好的连续数据 */
    uint16_t data_len = get_mode_data_len(id) * MANY_MOTOR_SIZE;

    static uint8_t buf[64] = {0};

    while (data_len > 0)
    {
        const uint8_t cut_len = (data_len > 60) ? 60 : data_len;      // 每帧最多切 60 字节数据
        data_len -= cut_len;

        /* 帧长 = 数据 + 1 查询码, 并做 CAN FD 块对齐 (8/12/16/20/24/32/48/64) */
        const uint16_t byte_len = cut_len + 1;
        const uint8_t frame_len = get_fdcan_len(byte_len);

        my_memcpy(buf, p_data, cut_len);
        buf[frame_len - 1] = (uint8_t)request_type;                  // 查询码放帧尾 (对齐后最后一字节)
        p_data += cut_len;

        /* 传字节数 byte_len, fdcan_send 内部自动转 DLC 并对齐填充 */
        fdcan_send(fdcanHandle, ID_PREFIX_TINT16 | id, buf, frame_len);
        id++;                                                         // 每帧模式块编号递增
    }
}
