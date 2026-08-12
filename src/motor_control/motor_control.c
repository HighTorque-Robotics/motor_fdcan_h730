#include "motor_control.h"
#include "motor.h"



/**
 * @brief DQ 电压模式（并让电机返回状态信息）— d轴=0, q轴=实际电压
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V（D 轴固定为 0）
 */
void motor_set_dq_vlot(port_t portx, const data_type_t type, const uint8_t id, const float volt)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float volt_raw = vol_float2int(volt, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_volt_float(fdcanHandle, id, 0.0f, volt_raw);
        break;
    case TINT32:
        set_dq_volt_int32(fdcanHandle, id, 0, volt_raw);
        break;
    case TINT16:
        set_dq_volt_int16(fdcanHandle, id, 0, volt_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief DQ 电流模式（并让电机返回状态信息）— d轴=0, q轴=实际电流
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A（D 轴固定为 0）
 */
void motor_set_dq_current(port_t portx, const data_type_t type, const uint8_t id, const float cur)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float cur_raw = cur_float2int(cur, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_current_float(fdcanHandle, id, 0.0f, cur_raw);
        break;
    case TINT32:
        set_dq_current_int32(fdcanHandle, id, 0, cur_raw);
        break;
    case TINT16:
        set_dq_current_int16(fdcanHandle, id, 0, cur_raw);
        break;
    default:
        break;
    }
}
/*
 * @brief DQ 电流模式，分别下发 D 轴和 Q 轴目标电流。
 * @note 函数同时兼容协议支持的三种数据类型；输入单位始终为A，
 *       发送前由 cur_float2int() 按协议量程完成换算。
 *       该接口函数用于测试，暂不开放给客户使用，新增可变化i_d电流参数
 * @param id 电机 ID
 * @param d_cur D 相电流，单位：（A），例：0.1 -> 0.1A
 * @param d_cur D 相电流，单位：（A），例：0.3 -> 0.3A
 */
void motor_set_dq_current_full(port_t portx, const data_type_t type,const uint8_t id, const float d_cur,const float q_cur)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float d_raw = cur_float2int(d_cur, type);
    const float q_raw = cur_float2int(q_cur, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_current_float(fdcanHandle, id, d_raw, q_raw);
        break;
    case TINT32:
        set_dq_current_int32(fdcanHandle, id, (int32_t)d_raw, (int32_t)q_raw);
        break;
    case TINT16:
        set_dq_current_int16(fdcanHandle, id, (int16_t)d_raw, (int16_t)q_raw);
        break;
    default:
        break;
    }
}

/**
 * @brief 位置模式，使用最大速度和加速度运动到目标位置（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos(port_t portx, const data_type_t type, const uint8_t id, const float pos)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_float(fdcanHandle, id, pos_raw);
        break;
    case TINT32:
        set_pos_int32(fdcanHandle, id, pos_raw);
        break;
    case TINT16:
        set_pos_int16(fdcanHandle, id, pos_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 速度模式，以最大加速度加速到指定速度（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_vel(port_t portx, const data_type_t type, const uint8_t id, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float vel_raw = vel_float2int(vel_turns, type);

    switch(type)
    {
    case TFLOAT:
        set_vel_float(fdcanHandle, id, vel_raw);
        break;
    case TINT32:
        set_vel_int32(fdcanHandle, id, vel_raw);
        break;
    case TINT16:
        set_vel_int16(fdcanHandle, id, vel_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 力矩模式（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（Nm）
 */
void motor_set_tqe(port_t portx, const data_type_t type, const uint8_t id, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float tqe_raw = tqe_float2int(tqe, type);

    switch(type)
    {
    case TFLOAT:
        set_torque_float(fdcanHandle, id, tqe_raw);
        break;
    case TINT32:
        set_torque_int32(fdcanHandle, id, tqe_raw);
        break;
    case TINT16:
        set_torque_int16(fdcanHandle, id, tqe_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，不限制加速度和最大输出力矩（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_vel(port_t portx, const data_type_t type, const uint8_t id, const float pos, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, type);
    const float vel_raw = vel_float2int(vel_turns, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_float(fdcanHandle, id, pos_raw, vel_raw, NAN);        // NAN_FLOAT
        break;
    case TINT32:
        set_pos_vel_tqe_int32(fdcanHandle, id, pos_raw, vel_raw, 0x80000000); // NAN_INT32
        break;
    case TINT16:
        set_pos_vel_tqe_int16(fdcanHandle, id, pos_raw, vel_raw, 0x8000);     // NAN_INT16
        break;
    default:
        break;
    }
}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（Nm）
 */
void motor_set_pos_vel_MAXtqe(port_t portx, const data_type_t type, const uint8_t id,
                              const float pos, const float vel, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, type);
    const float vel_raw = vel_float2int(vel_turns, type);
    const float tqe_raw = tqe_float2int(tqe, type);


    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_float(fdcanHandle, id, pos_raw, vel_raw, tqe_raw);
        break;
    case TINT32:
        set_pos_vel_tqe_int32(fdcanHandle, id, pos_raw, vel_raw, tqe_raw);
        break;
    case TINT16:
        set_pos_vel_tqe_int16(fdcanHandle, id, pos_raw, vel_raw, tqe_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 位置、速度、加速度模式（梯形控制）（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_velmax_acc(port_t portx, const data_type_t type, const uint8_t id, const float pos, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, type);
    const float vel_raw = vel_float2int(vel_turns, type);
    const float acc_raw = acc_float2int(acc_turns, type);


    switch(type)
    {
    case TFLOAT:
        set_pos_velmax_acc_float(fdcanHandle, id, pos_raw, vel_raw, acc_raw);
        break;
    case TINT32:
        set_pos_velmax_acc_int32(fdcanHandle, id, pos_raw, vel_raw, acc_raw);
        break;
    case TINT16:
        set_pos_velmax_acc_int16(fdcanHandle, id, pos_raw, vel_raw, acc_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 速度、加速度模式，以目标加速度加速到目标速度（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_vel_acc(port_t portx, const data_type_t type, const uint8_t id, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float vel_raw = vel_float2int(vel_turns, type);
    const float acc_raw = acc_float2int(acc_turns, type);

    switch(type)
    {
    case TFLOAT:
        set_vel_acc_float(fdcanHandle, id, vel_raw, acc_raw);
        break;
    case TINT32:
        set_vel_acc_int32(fdcanHandle, id, vel_raw, acc_raw);
        break;
    case TINT16:
        set_vel_acc_int16(fdcanHandle, id, vel_raw, acc_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 运控模式2(MIT模式) (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩)（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（Nm）
 * @param kp 单位可为牛米每转（Nm/rev）、牛米每弧度（Nm/rad）、或牛米每度（Nm/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param kd 单位可为牛米秒每转（Nm·s/rev）、牛米秒每弧度（Nm·s/rad）、或牛米秒每度（Nm·s/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_vel_tqe_kp_kd_2(port_t portx, const data_type_t type, const uint8_t id,
                                   const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    /* 单位转换成转 */
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float kp_turns = conv_from_turns(kp, MOTOR_DATA_TYPE_FLAG);
    const float kd_turns = conv_from_turns(kd, MOTOR_DATA_TYPE_FLAG);

    /* float -> int */
    const float pos_raw = pos_float2int(pos_turns, type);
    const float vel_raw = vel_float2int(vel_turns, type);
    const float tqe_raw = tqe_float2int(tqe, type);
    const float kp_raw = pid_float2int(kp_turns, type);
    const float kd_raw = pid_float2int(kd_turns, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_kp_kd_float_2(fdcanHandle, id, pos_raw, vel_raw, tqe_raw, kp_raw, kd_raw);
        break;
    case TINT32:
        set_pos_vel_tqe_kp_kd_int32_2(fdcanHandle, id, pos_raw, vel_raw, tqe_raw, kp_raw, kd_raw);
        break;
    case TINT16:
        set_pos_vel_tqe_kp_kd_int16_2(fdcanHandle, id, pos_raw, vel_raw, tqe_raw, kp_raw, kd_raw);
        break;
    default:
        break;
    }
}


/**
 * @brief 发送查询查询电机状态信息的指令（在motor_process_state中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 */
void motor_get_state_send(port_t portx, const data_type_t type, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    switch(type)
    {
    case TFLOAT:
        read_motor_state_float(fdcanHandle, id);
        break;
    case TINT32:
        read_motor_state_int32(fdcanHandle, id);
        break;
    case TINT16:
        read_motor_state_int16(fdcanHandle, id);
        break;
    default:
        break;
    }
}


/**
 * @brief 发送查询电机固件版本号指令（在motor_process_state中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_get_version(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    // for (uint8_t i = 0; i < 5; i++)
    {
        read_motor_version_int16(fdcanHandle, id);
    }
}


/**
 * @brief 发送查询电机型号指令（在 motor_process_state 中解析并打印）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_get_model(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    read_motor_model(fdcanHandle, id);
}


/**
 * @brief 发送查询电机硬件版本号指令（cmd: 0x00 0x05，在 motor_process_state 中解析并打印）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_get_hardware_version(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    read_motor_hardware_int16(fdcanHandle, id);
}


/**
 * @brief 停止模式，电机三相都断开（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_set_stop(port_t portx, const data_type_t type, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    switch(type)
    {
    case TFLOAT:
        set_motor_stop_float(fdcanHandle, id);
        break;
    case TINT32:
        set_motor_stop_int32(fdcanHandle, id);
        break;
    case TINT16:
        set_motor_stop_int16(fdcanHandle, id);
        break;
    default:
        break;
    }
}


/**
 * @brief 刹车模式（阻尼模式），电机三相都接地（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_set_brake(port_t portx, const data_type_t type, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    switch(type)
    {
    case TFLOAT:
        set_motor_brake_float(fdcanHandle, id);
        break;
    case TINT32:
        set_motor_brake_int32(fdcanHandle, id);
        break;
    case TINT16:
        set_motor_brake_int16(fdcanHandle, id);
        break;
    default:
        break;
    }
}


/**
 * @brief 电机软重启，重启后进入停止模式
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_set_reset(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_reset_int8(fdcanHandle, id);
}





