#include "motor_control.h"
#include "motor.h"



void motor_set_dq_vlot(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float volt)
{
    const float temp = vol_float2int(volt, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_volt_float(fdcanHandle, id, temp);
        break;
    case TINT32:
        set_dq_volt_int32(fdcanHandle, id, temp);
        break;
    case TINT16:
        set_dq_volt_int16(fdcanHandle, id, temp);
        break;
    default:
        break;
    }
}


void motor_set_dq_current(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float cur)
{
    const float temp = cur_float2int(cur, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_current_float(fdcanHandle, id, temp);
        break;
    case TINT32:
        set_dq_current_int32(fdcanHandle, id, temp);
        break;
    case TINT16:
        set_dq_current_int16(fdcanHandle, id, temp);
        break;
    default:
        break;
    }
}


void motor_set_pos(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos)
{
    const float temp1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float temp2 = pos_float2int(temp1, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_float(fdcanHandle, id, temp2);
        break;
    case TINT32:
        set_pos_int32(fdcanHandle, id, temp2);
        break;
    case TINT16:
        set_pos_int16(fdcanHandle, id, temp2);
        break;
    default:
        break;
    }
}


void motor_set_vel(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float vel)
{
    const float temp1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float temp2 = vel_float2int(temp1, type);

    switch(type)
    {
    case TFLOAT:
        set_vel_float(fdcanHandle, id, temp2);
        break;
    case TINT32:
        set_vel_int32(fdcanHandle, id, temp2);
        break;
    case TINT16:
        set_vel_int16(fdcanHandle, id, temp2);
        break;
    default:
        break;
    }
}


void motor_set_tqe(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float tqe)
{
    const float temp1 = tqe_adjust(tqe, motor_get_model1(fdcanHandle, id));
    const float temp2 = tqe_float2int(temp1, type);

    switch(type)
    {
    case TFLOAT:
        set_torque_float(fdcanHandle, id, temp2);
        break;
    case TINT32:
        set_torque_int32(fdcanHandle, id, temp2);
        break;
    case TINT16:
        set_torque_int16(fdcanHandle, id, temp2);
        break;
    default:
        break;
    }
}


void motor_set_pos_vel(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos, const float vel)
{
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_float(fdcanHandle, id, pos2, vel2, NAN_FLOAT);
        break;
    case TINT32:
        set_pos_vel_tqe_int32(fdcanHandle, id, pos2, vel2, NAN_INT32);
        break;
    case TINT16:
        set_pos_vel_tqe_int16(fdcanHandle, id, pos2, vel2, NAN_INT16);
        break;
    default:
        break;
    }
}


void motor_set_pos_vel_MAXtqe(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                              const float pos, const float vel, const float tqe)
{
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe1 = tqe_adjust(tqe, motor_get_model1(fdcanHandle, id));
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);
    const float tqe2 = tqe_float2int(tqe1, type);


    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_float(fdcanHandle, id, pos2, vel2, tqe2);
        break;
    case TINT32:
        set_pos_vel_tqe_int32(fdcanHandle, id, pos2, vel2, tqe2);
        break;
    case TINT16:
        set_pos_vel_tqe_int16(fdcanHandle, id, pos2, vel2, tqe2);
        break;
    default:
        break;
    }
}


void motor_set_pos_velmax_acc(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float pos, const float vel, const float acc)
{
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc1 = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);
    const float acc2 = acc_float2int(acc1, type);


    switch(type)
    {
    case TFLOAT:
        set_pos_velmax_acc_float(fdcanHandle, id, pos2, vel2, acc2);
        break;
    case TINT32:
        set_pos_velmax_acc_int32(fdcanHandle, id, pos2, vel2, acc2);
        break;
    case TINT16:
        set_pos_velmax_acc_int16(fdcanHandle, id, pos2, vel2, acc2);
        break;
    default:
        break;
    }
}


void motor_set_vel_acc(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id, const float vel, const float acc)
{
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc1 = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float vel2 = vel_float2int(vel1, type);
    const float acc2 = acc_float2int(acc1, type);

    switch(type)
    {
    case TFLOAT:
        set_vel_acc_float(fdcanHandle, id, vel2, acc2);
        break;
    case TINT32:
        set_vel_acc_int32(fdcanHandle, id, vel2, acc2);
        break;
    case TINT16:
        set_vel_acc_int16(fdcanHandle, id, vel2, acc2);
        break;
    default:
        break;
    }
}


void motor_set_pos_vel_tqe_kp_kd(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id,
                                 const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe1 = tqe_adjust(tqe, motor_get_model1(fdcanHandle, id));
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);
    const float tqe2 = tqe_float2int(tqe1, type);

    const float kp2 = pid_float2int(kp, type);
    const float kd2 = pid_float2int(kd, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_pd_float(fdcanHandle, id, pos2, vel2, tqe2, kp2, kd2);
        break;
    case TINT32:
        set_pos_vel_tqe_pd_int32(fdcanHandle, id, pos2, vel2, tqe2, kp2, kd2);
        break;
    case TINT16:
        set_pos_vel_tqe_pd_int16(fdcanHandle, id, pos2, vel2, tqe2, kp2, kd2);
        break;
    default:
        break;
    }
}


void motor_set_state(FDCAN_HandleTypeDef *fdcanHandle, const data_type_t type, const uint8_t id)
{
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




