#include "motor_many.h"

#ifdef __MICROLIB  // ”–Œﬁ∆Ù”√MicroLIBø‚
#include <string.h>
#endif



many_data_s many_data_port1;





void motor_many_volt(many_data_s *p_many_data, const uint8_t id, const float vol)
{
    const int16_t vol_int16 = vol_float2int(vol, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_VOLTAGE)
    {
        p_many_data->mode = MODE_VOLTAGE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->voltage[i] = 0;
        }
    }

    p_many_data->voltage[index] = vol_int16;
}


void motor_many_current(many_data_s *p_many_data, const uint8_t id, const float cur)
{
    const int16_t cur_int16 = cur_float2int(cur, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_CURRENT)
    {
        p_many_data->mode = MODE_CURRENT;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->current[i] = 0;
        }
    }

    p_many_data->current[index] = cur_int16;
}


void motor_many_pos(many_data_s *p_many_data, const uint8_t id, const float pos)
{
    const int16_t pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POSITION)
    {
        p_many_data->mode = MODE_POSITION;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->position[i] = NAN_INT16;
        }
    }

    p_many_data->position[index] = pos_int16;
}


void motor_many_vel(many_data_s *p_many_data, const uint8_t id, const float vel)
{
    const int16_t vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_VELOCITY)
    {
        p_many_data->mode = MODE_VELOCITY;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->velocity[i] = 0;
        }
    }

    p_many_data->velocity[index] = vel_int16;
}


void motor_many_tqe(many_data_s *p_many_data, const uint8_t id, const float tqe)
{
    const int16_t tqe_int16 = tqe_float2int(tqe, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_TORQUE)
    {
        p_many_data->mode = MODE_TORQUE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->torque[i] = 0;
        }
    }

    p_many_data->torque[index] = tqe_int16;
}


void motor_many_pos_vel(many_data_s *p_many_data, const uint8_t id, const float pos, const float vel)
{
    const int16_t pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_TQE)
    {
        p_many_data->mode = MODE_POS_VEL_TQE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_s); i++)
        {
            p_many_data->pos_vel_tqe[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe[i].vel = 0;
            p_many_data->pos_vel_tqe[i].tqe = 0;
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_int16;
    p_many_data->pos_vel_tqe[index].vel = vel_int16;
    p_many_data->pos_vel_tqe[index].tqe = NAN_INT16;
}


void motor_many_pos_vel_tqe(many_data_s *p_many_data, const uint8_t id, const float pos, const float vel, const float tqe)
{
    const int16_t pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t tqe_int16 = tqe_float2int(tqe, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_TQE)
    {
        p_many_data->mode = MODE_POS_VEL_TQE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_s); i++)
        {
            p_many_data->pos_vel_tqe[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe[i].vel = 0;
            p_many_data->pos_vel_tqe[i].tqe = 0;
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_int16;
    p_many_data->pos_vel_tqe[index].vel = vel_int16;
    p_many_data->pos_vel_tqe[index].tqe = tqe_int16;
}


void motor_many_pos_vel_acc(many_data_s *p_many_data, const uint8_t id, const float pos, const float vel, const float acc)
{
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t acc_int16 = acc_float2int(acc_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_ACC)
    {
        p_many_data->mode = MODE_POS_VEL_ACC;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_acc_s); i++)
        {
            p_many_data->pos_vel_tqe[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe[i].vel = 0;
            p_many_data->pos_vel_tqe[i].tqe = 0;
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_int16;
    p_many_data->pos_vel_tqe[index].vel = vel_int16;
    p_many_data->pos_vel_tqe[index].tqe = acc_int16;
}


void motor_many_pos_vel_tqe_kp_kd(many_data_s *p_many_data, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t acc_int16 = tqe_float2int(tqe, TINT16);
    const int16_t kp_int16 = pid_float2int(kp, TINT16);
    const int16_t kd_int16 = pid_float2int(kd, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_ACC)
    {
        p_many_data->mode = MODE_POS_VEL_ACC;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_kd_s); i++)
        {
            p_many_data->pos_vel_tqe_kp_kd[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe_kp_kd[i].vel = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].tqe = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].kp = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].kd = 0;
        }
    }

    p_many_data->pos_vel_tqe_kp_kd[index].pos = pos_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].vel = vel_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].tqe = acc_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].kp = kp_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].kd = kd_int16;
}


void motor_many_pos_vel_tqe_kp_ki_kd(many_data_s *p_many_data, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float ki, const float kd)
{
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t acc_int16 = tqe_float2int(tqe, TINT16);
    const int16_t kp_int16 = pid_float2int(kp, TINT16);
    const int16_t ki_int16 = pid_float2int(ki, TINT16);
    const int16_t kd_int16 = pid_float2int(kd, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_ACC)
    {
        p_many_data->mode = MODE_POS_VEL_ACC;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_ki_kd_s); i++)
        {
            p_many_data->pos_vel_tqe_kp_ki_kd[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe_kp_ki_kd[i].vel = 0;
            p_many_data->pos_vel_tqe_kp_ki_kd[i].tqe = 0;
            p_many_data->pos_vel_tqe_kp_ki_kd[i].kp = 0;
            p_many_data->pos_vel_tqe_kp_ki_kd[i].ki = 0;
            p_many_data->pos_vel_tqe_kp_ki_kd[i].kd = 0;
        }
    }

    p_many_data->pos_vel_tqe_kp_ki_kd[index].pos = pos_int16;
    p_many_data->pos_vel_tqe_kp_ki_kd[index].vel = vel_int16;
    p_many_data->pos_vel_tqe_kp_ki_kd[index].tqe = acc_int16;
    p_many_data->pos_vel_tqe_kp_ki_kd[index].kp = kp_int16;
    p_many_data->pos_vel_tqe_kp_ki_kd[index].ki = ki_int16;
    p_many_data->pos_vel_tqe_kp_ki_kd[index].kd = kd_int16;
}



static uint8_t get_data_max(uint8_t mode)
{
    switch (mode)
    {
    case (MODE_POS_VEL_RKP_RKD):
    case (MODE_POS_VEL_KP_KD):
        return 56;
    default:
        return 60;
    }
}


static uint8_t get_motor_data_len(uint16_t size)
{
    uint8_t data_len = 0;

    size += 2;
    if (size <= 8)
    {
        data_len = size;
    }
    else if (size <= 12)
    {
        data_len = 12;
    }
    else if (size <= 16)
    {
        data_len = 16;
    }
    else if (size <= 20)
    {
        data_len = 20;
    }
    else if (size <= 24)
    {
        data_len = 24;
    }
    else if (size <= 32)
    {
        data_len = 32;
    }
    else if (size <= 48)
    {
        data_len = 48;
    }
    else
    {
        data_len = 64;
    }

    return data_len;
}



void motor_many_send(FDCAN_HandleTypeDef *fdcanHandle, many_data_s *p_many_data)
{
    static uint8_t cmd[64] = {0};
    const uint8_t read_state_cmd1[] = {0xFF, 0xFF};
    uint8_t id = p_many_data->mode;

    uint16_t remaining_len = MANY_DATA_BUF_MAX_LEN;
    uint8_t data_len_max = get_data_max(id);
    uint8_t *data = p_many_data->data;

    while (remaining_len > 0)
    {
        const uint8_t current_data_len = (remaining_len > data_len_max) ? data_len_max : remaining_len;
        uint8_t cmd_len = get_motor_data_len(current_data_len);

        my_memcpy(cmd, data, current_data_len);
        data += current_data_len;
        remaining_len -= current_data_len;
        my_memcpy(cmd + cmd_len - 2, read_state_cmd1, sizeof(read_state_cmd1));
        fdcan_send(fdcanHandle, 0x8000 | id, cmd, cmd_len);
        ++id;
    }
}

