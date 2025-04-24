#include "motor.h"


static motor_state_s motor_state_port1[MOTOR_MAX_NUM] =
{
    {
        .id = 1,
        .model = M4438_30,
    },

    {
        .id = 2,
        .model = M5047_36,
    }
};


static motor_state_s motor_state_port2[MOTOR_MAX_NUM] =
{
    {
        .id = 1,
        .model = M5047_36,
    },

    {
        .id = 2,
        .model = M5047_36,
    }
};


many_data_s many_data_port1;
many_data_s many_data_port2;




void motor_print_state()
{
    printf("\r\n");
    for (uint8_t i = 0; i < MOTOR_MAX_NUM; i++)
    {
        printf("id:%2d, mode:%2d, fault:%2d, pos:%.3lf, vel:%.3lf, tqe:%.3lf\r\n", motor_state_port1[i].id, motor_state_port1[i].mode, motor_state_port1[i].fault,
               motor_state_port1[i].position, motor_state_port1[i].velocity, motor_state_port1[i].torque);
    }

    printf("\r\n");
    for (uint8_t i = 0; i < MOTOR_MAX_NUM; i++)
    {
        printf("id:%2d, mode:%2d, fault:%2d, pos:%.3lf, vel:%.3lf, tqe:%.3lf\r\n", motor_state_port2[i].id, motor_state_port2[i].mode, motor_state_port2[i].fault,
               motor_state_port2[i].position, motor_state_port2[i].velocity, motor_state_port2[i].torque);
    }
}


void motor_print_version()
{
    printf("\r\n");
    for (uint8_t i = 0; i < MOTOR_MAX_NUM; i++)
    {
        printf("ID:%2d, version=%d.%d.%d\r\n", motor_state_port1[i].id,
               motor_state_port1[i].version.major, motor_state_port1[i].version.minor, motor_state_port1[i].version.patch);
    }

    printf("\r\n");
    for (uint8_t i = 0; i < MOTOR_MAX_NUM; i++)
    {
        printf("ID:%2d, version=%d.%d.%d\r\n", motor_state_port2[i].id,
               motor_state_port2[i].version.major, motor_state_port2[i].version.minor, motor_state_port2[i].version.patch);
    }
}


/**
 * @brief 获取指定端口和ID的电机状态指针
 * @param portx 指定电机所在的端口，可能的值为 PORT1 或 PORT2
 * @param id 电机 ID
 * @return 返回类型为 `p_motor_state_s` 的指针
 */
p_motor_state_s motor_get_state(port_t portx, uint8_t id)
{
    const uint8_t index = id - 1;

    switch (portx)
    {
    case PORT1:
        return &(motor_state_port1[index]);

    case PORT2:
        return &(motor_state_port2[index]);

    default:
        return NULL;
    }

    return NULL;
}


p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle->Instance == FDCAN1)
    {
        return motor_state_port1;
    }
    else if (fdcanHandle->Instance == FDCAN2)
    {
        return motor_state_port2;
    }

    return NULL;
}


p_motor_state_s motor_get_state_pointer2(port_t portx)
{
    if (portx == PORT1)
    {
        return motor_state_port1;
    }
    else if (portx == PORT2)
    {
        return motor_state_port2;
    }

    return NULL;
}


p_many_data_s motor_get_many_pointer(port_t portx)
{
    if (portx == PORT1)
    {
        return &many_data_port1;
    }
    else if (portx == PORT2)
    {
        return &many_data_port2;
    }

    return NULL;
}


FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx)
{
    if (portx == PORT1)
    {
        return &hfdcan1;
    }
    else if (portx == PORT2)
    {
        return &hfdcan2;
    }

    return NULL;
}


uint8_t motor_get_model1(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    if (id < 0 || id > MOTOR_MAX_NUM)
    {
        led_toggle_err();
        return MNULL;
    }

    const p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    return p_motor_state[id - 1].model;
}


uint8_t motor_get_model2(port_t portx, uint8_t id)
{
    if (id < 0 || id > MOTOR_MAX_NUM)
    {
        led_toggle_err();
        return MNULL;
    }

    const p_motor_state_s p_motor_state = motor_get_state_pointer2(portx);
    const motor_type_t model = p_motor_state[id - 1].model;
    return model;
}




/**
 * @brief 解析电机返回信息
 * @param fdcanHandle
 * @param id 电机 ID
 * @param p_data fdcan 帧数据指针
 * @param len fdcan 数据长度
 */
static void motor_process_state(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id, const uint8_t *p_data, const uint8_t len)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;

    if (p_data[0] == 0x24 && p_data[1] == 0x04 && p_data[2] == 0x00  // TINT16 解析
            && p_data[11] == 0x21 && p_data[12] == 0x0F)
    {
        int16_t pos = 0;
        int16_t vel = 0;
        int16_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 5, sizeof(int16_t));
        my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(int16_t));
        my_memcpy((uint8_t *)&tqe, p_data + 9, sizeof(int16_t));

        p_motor_state[id_index].mode = p_data[3];
        p_motor_state[id_index].position = pos_int2float(pos, TINT16);
        p_motor_state[id_index].velocity = vel_int2float(vel, TINT16);
        const float tqe_temp = tqe_int2float(tqe, TINT16);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
        p_motor_state[id_index].fault = (uint8_t)p_data[13];
    }
    else if (p_data[0] == 0x28 && p_data[1] == 0x04 && p_data[2] == 0x00  // TINT32 解析
             && p_data[19] == 0x21 && p_data[20] == 0x0F)
    {
        int32_t pos = 0;
        int32_t vel = 0;
        int32_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 7, sizeof(int32_t));
        my_memcpy((uint8_t *)&vel, p_data + 11, sizeof(int32_t));
        my_memcpy((uint8_t *)&tqe, p_data + 15, sizeof(int32_t));

        p_motor_state[id_index].mode = p_data[3];
        p_motor_state[id_index].position = pos_int2float(pos, TINT32);
        p_motor_state[id_index].velocity = vel_int2float(vel, TINT32);
        const float tqe_temp = tqe_int2float(tqe, TINT32);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
        p_motor_state[id_index].fault = (uint8_t)p_data[21];
    }
    else if (p_data[0] == 0x2C && p_data[1] == 0x04 && p_data[2] == 0x00  // TFLOAT 解析
             && p_data[19] == 0x21 && p_data[20] == 0x0F)
    {
        float pos = 0;
        float vel = 0;
        float tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 7, sizeof(float));
        my_memcpy((uint8_t *)&vel, p_data + 11, sizeof(float));
        my_memcpy((uint8_t *)&tqe, p_data + 15, sizeof(float));

        p_motor_state[id_index].mode = p_data[3];
        p_motor_state[id_index].position = pos_int2float(pos, TFLOAT);
        p_motor_state[id_index].velocity = vel_int2float(vel, TFLOAT);
        const float tqe_temp = tqe_int2float(tqe, TFLOAT);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
        p_motor_state[id_index].fault = (uint8_t)p_data[21];
    }
    else if (id_index < MANY_MOTOR_SIZE && len == 8)   // 一拖多模式解析
    {
        int16_t pos = 0;
        int16_t vel = 0;
        int16_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 2, sizeof(int16_t));
        my_memcpy((uint8_t *)&vel, p_data + 4, sizeof(int16_t));
        my_memcpy((uint8_t *)&tqe, p_data + 6, sizeof(int16_t));

        p_motor_state[id_index].mode = p_data[0];
        p_motor_state[id_index].fault = p_data[1];

        p_motor_state[id_index].position = pos_int2float(pos, TINT16);
        p_motor_state[id_index].velocity = vel_int2float(vel, TINT16);
        const float tqe_temp = tqe_int2float(tqe, TINT16);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
    }
    else if (len == 7 && p_data[0] == 0x41 && p_data[1] == 0x01 && p_data[2] == 0x04  // 设置信息解析
             && p_data[3] == 0x4F && p_data[4] == 0x4B && p_data[5] == 0x0D && p_data[6] == 0x0A)
    {
        p_motor_state[id_index].ack = 1;
    }
    else if (p_data[1] == 0xB5 && p_data[2] == 0x02)  // 电机固件版本
    {
        if (len == 5)
        {
            p_motor_state[id_index].version.major = p_data[4] >> 4;
            p_motor_state[id_index].version.minor = p_data[4] & 0x0F | p_data[3] >> 4;
            p_motor_state[id_index].version.patch = p_data[3] & 0x0F;
        }
        else
        {
            p_motor_state[id_index].version.major = 3;
            p_motor_state[id_index].version.minor = 9;
            p_motor_state[id_index].version.patch = 1;
        }
    }
}


static FDCAN_RxHeaderTypeDef fdcan_rx_header;
static uint8_t fdcan_rdata[64] = {0};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata) == HAL_OK)
        {
            if (fdcan_rx_header.DataLength != 0)
            {
                const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

                motor_process_state(hfdcan, fdcan_rx_header.Identifier >> 8, fdcan_rdata, len);
            }
        }
    }
}
