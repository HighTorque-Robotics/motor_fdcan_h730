#include "motor.h"



/************************************下面为需要修改的部分*******************************************/

static motor_state_s motor_state_port[MOTOR_PORT_NUM][MOTOR_MAX_NUM];


const port_mapping_s port_maping[MOTOR_PORT_NUM] =  // 通道映射表
{
    {
        .port = PORT1,
        .fdcan = &hfdcan1,
        .state = motor_state_port[0],
    },

    {
        .port = PORT2,
        .fdcan = &hfdcan2,
        .state = motor_state_port[1],
    },

    {
        .port = PORT3,
        .fdcan = &hfdcan3,
        .state = motor_state_port[2],
    },
};

/*******************************************END***************************************************/


p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (fdcanHandle->Instance == port_maping[i].fdcan->Instance)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


p_motor_state_s motor_get_state_pointer2(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].fdcan;
        }
    }

    MOTOR_ERR();
    return NULL;
}



void motor_print_state()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            motor_state_s *p_motor_state = motor_get_state(portx, id);

            printf("PORT: %d, ID: %2d, mode: %2d, temp: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", portx, id, p_motor_state->mode, p_motor_state->temp,
                   p_motor_state->fault, p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
        }
        printf("\r\n");
    }
}


void motor_print_version()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            const p_version_s p_version = &(motor_get_state(portx, id)->version);

            printf("PORT: %d, ID: %2d, version = %d.%d.%d\r\n", portx, id, p_version->major, p_version->minor, p_version->patch);
        }
        printf("\r\n");
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

    return &(motor_get_state_pointer2(portx)[index]);
}


/**
 * @brief 解析电机返回信息
 * @param fdcanHandle
 * @param id 电机 ID
 * @param p_data fdcan 帧数据指针
 * @param len fdcan 数据长度
 */
static void motor_process_state(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id, const uint32_t id_title, const uint8_t *p_data, const uint8_t len)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;

    // ===================== QUERY_MODE_FAULT_POS_VEL_TQE (0x0B) 响应 =====================
    if (p_data[0] == MANY_GET_MODE_FLAUT_POS_VEL_TQE)
    {
        // --------- TINT16: 响应 ID=0x10xxx, bits[17:16]=01 ---------
        if (id_title == ID_PREFIX_TINT16)
        {
            int16_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel, p_data + 5, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe, p_data + 7, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT16);
        }
        // --------- TINT32: 响应 ID=0x20xxx, bits[17:16]=10 ---------
        else if (id_title == ID_PREFIX_TINT32 )
        {
            int32_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(int32_t));
            my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(int32_t));
            my_memcpy((uint8_t *)&tqe, p_data + 11, sizeof(int32_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT32);
        }
        // --------- TFLOAT: 响应 ID=0x30xxx, bits[17:16]=11 ---------
        else if (id_title == ID_PREFIX_TFLOAT )
        {
            float pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(float));
            my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(float));
            my_memcpy((uint8_t *)&tqe, p_data + 11, sizeof(float));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = (uint8_t)p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].position  = conv_from_turns(pos, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe;
        }
    }
    
    // ===================== QUERY_MODE_FAULT_TEMP_POS_VEL_TQE (0x0C) 响应 =====================
    else if (p_data[0] == MANY_GET_MODE_FLAUT_TEMP_POS_VEL_TQE)
    {
        // --------- TINT16: 响应 ID=0x10xxx, bits[17:16]=01 ---------
        if (id_title == ID_PREFIX_TINT16 )
        {
            int16_t pos = 0, vel = 0, tqe = 0, temp_raw = 0;

            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&pos,     p_data + 5, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel,     p_data + 7, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe,     p_data + 9, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)(temp_raw / 10);  // 0.1°C/LSB
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT16);
        }
        // --------- TINT32: 响应 ID=0x20xxx, bits[17:16]=10 ---------
        else if (id_title == ID_PREFIX_TINT32 )
        {
            int32_t pos = 0, vel = 0, tqe = 0, temp_raw = 0;

            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(int32_t));
            my_memcpy((uint8_t *)&pos,     p_data + 7, sizeof(int32_t));
            my_memcpy((uint8_t *)&vel,     p_data + 11, sizeof(int32_t));
            my_memcpy((uint8_t *)&tqe,     p_data + 15, sizeof(int32_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)(temp_raw / 1000);  // 0.001°C/LSB
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT32), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT32);
        }
        // --------- TFLOAT: 响应 ID=0x30xxx, bits[17:16]=11 ---------
        else if (id_title == ID_PREFIX_TFLOAT )
        {
            float pos = 0, vel = 0, tqe = 0, temp_raw = 0;

            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(float));
            my_memcpy((uint8_t *)&pos,     p_data + 7, sizeof(float));
            my_memcpy((uint8_t *)&vel,     p_data + 11, sizeof(float));
            my_memcpy((uint8_t *)&tqe,     p_data + 15, sizeof(float));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = (uint8_t)p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)(temp_raw);  // 1°C/LSB
            p_motor_state[id_index].position  = conv_from_turns(pos, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel, MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe;
        }
    }

    // ===================== 电机固件版本 (0x04) =====================
    // 返回帧: 04 | patch | minor | major (各1字节)
    else if (p_data[0] == 0x04)
    {
        p_motor_state[id_index].version.major = p_data[3];
        p_motor_state[id_index].version.minor = p_data[2];
        p_motor_state[id_index].version.patch = p_data[1];
    }
    // ===================== 电机型号查询响应 =====================
    // 注意: 必须在"一拖多"分支之前, 否则会被 id_index < MOTOR_MAX_NUM 拦截
    else if (p_data[0] == 0x07)
    {
        const uint8_t model_len = p_data[1];

        if (model_len > 0 && model_len <= 15 && len >= model_len + 2)
        {
            char model_str[16] = {0};

            // 每字节取低 4 位，转为十六进制字符
            for (uint8_t i = 0; i < model_len; i++)
            {
                const uint8_t nibble = p_data[2 + i] & 0x0F;
                model_str[i] = (nibble < 10) ? ('0' + nibble) : ('A' + (nibble - 10));
            }

            // 查找当前 fdcanHandle 对应的端口号
            uint8_t port_num = 0;
            for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
            {
                if (fdcanHandle->Instance == port_maping[i].fdcan->Instance)
                {
                    port_num = port_maping[i].port;
                    break;
                }
            }
        }
    }
    // ===================== 一拖多模式解析 (仅处理 0x0B/0x0C 帧, 格式与普通模式 TINT16 一致) =====================
    else if (id_index < MOTOR_MAX_NUM && (p_data[0] == MANY_GET_MODE_FLAUT_POS_VEL_TQE || p_data[0] == MANY_GET_MODE_FLAUT_TEMP_POS_VEL_TQE))
    {
        int16_t pos = 0, vel = 0, tqe = 0;

        // query=0x0C: 返回温度+模式+错误+位置+速度+力矩
        if (p_data[0] == MANY_GET_MODE_FLAUT_TEMP_POS_VEL_TQE)
        {
            int16_t temp_raw = 0;
            my_memcpy((uint8_t *)&temp_raw, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&pos,     p_data + 5, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel,     p_data + 7, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe,     p_data + 9, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
            p_motor_state[id_index].temp      = (int8_t)(temp_raw / 10);  // 0.1°C/LSB
        }
        // query=0x0B: 返回模式+错误+位置+速度+力矩
        else if (p_data[0] == MANY_GET_MODE_FLAUT_POS_VEL_TQE)
        {
            my_memcpy((uint8_t *)&pos, p_data + 3, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel, p_data + 5, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe, p_data + 7, sizeof(int16_t));

            p_motor_state[id_index].query     = p_data[0];
            p_motor_state[id_index].mode      = p_data[1];
            p_motor_state[id_index].fault     = p_data[2];
        }
        else
        {
            return;
        }

        p_motor_state[id_index].position = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
        p_motor_state[id_index].velocity = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
        p_motor_state[id_index].torque = tqe_int2float(tqe, TINT16);
    }
}




static FDCAN_RxHeaderTypeDef fdcan_rx_header;
static uint8_t fdcan_rdata[64] = {0};

/**
 * @brief 解析所有 CAN 通道 FIFO 中的电机状态数据
 *
 */
void motor_process_state_all()
{
    for (int i = 0; i < MOTOR_PORT_NUM; i++)
    {
        while (HAL_FDCAN_GetRxMessage(port_maping[i].fdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata) == HAL_OK)
        {
            if (fdcan_rx_header.DataLength != 0)
            {
                const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

                const uint32_t id_title = fdcan_rx_header.Identifier & 0x00030000UL;  // 提取 bits[17:16] 数据类型
                const uint8_t  motor_id = (fdcan_rx_header.Identifier >> 8) & 0x7F;  // 提取 bits[14:8] 主机ID (电机返回ID, 1~127)


                if (motor_id > 0 && motor_id <= MOTOR_MAX_NUM)
                {
                    motor_process_state(port_maping[i].fdcan, motor_id, id_title, fdcan_rdata, len);
                }
            }
        }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)
    {
        // while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata) == HAL_OK)
        // {
        //     if (fdcan_rx_header.DataLength != 0)
        //     {
        //         const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

        //         motor_process_state(hfdcan, fdcan_rx_header.Identifier >> 8, fdcan_rdata, len);
        //     }
        // }
    }
}
