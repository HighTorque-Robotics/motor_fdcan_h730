#ifndef _MOTOR_H
#define _MOTOR_H




#define  MOTOR_PORT_NUM  1  // 使用 CAN 通道数量  
#define  MOTOR_MAX_NUM   10// 单个 CAN 通道所连接的最大电机数量



#include "fdcan.h"
#include "my_fdcan.h"
#include "convert.h"
#include "livelybot_fdcan.h"


/* ============================================================
 *  电机控制模式码 (控制帧)
 * ============================================================ */
typedef enum
{
    MOTOR_MODE_STOP        = 0x01,  /**< 停止 (惯性停止)            */
    MOTOR_MODE_BRAKE       = 0x18,  /**< 刹车 (主动制动)            */
    MOTOR_MODE_VOLT        = 0x19,  /**< DQ 电压控制                */
    MOTOR_MODE_CUR         = 0x1A,  /**< DQ 电流控制                */
    MOTOR_MODE_TQE         = 0x1B,  /**< 力矩控制                   */
    MOTOR_MODE_VEL         = 0x1C,  /**< 速度控制                   */
    MOTOR_MODE_POS         = 0x1D,  /**< 位置控制                   */
    MOTOR_MODE_VEL_ACC     = 0x1E,  /**< 速度 + 加速度             */
    MOTOR_MODE_POS_VEL_TQE = 0x1F,  /**< 位置 + 速度 + 前馈力矩    */
    MOTOR_MODE_POS_VEL_ACC = 0x20,  /**< 位置 + 速度 + 加速度(梯形) */
    MOTOR_MODE_MIT         = 0x21,  /**< MIT 运控 (位置+速度+力矩+Kp+Kd) */
} motor_mode_t;

/* ============================================================
 *  电机查询码 (查询帧)
 * ============================================================ */
typedef enum
{
    MANY_GET_MODE_FLAUT_NUM              = 0x0A,  // 模式、错误码、NUM(查询指令 0x0A)
    MANY_GET_MODE_FLAUT_POS_VEL_TQE      = 0x0B,  // 模式、错误码、位置、速度、力矩 (查询指令 0x0B)
    MANY_GET_MODE_FLAUT_TEMP_POS_VEL_TQE = 0x0C,  // 模式、温度、错误码、位置、速度、力矩（查询指令 0x0C）
} many_request_type_t;


typedef enum
{
    PNULL = 0,
    PORT1,
    PORT2,
    PORT3,
} port_t;


typedef struct
{
    uint8_t major : 4;
    uint8_t minor : 8;
    uint8_t patch : 4;
} version_s, *p_version_s;


typedef struct
{
    uint8_t mode;      // 模式（对应 motor_mode_t 枚举）
    uint8_t fault;     // 错误码
    float position;  // 位置
    float velocity;  // 速度
    float torque;    // 力矩
    int8_t  temp;      // 温度（单位：摄氏度，分辨率：1度）
    uint8_t query;     // 最后响应的查询码（对应 prot_query_t 枚举）
    uint8_t ack;     // 应答，用于电机设置相关的应答
    version_s version;  // 电机固件版本号
} motor_state_s, *p_motor_state_s;  // 这个结构体会定义成结构体数组，其中数组下标 +1 即为电机 ID


typedef struct
{
    const port_t port;
    FDCAN_HandleTypeDef *fdcan;
    const p_motor_state_s state;
} port_mapping_s, *p_port_mapping_s;




void motor_print_state(void);
void motor_print_version(void);

p_motor_state_s motor_get_state(port_t portx, uint8_t id);

FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx);
p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle);
p_motor_state_s motor_get_state_pointer2(port_t portx);
void motor_process_state_all(void);




#define  MOTOR_SDK_VERSION   "1.0.0"

#endif
