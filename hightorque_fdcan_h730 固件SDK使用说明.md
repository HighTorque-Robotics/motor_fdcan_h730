# hightorque fdcan\_h730例程使用说明

`hightorque_fdcan_h730` 是面向高擎（hightorque）电机的电机控制 例程（C 语言 / Keil 工程）。

应用层负责规划控制目标并周期下发，例程 负责将目标按 FDCAN 协议发送到对应通道、持续接收反馈并整理成可读取的状态。

# 1.相关文档链接

# 2. 项目概览

## 2.1 核心模块与职责

例程 按「底层协议 → 上层封装」分为多层，各目录/模块职责如下：

| 模块               | 对应目录                | 职责                                                         |
| ------------------ | ----------------------- | ------------------------------------------------------------ |
| `hightorque_fdcan` | `src/hightorque_fdcan/` | 底层协议封装（`hightorque_*` 发送函数）<br>与具体控制模式一一对应 |
| `convert`          | `src/convert/`          | 单位换算(可更换)：弧度/角度/圈数 ↔ float ↔ int<br>（`conv_to_turns` / `*_float2int`） |
| `motor`            | `src/motor/`            | 电机状态管理：`motor_state_s` 全局数组<br>（下标 +1 即电机 ID）<br>`motor_process_state_all` 返回帧解析 |
| `motor_control`    | `src/motor_control/`    | 单电机控制封装（`motor_*`）：<br>单位换算后立即调用底层并发送 |
| `motor_many`       | `src/motor_many/`       | 一拖多批量控制<br>（`motor_many_*` 打包 + `motor_many_send` 统一发送） |
| `motor_config`     | `src/motor_config/`     | 电机配置（改 ID / 重置零位 / 周期返回，闭环确认）            |

## 2.2 目录结构

```plaintext
fdcan_h730/
├── Core/
│   ├── Inc/
│   └── Src/
│       ├── main.c             # 主循环：1kHz 控制例程 + 状态解析 + 500ms 打印
│       └── fdcan.c            # FDCAN 外设初始化（1M/5M、FD、BRS）
├── src/                       # 源码实现（电机协议核心）
│   ├── hightorque_fdcan/      # 底层通信协议封装（hightorque_* 发送函数）
│   ├── convert/               # 单位换算（弧度/角度/圈数，float↔int）
│   ├── motor/                 # 电机状态管理（返回帧解析）
│   ├── motor_control/         # 单电机控制封装（motor_*）
│   ├── motor_many/            # 一拖多控制（motor_many_*）
│   └── motor_config/          # 电机配置（改 ID / 重置零位 / 周期返回）
├── test/                      # 示例程序（demo）
    ├── test_motor/            # 单电机控制测试（test_motor_control）
    └── test_motor_many/       # 一拖多控制测试（test_motor_many / stop）
```

## 2.3 适用版本

*   例程版本号：`v4.0.0`

*   电机固件：`v0.3.3+`

*   通信协议：FDCAN，仲裁段 1M / 数据段 5M

*   主控芯片：`STM32H730`

*   开发环境：`Keil MDK-ARM``STM32CubeMX`

---

# 3. 环境与安全

## 3.1 硬件准备

*   一块高擎主控板（本工程演示配置：`STM32H730`，默认 2 路 FDCAN），主控板引脚配置适用 `v1.6+`；

*   一台或多台高擎电机，供电正常；

*   主控与电机通过 FDCAN 接线：`CAN_TX` / `CAN_RX` 对应连接；

*   总线两端建议加 **120Ω 终端电阻**；

*   电机 ID 有效范围 `[1, 126]`（127 为广播地址，不可作为电机 ID）。

## 3.2 安全注意事项

*   首次测试使用**低速、小范围目标**，确认转动方向与预期一致；

*   电机运动前确认机械结构固定、无干涉，并准备**独立急停或断电手段**；

*   软件 `motor_stop()` 只发送正常停止命令，**不能替代硬件急停/断电**；

*   位置模式（`motor_pos`）速度与力矩均为最大，运动较激烈，慎用；推荐梯形控制。

---

# 4. 快速上手与设备检测

## 4.1 运行代码

1.  **改配置**：打开 `src/motor/motor.h`，把 `MOTOR_MAX_NUM` 设为「单通道实际电机数」、`MOTOR_PORT_NUM` 设为「实际使用通道数」。

2.  **选单位制**：打开 `src/convert/convert.h`，确认 `MOTOR_DATA_TYPE_FLAG`（默认 `TURNS` 圈数；可选 `RADIAN_2PI` 弧度 / `ANGLE_360` 角度）。

3.  **编译下载并运行例程**：

    *   用 Keil 打开 `MDK-ARM/fdcan_h730.uvprojx`，编译后烧录；

    *   在 `Core/Src/main.c` 主循环中调用示例 `test_motor_control()`（单电机）或 `test_motor_many()`（一拖多）；

    *   上电后通过串口观察 `motor_print_state()` 输出（默认每 500ms 打印各通道各电机的 `mode / temp / fault / pos / vel / tqe`）。

**验证成功标志**：串口能持续打印非 0、且随指令变化的 `pos / vel / tqe`，且 `fault` 为 0。若一直为 0 或无打印，见「11. 故障排查」。

## 5. 示例程序

示例位于 `test/`，共 2 个目录，均依赖真实硬件（电机 + FDCAN 接线），无硬件无法观测效果。

| 示例       | 入口                           | 说明                                                         |
| ---------- | ------------------------------ | ------------------------------------------------------------ |
| 单电机     | `test_motor_control(id)`       | 通过 `mode` 变量（0~14）切换：DQ 电压 / DQ 电流 / 位置 / 速度 / 力矩 / 位置+速度 / 位置+速度+最大力矩 / 梯形 / MIT / 速度+加速度 / 停止 / 读取状态 / 读取版本 / 刹车 / 读取型号。默认 `mode = 3`（速度控制） |
| 一拖多     | `test_motor_many()`            | 通过 `mode` 变量切换一拖多各模式，循环遍历本通道所有电机写入缓冲后统一 `motor_many_send()` |
| 一拖多停止 | `test_motor_many_stop(enable)` | 演示批量停止并连续发送 3 次查询帧确保电机响应                |

---

## 6. 配置

配置入口为 `src/motor/motor.h` 与 `src/convert/convert.h`，修改后需重新编译。

### 6.1 通道与电机数（`src/motor/motor.h`）

```c
#define MOTOR_PORT_NUM  2   // CAN 通道数量（当前 2 路）
#define MOTOR_MAX_NUM   2   // 每通道电机数量（需 ≥ 实际电机 ID 最大值）

```

`MOTOR_MAX_NUM` 决定状态数组大小；电机 ID 从 1 开始连续，**ID > MOTOR\_MAX\_NUM 的返回帧不会被解析**（见 `motor.c` 入口判断）。

### 6.2 更换单位（`src/convert/convert.h`）

```c
#define MOTOR_DATA_TYPE_FLAG  TURNS   // RADIAN_2PI（弧度）/ ANGLE_360（角度）/ TURNS（圈数）

```

角度类参数（`pos` / `vel` / `acc`）统一按此宏换算；`tqe`（力矩）固定为 N·m，不随单位制变化。

### 6.3 电机 ID 范围（`src/convert/convert.h`）

```c
#define BROADCAST_ID  0x7F
#define MOTOR_ID_MIN  1
#define MOTOR_ID_MAX  (BROADCAST_ID - 1)   // 126

```

### 6.4 通道映射（`src/motor/motor.c` 的 `port_maping`）

| 通道    | 外设      | 说明                                            |
| ------- | --------- | ----------------------------------------------- |
| `PORT1` | `hfdcan1` | 默认使用的通道                                  |
| `PORT2` | `hfdcan2` | 第二路通道                                      |
| `PORT3` | `hfdcan3` | 默认注释，需自行启用（含 `port_maping` 补条目） |

如需更多通道，在 `port_maping` 中增加条目即可。

---

## 7. 初始化与运行结构

### 7.1 初始化流程

上电后由 CubeMX 生成的外设初始化（`MX_FDCAN1_Init` / `MX_FDCAN2_Init`）完成 FDCAN 波特率（1M/5M）、FD 帧、BRS 配置；随后 `fdcan_filter_init(fdcanHandle)` 配置全局过滤器、使能接收中断与发送 FIFO 空中断并启动 FDCAN。

### 7.2 主循环（`Core/Src/main.c`）

```c
while (1)
{
    /* 1kHz: 控制例程调用 */
    if (HAL_GetTick() - tick_ctrl >= 1)
    {
        tick_ctrl = HAL_GetTick();
        test_motor_many();          // 或 test_motor_control(id)
    }

    motor_process_state_all();      // 解析所有通道返回帧

    /* 500ms: 终端打印 + LED */
    if (HAL_GetTick() - tick_print >= 500)
    {
        tick_print = HAL_GetTick();
        led_toggle();
        motor_print_state();
    }
}

```

控制例程按需下发指令；一拖多方式需先写缓冲再 `motor_many_send()` 统一发送。

### 7.3 反馈机制

*   控制帧（除软重启外）**自带状态查询**：电机收到控制指令后按查询码返回一帧状态，`motor_process_state_all()` 自动解析并写入对应 `motor_state_s`；

*   解析由主循环周期调用完成，无需用户干预。

---

## 8. 接口层次

### 8.1 单电机方式（`motor_control`）

*   直接调用 `motor_*(portx, type, id, ...)`，一次控制一台电机；

*   控制函数**内部立即发送**（单位换算 → `hightorque_*` → `fdcan_send`），无需额外 `send()`；

*   适合小电机数、逐个控制的场景。

### 8.2 一拖多方式（`motor_many`）

*   先调用 `motor_many_*(portx, id, ...)` 把多台电机指令打包进缓冲区（`many_data_s`）；

*   再调用 `motor_many_send(portx, request_type)` **统一发送**（自动切分多帧、ID 递增、查询码填帧尾）；

*   适合多电机（每条 CAN 通道 ≤ 30 台）批量控制的场景。

### 8.3 底层接口（`hightorque_fdcan`）

`hightorque_*` 是协议层最小发送单元，按数据类型分为 `*_float` / `*_int32` / `*_int16` 三组。`motor_control` / `motor_many` 只是单位换算与打包封装，最终都落到 `hightorque_*` + `fdcan_send` 的同一套协议，**可以混用**。

### 8.4 使用方式对比

| 方式   | 控制调用                            | 发送方式                     | 状态读取                    | 适用场景               |
| ------ | ----------------------------------- | ---------------------------- | --------------------------- | ---------------------- |
| 单电机 | `motor_pos(PORT1, TFLOAT, 1, 1.0f)` | 内部立即发送                 | `motor_get_state(PORT1, 1)` | 电机少、逐个控制       |
| 一拖多 | `motor_many_pos(PORT1, 1, 1.0f)`    | `motor_many_send()` 统一发送 | 同上                        | 单通道 ≤ 30 台批量控制 |

---

## 9. 控制与反馈

### 9.1 单电机控制函数（`motor_control` / `hightorque`）

所有单电机控制函数最终实现在 `src/hightorque_fdcan/hightorque_fdcan.c`（`hightorque_*`），`motor_control.c` 的 `motor_*` 做单位换算后调用。调用形式统一为：

```c
motor_xxx(portx, type, id, 参数...);

```

*   `portx`：通道（`PORT1` / `PORT2`）；

*   `type`：数据类型（`TINT16` / `TINT32` / `TFLOAT`），影响精度与量程；

*   `id`：电机 ID，范围 `[1, MOTOR_MAX_NUM]`；

*   每个控制函数**内部立即发送**，无需额外 `send()`。

#### DQ 电压控制模式

设置 D/Q 轴电压（本工程固定 `d=0`，仅设置 q 轴）。

*   `volt`：Q 相电压，单位 V，例：`0.3 -> 0.3V`。

```c
motor_dq_vlot(PORT1, TFLOAT, id, 0.3f);

```

#### DQ 电流控制模式

设置 D/Q 轴电流（本工程固定 `d=0`，仅设置 q 轴）。

*   `cur`：Q 相电流，单位 A，例：`0.5 -> 0.5A`。

```c
motor_dq_current(PORT1, TFLOAT, id, 0.5f);

```

#### 位置控制模式

电机以最大速度和最大加速度运动到指定目标位置。

*   `pos`：目标位置，单位由 `MOTOR_DATA_TYPE_FLAG` 决定（默认圈数）。

**注意**：该模式速度与力矩均为最大，运动激烈、瞬时电流可能飙升；适用于极端响应场合，一般不建议，位置控制推荐梯形控制（`motor_pos_velmax_acc`）。

```c
motor_pos(PORT1, TFLOAT, id, pos);

```

#### 速度控制模式

电机以最大加速度加速到指定目标速度。

*   `vel`：目标速度，单位由 `MOTOR_DATA_TYPE_FLAG` 决定。

```c
motor_vel(PORT1, TFLOAT, id, vel);

```

#### 力矩控制模式

电机按设定目标力矩转动。

*   `tqe`：目标力矩，单位 N·m。

**注意**：力矩过小电机可能无法转动。

```c
motor_tqe(PORT1, TFLOAT, id, tqe);

```

#### 速度 + 加速度控制模式

电机以指定加速度加速到目标速度。

*   `vel`：目标速度；`acc`：加速度（单位同随 `MOTOR_DATA_TYPE_FLAG`）。

```c
motor_vel_acc(PORT1, TFLOAT, id, vel, acc);

```

#### 位置 + 速度控制模式

电机以目标速度运动至目标位置，不限制加速度和最大输出力矩。

*   `pos`：目标位置；`vel`：目标速度（单位随 `MOTOR_DATA_TYPE_FLAG`）。

```c
motor_pos_vel(PORT1, TFLOAT, id, pos, vel);

```

#### 位置 + 速度 + 最大力矩控制模式

电机以目标速度运动至指定目标位置，同时限制最大输出力矩。

*   `pos` / `vel`：目标位置/速度；`tqe`：最大允许输出力矩（N·m）。

**注意**：最大力矩过小电机可能无法达到目标速度。

```c
motor_pos_vel_MAXtqe(PORT1, TFLOAT, id, pos, vel, tqe);

```

#### 位置 + 速度 + 加速度控制模式（梯形控制）

电机按恒定加速度运动，实现「先加速 → 匀速 → 减速」的梯形速度控制。

*   `pos` / `vel` / `acc`：目标位置/速度/加速度。

**注意**：一般位置控制推荐使用该模式，运动平稳。

```c
motor_pos_velmax_acc(PORT1, TFLOAT, id, pos, vel, acc);

```

#### MIT 模式（位置 + 速度 + 力矩 + Kp/Kd）

电机输出力矩计算公式：

```text
输出力矩 =（目标位置 - 当前位置）× kp +（目标速度 - 当前速度）× kd + 前馈力矩 tqe

```

模式用法：

*   `kp=0, kd=0`：给定 `tqe` 即可实现恒定力矩输出；

*   `kp=0, kd≠0`：给定 `vel` 可实现匀速转动（存在速度静差，`kd` 不宜过大，过大会震荡）；

*   `kp≠0, kd≠0`：`pos` 恒定且 `vel=0` 时为定点控制；`pos` 可导且 `vel` 为其导数时可实现位置/速度跟踪。

参数：`pos`（位置）、`vel`（速度）、`tqe`（前馈力矩 N·m）、`kp` / `kd`（控制增益）。

**注意**：需设置合适的 `kp` / `kd`，否则控制效果差。

```c
motor_pos_vel_tqe_kp_kd(PORT1, TFLOAT, id, pos, vel, tqe, kp, kd);

```

### 9.2 一拖多控制（`motor_many`）

`motor_many` 是一拖多批量控制封装，内部持有 `many_data_s` 打包缓冲区。控制函数需指定 `portx` 与电机 `id`；写入缓冲后统一 `motor_many_send()` 发送。

**注意：**

*   一拖多固定 `TINT16` 数据类型（无需指定 `type`），每电机数据长度由模式决定（2/4/6/10 字节），单帧最多切 60 字节数据，超出自动切分多帧、模式块编号递增；

*   停止 / 刹车 / 软重启 / 重置零位为每电机 1 字节 `enable`（非 0 启用）；

*   发送帧 CAN ID = `ID_PREFIX_TINT16 | 模式块编号`（`fdcan_send` 自动置 bit\[15\]=1）；

*   `motor_many_send(portx, request_type)` 的第二参数为查询码，决定返回帧内容（0x0B 标准状态 / 0x0C 含温度 / 0x0E 无模式）。

#### 位置控制

```c
motor_many_pos(PORT1, 1, pos1);
motor_many_pos(PORT1, 2, pos2);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### 速度控制

```c
motor_many_vel(PORT1, id, vel);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### 力矩控制

```c
motor_many_tqe(PORT1, id, tqe);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### DQ 电压 / 电流

```c
motor_many_dq_volt(PORT1, id, vol);      // DQ 电压
motor_many_dq_current(PORT1, id, cur);   // DQ 电流
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### 速度 + 加速度

```c
motor_many_vel_acc(PORT1, id, vel, acc);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### 位置 + 速度 + 最大力矩

```c
motor_many_pos_vel_MAXtqe(PORT1, id, pos, vel, tqe);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### 位置 + 速度 + 加速度（梯形控制）

```c
motor_many_pos_vel_acc(PORT1, id, pos, vel, acc);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### MIT 运控

```c
motor_many_pos_vel_tqe_kp_kd(PORT1, id, pos, vel, tqe, kp, kd);
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

#### 停止 / 刹车 / 软重启 / 重置零位（每电机 1 字节 enable）

```c
motor_many_stop(PORT1, id, 1);    // 停止
motor_many_brake(PORT1, id, 1);   // 刹车
motor_many_reset(PORT1, id, 1);   // 软重启
motor_many_rezero(PORT1, id, 1);  // 重置零位
motor_many_send(PORT1, MODE_FLAUT_POS_VEL_TQE);

```

### 9.3 状态读取与系统函数

#### 停止 / 刹车

```c
motor_stop(PORT1, TFLOAT, id);    // 三相悬空，可自由转动
motor_brake(PORT1, TFLOAT, id);   // 三相接地，阻尼刹车（有外力仍可缓慢转动）

```

#### 主动读取状态（不控制时）

```c
motor_request_state(PORT1, TFLOAT, id);   // 发送查询指令（查询码 0x0B），返回帧自动解析

```

#### 读取最新状态

```c
p_motor_state_s state = motor_get_state(PORT1, id);
printf("ID:%2d, mode:%2X, fault:%2X, pos: %8.4f, vel: %8.4f, tor: %8.4f\r\n",
       id, state->mode, state->fault, state->position, state->velocity, state->torque);

```

*   `motor_get_state()` 返回 `p_motor_state_s`（`motor_state_s*`），包含 `mode` / `fault` / `position` / `velocity` / `torque` / `temp` / `version` / `hw_version` / `model` 等字段；

*   状态解析由主循环 `motor_process_state_all()` 自动完成；读取前须保证 `id ∈ [1, MOTOR_MAX_NUM]`（越界有 `MOTOR_ERR` 保护）。

#### 读取固件版本 / 硬件版本 / 型号

```c
motor_request_fw_version(PORT1, id);            // 固件版本 → state.version (major.minor.patch)
motor_request_hw_version(PORT1, id);   // 硬件版本 → state.hw_version
motor_request_model(PORT1, id);              // 型号 → state.model (ASCII)

```

### 9.4 配置类函数（`motor_config`）

配置类函数用于修改电机参数（零位 / ID / 周期返回），底层采用**闭环确认**机制（`motor_config_closed_loop`：清零 `ack` → 循环重发 → 等待 `ack` 非 0 或超时），函数返回值 `0` 表示成功、`1` 表示超时失败。

#### 重置零位（校准）

将当前机械位置设为电机零点，需电机处于静止状态后调用。

```c
uint8_t motor_pos_reset(port_t portx, const uint8_t id);   // 返回 0 成功，1 失败
```

*   内部流程：软重启 → 重发重设零位命令并闭环确认 → 再次软重启，全程自动完成；
*   成功后电机以当前位置为新零点，后续位置控制均以此为基准。

#### 更改电机 ID

将电机 ID 从 `old_id` 改为 `new_id`。

```c
uint8_t motor_set_id(port_t portx, const uint8_t old_id, const uint8_t new_id);   // 返回 0 成功，1 失败
```

*   底层按 `old_id` 寻址下发改 ID 命令，闭环确认后用 `new_id` 软重启；
*   **注意**：修改后请同步更新 `src/motor/motor.h` 的 `MOTOR_MAX_NUM`（及配置），否则新 ID 超出范围不会解析返回帧。

#### 周期返回状态（运行时功能）

让电机按设定周期自动返回状态帧（无需每次手动查询）。

```c
uint8_t motor_timed_return_status(port_t portx, const uint8_t id, const uint32_t t_us);   // 返回 0 成功，1 失败
```

*   `t_us`：返回周期，单位 1us（4 字节小端），填 `0` 停止周期返回；
*   **注意**：周期小于 100us 电机会报错（系统命令返回 `03 01`）；
*   **注意**：该功能为运行时设置，末尾**不做软重启**（重启会清除该设置），这点与 `motor_pos_reset` / `motor_set_id` 不同。


---

## 10. 单位与数据类型

### 10.1 单位制

*   角度单位默认**圈**（`MOTOR_DATA_TYPE_FLAG = TURNS`），可在 `src/convert/convert.h` 改为弧度（`RADIAN_2PI`）或角度（`ANGLE_360`）；

*   力矩（`tqe`）固定为 N·m；温度固定为 ℃。

### 10.2 数据类型 `data_type_t`

枚举值 = CAN ID bits\[17:16\]：

| 枚举           | 说明           |
| -------------- | -------------- |
| `TINT16_NOHDR` | 无模式头 int16 |
| `TINT16`       | int16          |
| `TINT32`       | int32          |
| `TFLOAT`       | float          |

### 10.3 换算与量程（发送：物理→raw；接收：raw→物理）

| 量          | TINT16 缩放   | TINT32 缩放     | TFLOAT     | TINT16 最大表示范围 |
| ----------- | ------------- | --------------- | ---------- | ------------------- |
| 位置        | 10000/圈      | 100000/圈       | 直接 float | ±3.276 圈（±32768） |
| 速度        | 4000/(圈/秒)  | 100000/(圈/秒)  | 直接 float | ±8.19 圈/秒         |
| 力矩        | 100/N·m       | 1000/N·m        | 直接 float | ±327.6 N·m          |
| 电流 / 电压 | 10/A(V)       | 1000/A(V)       | 直接 float | ±3276 A / V         |
| 加速度      | 1000/(圈/秒²) | 100000/(圈/秒²) | 直接 float | ±32.76 圈/秒²       |
| kp / kd     | 10            | 1000            | 直接 float | ±3276               |

*   超限数据会被 `data_limit` 钳位到类型上限；NAN 用 `NAN_INT16 = 0x8000` / `NAN_INT32 = 0x80000000` 表示。

---

## 11. 故障排查

| 现象                    | 处理方向                                                     |
| ----------------------- | ------------------------------------------------------------ |
| 控制无响应              | 确认电机 ID 正确；<br>主控板是否插上type\_c                  |
| 读取位置异常 / 一直为 0 | 确认主函数是否调用 `motor_process_state_all()`；确认查询码（0x0B）返回帧正常返回； |
| 一拖多电机不响应        | 确认 `motor_many_send()` 已被调用（仅打包不发送不生效）；<br>确认电机数 ≤ `MANY_MOTOR_SIZE`； |
| 电机震荡或失控          | 运控模式确认 `kp` / `kd` 设置合理（位置控制 `kd` 不能为 0）； |
| LED 闪烁报错            | 电机数量 / CAN 通道配置问题：确认 `motor.h` 中 `MOTOR_MAX_NUM` 与 `MOTOR_PORT_NUM` 是否正确 |