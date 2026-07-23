#!/usr/bin/env python3
"""
高擎电机 FDCAN 协议解析与报文生成工具

基于 README 协议手册，支持：
  1. 解析电机返回报文 (CAN 帧 → 结构化物理量)
  2. 根据 ID / 模式 / 参数 生成控制报文 (物理量 → CAN 帧)

用法：
  python motor_protocol.py parse  <hex_data> <data_type> [--motor-id N]
  python motor_protocol.py gen    <mode> <data_type> <motor_id> [params...]
  python motor_protocol.py interactive
"""

import struct
import sys
import math
import argparse
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Tuple, Union
from enum import IntEnum


# ============================================================
#  常量定义
# ============================================================

class DataType(IntEnum):
    """数据类型 (CAN ID bits[17:16])"""
    TINT16 = 0
    TINT32 = 1
    TFLOAT = 2

    @property
    def id_title(self) -> int:
        return (self.value + 1) << 16  # 0x10000, 0x20000, 0x30000

    @property
    def label(self) -> str:
        return ["TINT16", "TINT32", "TFLOAT"][self.value]


class Mode(IntEnum):
    """电机控制模式码"""
    STOP          = 0x01
    BRAKE         = 0x18
    VOLT          = 0x19   # d=0, q=实际电压
    CUR           = 0x1A   # d=0, q=实际电流
    TQE           = 0x1B
    VEL           = 0x1C
    POS           = 0x1D
    VEL_ACC       = 0x1E
    POS_VEL_TQE   = 0x1F
    POS_VEL_ACC   = 0x20
    MIT           = 0x21


class SysCmd(IntEnum):
    """系统命令 cmd 前缀"""
    READ_STATE      = 0x00  # cmd[1]=0x0B
    READ_VERSION    = 0x00  # cmd[1]=0x04
    READ_HARDWARE   = 0x00  # cmd[1]=0x05
    READ_MODEL      = 0x00  # cmd[1]=0x07
    SOFT_RESET      = 0x03  # cmd={0x03, 0x03, 0x01}
    SAVE_CONFIG     = 0x03  # cmd={0x03, 0x03, 0x02}
    SET_ZERO        = 0x03  # cmd={0x03, 0x03, 0x03}
    CHANGE_ID       = 0x03  # cmd={0x03, 0x03, 0x04, new_id}
    SET_TIMEOUT     = 0x05  # cmd={0x05, 0x1F, t_lo, t_hi}


# 查询码 (cmd[1])
QUERY_STANDARD = 0x0B       # 返回模式/错误/位置/速度/力矩
QUERY_TEMP     = 0x0C       # 返回温度+模式+错误+位置+速度+力矩
QUERY_VERSION  = 0x04
QUERY_HARDWARE = 0x05
QUERY_MODEL    = 0x07


# ============================================================
#  缩放系数 (物理值 <-> raw 值)
# ============================================================

# 发送方向: raw = 物理值 × 系数
SCALE_SEND: Dict[DataType, Dict[str, float]] = {
    DataType.TINT16: {"pos": 10000, "vel": 4000, "tqe": 100,
                      "cur": 10, "vol": 10, "acc": 1000, "pid": 100},
    DataType.TINT32: {"pos": 100000, "vel": 100000, "tqe": 1000,
                      "cur": 1000, "vol": 1000, "acc": 100000, "pid": 1000},
    DataType.TFLOAT: {"pos": 1, "vel": 1, "tqe": 1,
                      "cur": 1, "vol": 1, "acc": 1, "pid": 1},
}

# 接收方向: 物理值 = raw / 系数  (与发送对称)
SCALE_RECV = SCALE_SEND  # 对称


# ============================================================
#  帧长表: (TINT16 帧长, TFLOAT/TINT32 帧长)
# ============================================================

FRAME_LEN: Dict[Mode, Tuple[int, int]] = {
    Mode.STOP:          (2, 2),
    Mode.BRAKE:         (2, 2),
    Mode.VOLT:          (6, 10),   # d(2/4) + q(2/4)
    Mode.CUR:           (6, 10),   # d(2/4) + q(2/4)
    Mode.TQE:           (4, 6),
    Mode.VEL:           (4, 6),
    Mode.POS:           (4, 6),
    Mode.VEL_ACC:       (6, 10),
    Mode.POS_VEL_TQE:   (8, 14),
    Mode.POS_VEL_ACC:   (8, 14),
    Mode.MIT:           (12, 22),
}

# 每值字节数
VAL_BYTES: Dict[DataType, int] = {DataType.TINT16: 2, DataType.TINT32: 4, DataType.TFLOAT: 4}


# ============================================================
#  一拖多模式定义 (固定 TINT16)
# ============================================================

@dataclass
class ManyMode:
    name: str
    label: str
    can_id_base: int      # 起始 CAN ID (如 0x10080)
    bytes_per_motor: int  # 每电机字节数
    motors_per_frame: int # 每帧最多电机数
    param_names: List[str]
    scales: List[str]     # scale field names for phys_to_raw

MANY_MODES = {
    "pos":         ManyMode("pos",         "位置",           0x10080, 2, 30, ["位置(圈)"],                       ["pos"]),
    "vel":         ManyMode("vel",         "速度",           0x10081, 2, 30, ["速度(圈/s)"],                    ["vel"]),
    "tqe":         ManyMode("tqe",         "力矩",           0x10082, 2, 30, ["力矩(Nm)"],                       ["tqe"]),
    "volt":        ManyMode("volt",        "电压",           0x10083, 2, 30, ["电压(V)"],                        ["vol"]),
    "cur":         ManyMode("cur",         "电流",           0x10084, 2, 30, ["电流(A)"],                        ["cur"]),
    "timeout":     ManyMode("timeout",     "超时",           0x10085, 2, 30, ["超时(ms)"],                       ["pid"]),
    "vel_acc":     ManyMode("vel_acc",     "速度+加速度",     0x10090, 4, 10, ["速度(圈/s)","加速度(圈/s²)"],      ["vel","acc"]),
    "pos_vel_tqe": ManyMode("pos_vel_tqe", "位置+速度+力矩",   0x10092, 6, 10, ["位置(圈)","速度(圈/s)","力矩(Nm)"], ["pos","vel","tqe"]),
    "pos_vel_acc": ManyMode("pos_vel_acc", "位置+速度+加速度", 0x10095, 6, 10, ["位置(圈)","速度(圈/s)","加速度(圈/s²)"],["pos","vel","acc"]),
    "mit":         ManyMode("mit",         "MIT运控",        0x18098, 10, 6, ["位置","速度","力矩","Kp","Kd"],     ["pos","vel","tqe","pid","pid"]),
}


# ============================================================
#  数据结构
# ============================================================

# 模式码 → 名称映射
MODE_NAMES: Dict[int, str] = {
    0x00: "查询/系统命令",
    0x01: "停止 (STOP)",
    0x18: "刹车 (BRAKE)",
    0x19: "DQ电压 (VOLT)",
    0x1A: "DQ电流 (CUR)",
    0x1B: "力矩 (TQE)",
    0x1C: "速度 (VEL)",
    0x1D: "位置 (POS)",
    0x1E: "速度+加速度 (VEL_ACC)",
    0x1F: "位置+速度+力矩 (POS_VEL_TQE)",
    0x20: "位置+速度+加速度 (POS_VEL_ACC)",
    0x21: "MIT运控",
}

# 错误码 → 名称映射
FAULT_NAMES: Dict[int, str] = {
    0x01: "过温",
    0x02: "过流",
    0x04: "过压",
    0x08: "欠压",
    0x10: "编码器异常",
    0x20: "缺相",
    0x40: "堵转/过载",
    0x80: "通信超时",
}


def _fault_desc(fault: int) -> str:
    """解析错误码为描述字符串"""
    if fault == 0:
        return "无"
    names = []
    for bit, name in FAULT_NAMES.items():
        if fault & bit:
            names.append(name)
    return ", ".join(names)


@dataclass
class MotorState:
    """电机状态 (解析响应报文)"""
    query:      int = 0
    mode:       int = 0
    fault:      int = 0
    temp:       float = 0.0     # °C
    position:   float = 0.0     # 圈
    velocity:   float = 0.0     # 圈/s
    torque:     float = 0.0     # Nm
    data_type:  DataType = DataType.TINT16
    motor_id:   int = 0

    # 版本号
    version:    Optional[str] = None  # "major.minor.patch"

    # 型号
    model:      Optional[str] = None  # "5036_02"

    def __repr__(self):
        lines = [f"MotorState (ID={self.motor_id}, {self.data_type.label})"]
        if self.version:
            lines.append(f"  version = {self.version}")
        if self.model:
            lines.append(f"  model   = {self.model}")
        else:
            mode_name = MODE_NAMES.get(self.mode, "未知")
            lines.append(f"  query  = {self.query} (0x{self.query:02X})")
            lines.append(f"  mode   = {self.mode} (0x{self.mode:02X}) → {mode_name}")
            lines.append(f"  fault  = {self.fault} (0x{self.fault:02X}) → {_fault_desc(self.fault)}")
            lines.append(f"  temp   = {self.temp:.1f} °C")
            lines.append(f"  pos    = {self.position:.4f} 圈")
            lines.append(f"  vel    = {self.velocity:.4f} 圈/s")
            lines.append(f"  tqe    = {self.torque:.4f} Nm")
        return "\n".join(lines)


# ============================================================
#  CAN ID 编解码
# ============================================================

def encode_can_id(data_type: DataType, motor_id: int, is_mit: bool = False) -> int:
    """构造发送 CAN ID: id_title | motor_id"""
    base = data_type.id_title
    if is_mit and data_type == DataType.TINT16:
        base = 0x18000
    return base | (motor_id & 0x7F)


def decode_can_id(can_id: int) -> Dict[str, Any]:
    """解码 CAN ID, 返回 {type, motor_id, is_mit} 等信息"""
    motor_id_send = can_id & 0x7F
    motor_id_recv = (can_id >> 8) & 0x7F
    bits_type = (can_id >> 16) & 3
    is_mit = bool((can_id >> 15) & 1)

    data_type = DataType(bits_type - 1) if bits_type in (1, 2, 3) else DataType.TINT16

    return {
        "data_type": data_type,
        "motor_id_send": motor_id_send,
        "motor_id_recv": motor_id_recv,
        "is_mit": is_mit,
    }


# ============================================================
#  物理量 ⇄ raw 值 转换
# ============================================================

# 物理量单位名称
FIELD_UNITS: Dict[str, str] = {
    "pos": "圈", "vel": "圈/s", "tqe": "Nm",
    "cur": "A", "vol": "V", "acc": "圈/s²", "pid": "",
}

# 数据类型范围 (signed int 的 min/max)
DTYPE_RANGE: Dict[DataType, Tuple[int, int]] = {
    DataType.TINT16: (-32768, 32767),
    DataType.TINT32: (-2147483648, 2147483647),
    DataType.TFLOAT: (0, 0),  # 不限制
}


def _check_range(raw: int, data_type: DataType, field: str, value: float):
    """检查 raw 值是否超出数据类型范围, 超出则抛出友好错误"""
    if data_type == DataType.TFLOAT:
        return
    lo, hi = DTYPE_RANGE[data_type]
    if lo <= raw <= hi:
        return
    scale = SCALE_SEND[data_type][field]
    unit = FIELD_UNITS.get(field, "")
    lo_phys = lo / scale
    hi_phys = hi / scale
    raise ValueError(
        f"数值 {value}{unit} 超出 {data_type.label} 范围 "
        f"[{lo_phys:.4f}{unit} ~ {hi_phys:.4f}{unit}], "
        f"raw={raw}, 上限={hi}"
    )


def phys_to_raw(value: float, data_type: DataType, field: str) -> int:
    """物理值 → raw 整数 (发送用), NaN 自动转为对应类型的无效值"""
    if math.isnan(value):
        if data_type == DataType.TFLOAT:
            return 0  # 由 pack_value 打包为 NaN float
        elif data_type == DataType.TINT32:
            return -2147483648   # NAN_INT32 = 0x80000000
        else:
            return -32768         # NAN_INT16 = 0x8000
    scale = SCALE_SEND[data_type][field]
    raw = round(value * scale)
    _check_range(raw, data_type, field, value)
    return raw


def raw_to_phys(raw: int, data_type: DataType, field: str) -> float:
    """raw 整数 → 物理值 (接收用)"""
    scale = SCALE_RECV[data_type][field]
    return raw / scale


def pack_value(raw: int, data_type: DataType) -> bytes:
    """将 raw 值打包为小端字节, TFLOAT 的 NaN 打包为 IEEE 754 NaN"""
    if data_type == DataType.TFLOAT:
        val = float(raw)
        if math.isnan(val):
            val = float('nan')
        return struct.pack("<f", val)
    elif data_type == DataType.TINT32:
        return struct.pack("<i", raw)
    else:
        return struct.pack("<h", raw)


def unpack_value(data: bytes, offset: int, data_type: DataType) -> int:
    """从小端字节解析 raw 值"""
    if data_type == DataType.TFLOAT:
        raw = struct.unpack_from("<f", data, offset)[0]
        return int(raw)  # TFLOAT 物理值等同 raw
    elif data_type == DataType.TINT32:
        return struct.unpack_from("<i", data, offset)[0]
    else:
        return struct.unpack_from("<h", data, offset)[0]


# ============================================================
#  报文生成
# ============================================================

def build_cmd(mode: Mode, data_type: DataType, params: List[float]) -> Tuple[int, bytes]:
    """
    生成控制命令报文
    参数:
      mode      - 控制模式 (Mode 枚举)
      data_type - 数据类型
      params    - 物理值列表, 顺序取决于模式
                  VOLT:         [q_volt]           (d 自动填 0)
                  CUR:          [q_cur]            (d 自动填 0)
                  TQE/VEL/POS:  [value]
                  VEL_ACC:      [vel, acc]
                  POS_VEL_TQE:  [pos, vel, tqe]
                  POS_VEL_ACC:  [pos, vel_max, acc]
                  MIT:          [pos, vel, tqe, kp, kd]
    返回: (CAN ID 不含 motor_id, cmd bytes)
    """
    if mode == Mode.VOLT:
        # d=0, q=实际电压
        assert len(params) == 1, "VOLT 需要 1 个参数: [q_volt]"
        raw_d = 0
        raw_q = phys_to_raw(params[0], data_type, "vol")
        raw_params = [raw_d, raw_q]

    elif mode == Mode.CUR:
        # d=0, q=实际电流
        assert len(params) == 1, "CUR 需要 1 个参数: [q_cur]"
        raw_d = 0
        raw_q = phys_to_raw(params[0], data_type, "cur")
        raw_params = [raw_d, raw_q]

    elif mode == Mode.TQE:
        assert len(params) == 1, "TQE 需要 1 个参数: [tqe]"
        raw_params = [phys_to_raw(params[0], data_type, "tqe")]

    elif mode == Mode.VEL:
        assert len(params) == 1, "VEL 需要 1 个参数: [vel]"
        raw_params = [phys_to_raw(params[0], data_type, "vel")]

    elif mode == Mode.POS:
        assert len(params) == 1, "POS 需要 1 个参数: [pos]"
        raw_params = [phys_to_raw(params[0], data_type, "pos")]

    elif mode == Mode.VEL_ACC:
        assert len(params) == 2, "VEL_ACC 需要 2 个参数: [vel, acc]"
        raw_params = [phys_to_raw(params[0], data_type, "vel"),
                      phys_to_raw(params[1], data_type, "acc")]

    elif mode == Mode.POS_VEL_TQE:
        assert len(params) == 3, "POS_VEL_TQE 需要 3 个参数: [pos, vel, tqe]"
        raw_params = [phys_to_raw(params[0], data_type, "pos"),
                      phys_to_raw(params[1], data_type, "vel"),
                      phys_to_raw(params[2], data_type, "tqe")]

    elif mode == Mode.POS_VEL_ACC:
        assert len(params) == 3, "POS_VEL_ACC 需要 3 个参数: [pos, vel_max, acc]"
        raw_params = [phys_to_raw(params[0], data_type, "pos"),
                      phys_to_raw(params[1], data_type, "vel"),
                      phys_to_raw(params[2], data_type, "acc")]

    elif mode == Mode.MIT:
        assert len(params) == 5, "MIT 需要 5 个参数: [pos, vel, tqe, kp, kd]"
        raw_params = [phys_to_raw(params[0], data_type, "pos"),
                      phys_to_raw(params[1], data_type, "vel"),
                      phys_to_raw(params[2], data_type, "tqe"),
                      phys_to_raw(params[3], data_type, "pid"),
                      phys_to_raw(params[4], data_type, "pid")]

    elif mode in (Mode.STOP, Mode.BRAKE):
        raw_params = []

    else:
        raise ValueError(f"不支持的模式: {hex(mode)}")

    # 构造 cmd 字节
    cmd = bytearray()
    cmd.append(mode.value)
    cmd.append(QUERY_STANDARD)
    for raw in raw_params:
        cmd.extend(pack_value(raw, data_type))

    can_id_base = data_type.id_title
    return can_id_base, bytes(cmd)


def build_sys_cmd(cmd_type: str, motor_id: int = 1, **kwargs) -> Tuple[int, bytes]:
    """生成系统命令报文"""
    can_id = DataType.TINT16.id_title | (motor_id & 0x7F)

    if cmd_type == "read_state":
        return can_id, bytes([0x00, 0x0B])
    elif cmd_type == "read_version":
        return can_id, bytes([0x00, 0x04])
    elif cmd_type == "read_hardware":
        return can_id, bytes([0x00, 0x05])
    elif cmd_type == "read_model":
        return can_id, bytes([0x00, 0x07])
    elif cmd_type == "stop":
        return can_id, bytes([0x01, 0x0B])
    elif cmd_type == "brake":
        return can_id, bytes([0x18, 0x0B])
    elif cmd_type == "soft_reset":
        return can_id, bytes([0x03, 0x03, 0x01])
    elif cmd_type == "save_config":
        return can_id, bytes([0x03, 0x03, 0x02])
    elif cmd_type == "set_zero":
        return can_id, bytes([0x03, 0x03, 0x03])
    elif cmd_type == "change_id":
        new_id = kwargs.get("new_id", 1)
        return can_id, bytes([0x03, 0x03, 0x04, new_id & 0x7F])
    elif cmd_type == "set_timeout":
        t_ms = kwargs.get("timeout_ms", 100)
        return can_id, bytes([0x05, 0x1F, t_ms & 0xFF, (t_ms >> 8) & 0xFF])
    else:
        raise ValueError(f"不支持的系统命令: {cmd_type}")


# ============================================================
#  一拖多报文生成 (固定 TINT16)
# ============================================================

def build_many_cmd(mode_name: str, motors: List[Tuple[int, List[float]]]) -> List[Tuple[int, bytes]]:
    """
    生成一拖多控制报文
    参数:
      mode_name - 一拖多模式名 (MANY_MODES 的 key)
      motors    - [(motor_id, [param_values]), ...]
    返回:
      [(can_id, cmd_bytes), ...]  多个帧 (多帧模式可能拆分)
    """
    if mode_name not in MANY_MODES:
        raise ValueError(f"不支持的一拖多模式: {mode_name}, 可选: {list(MANY_MODES.keys())}")

    mm = MANY_MODES[mode_name]

    if len(motors) == 0:
        raise ValueError("至少需要一个电机")

    # 验证参数个数
    for motor_id, params in motors:
        if len(params) != len(mm.param_names):
            raise ValueError(
                f"电机{motor_id}: 需要 {len(mm.param_names)} 个参数 ({', '.join(mm.param_names)}), "
                f"提供了 {len(params)} 个"
            )

    # 打包所有电机数据为 raw 字节
    data_type = DataType.TINT16  # 一拖多固定 TINT16
    all_data = bytearray()
    for motor_id, params in motors:
        for i, (val, scale_field) in enumerate(zip(params, mm.scales)):
            try:
                raw = phys_to_raw(val, data_type, scale_field)
                all_data.extend(struct.pack("<h", raw))
            except ValueError as e:
                raise ValueError(f"电机{motor_id} 参数{i+1}({mm.param_names[i]}): {e}") from e

    # 按帧拆分 (每帧最多 motors_per_frame 个电机)
    frames = []
    offset = 0
    can_id = mm.can_id_base
    bytes_per_frame = mm.bytes_per_motor * mm.motors_per_frame

    while offset < len(all_data):
        chunk = all_data[offset:offset + bytes_per_frame]
        frames.append((can_id, bytes(chunk)))
        offset += bytes_per_frame
        can_id += 1

    return frames


# ============================================================
#  报文解析
# ============================================================

def parse_response(hex_data: str, data_type: DataType = DataType.TINT16) -> MotorState:
    """
    解析电机响应报文
    参数:
      hex_data  - 十六进制字符串, 如 "0B1C00C80090019003" (含空格亦可)
      data_type - 数据类型, 默认 TINT16
    """
    hex_data = hex_data.replace(" ", "").replace("\n", "")
    data = bytes.fromhex(hex_data)

    state = MotorState(data_type=data_type)

    byte0 = data[0]

    # --- 型号查询响应 (0x07) ---
    if byte0 == 0x07 and len(data) >= 3:
        model_len = data[1]
        if model_len > 0 and len(data) >= model_len + 2:
            chars = []
            for i in range(model_len):
                nibble = data[2 + i] & 0x0F
                chars.append(f"{nibble:X}")
            model_str = "".join(chars)
            if len(model_str) > 5:
                state.model = f"{model_str[:4]}_{model_str[5:]}"
            else:
                state.model = model_str
            return state

    # --- 版本查询响应 (0xB5 0x02) ---
    if len(data) >= 2 and data[1] == 0xB5 and data[2] == 0x02:
        if len(data) >= 5:
            major  = data[4] >> 4
            minor  = (data[4] & 0x0F) | (data[3] >> 4)
            patch  = data[3] & 0x0F
            state.version = f"{major}.{minor}.{patch}"
        return state

    # --- 普通响应: query 0x0B 或 0x0C ---
    state.query = byte0
    offset = 1

    state.mode  = data[offset]; offset += 1
    state.fault = data[offset]; offset += 1

    if byte0 == 0x0C:
        # 含温度
        temp_raw = unpack_value(data, offset, data_type); offset += VAL_BYTES[data_type]
        state.temp = raw_to_phys(temp_raw, data_type, "cur")  # 温度用 cur 的系数 (0.1°C/LSB for TINT16)
        # 对于 TINT16, temp_raw / 10 = °C
        if data_type == DataType.TINT16:
            state.temp = temp_raw / 10.0
        elif data_type == DataType.TINT32:
            state.temp = temp_raw / 1000.0
        else:
            state.temp = float(temp_raw)

    pos_raw = unpack_value(data, offset, data_type); offset += VAL_BYTES[data_type]
    vel_raw = unpack_value(data, offset, data_type); offset += VAL_BYTES[data_type]
    tqe_raw = unpack_value(data, offset, data_type)

    state.position = raw_to_phys(pos_raw, data_type, "pos")
    state.velocity = raw_to_phys(vel_raw, data_type, "vel")
    state.torque   = raw_to_phys(tqe_raw, data_type, "tqe")

    return state


# ============================================================
#  菜单项定义
# ============================================================

@dataclass
class MenuItem:
    index: int
    label: str
    param_names: List[str]       # 参数名列表
    handler: str                  # "mode" / "sys"
    handler_arg: Any              # Mode 枚举 或 系统命令字符串
    note: str = ""                # 额外说明

MENU_ITEMS: List[MenuItem] = [
    # -- 普通模式 (0-12) --
    MenuItem(0,  "无操作",              [],          "sys",  None,              ""),
    MenuItem(1,  "查询状态",            [],          "sys",  "read_state",      ""),
    MenuItem(2,  "停止",                [],          "sys",  "stop",            ""),
    MenuItem(3,  "刹车",                [],          "sys",  "brake",           ""),
    MenuItem(4,  "DQ电压控制",          ["电压(V)"], "mode", Mode.VOLT,         "(d轴固定=0)"),
    MenuItem(5,  "DQ电流控制",          ["电流(A)"], "mode", Mode.CUR,          "(d轴固定=0)"),
    MenuItem(6,  "位置控制",            ["位置(圈)"],"mode", Mode.POS,          ""),
    MenuItem(7,  "速度控制",            ["速度(圈/s)"],"mode",Mode.VEL,         ""),
    MenuItem(8,  "力矩控制",            ["力矩(Nm)"], "mode", Mode.TQE,         ""),
    MenuItem(9,  "位置+速度",           ["位置(圈)","速度(圈/s)"],
                                           "mode", Mode.POS_VEL_TQE,  "(力矩不限制)"),
    MenuItem(10, "位置+速度+力矩",       ["位置(圈)","速度(圈/s)","力矩(Nm)"],
                                           "mode", Mode.POS_VEL_TQE,  ""),
    MenuItem(11, "位置+速度+加速度",     ["位置(圈)","速度(圈/s)","加速度(圈/s²)"],
                                           "mode", Mode.POS_VEL_ACC,  ""),
    MenuItem(12, "位置+速度+力矩+KpKd",  ["位置(圈)","速度(圈/s)","力矩(Nm)","Kp刚度","Kd阻尼"],
                                           "mode", Mode.MIT,          "(MIT运控模式)"),
    # -- 一拖多模式 (TINT16固定) (13-22) --
    MenuItem(13, "一拖多-位置",          ["位置(圈)..."],       "many", "pos",         "多电机"),
    MenuItem(14, "一拖多-速度",          ["速度(圈/s)..."],     "many", "vel",         "多电机"),
    MenuItem(15, "一拖多-力矩",          ["力矩(Nm)..."],       "many", "tqe",         "多电机"),
    MenuItem(16, "一拖多-电压",          ["电压(V)..."],        "many", "volt",        "多电机"),
    MenuItem(17, "一拖多-电流",          ["电流(A)..."],        "many", "cur",         "多电机"),
    MenuItem(18, "一拖多-速度+加速度",    ["速度,加速度..."],     "many", "vel_acc",     "多电机"),
    MenuItem(19, "一拖多-位置+速度+力矩", ["位置,速度,力矩..."],  "many", "pos_vel_tqe", "多电机"),
    MenuItem(20, "一拖多-位置+速度+加速度",["位置,速度,加速度..."],"many", "pos_vel_acc", "多电机"),
    MenuItem(21, "一拖多-MIT运控",       ["位置,速度,力矩,Kp,Kd..."],"many","mit",      "多电机"),
]

# ============================================================
#  交互模式
# ============================================================

def _print_menu():
    """打印模式选择菜单"""
    print()
    print("  可选控制模式及所需参数:")
    for item in MENU_ITEMS:
        params = ", ".join(item.param_names) if item.param_names else "无"
        note = f" {item.note}" if item.note else ""
        print(f"  {item.index:>4}  {item.label:<22} → 参数: {params}{note}")
    print()


def _prompt_mode(default: int = 0) -> MenuItem:
    """让用户选择模式"""
    while True:
        try:
            raw = input(f"  选择控制模式 [默认:{default}]: ").strip()
            if raw == "":
                idx = default
            else:
                idx = int(raw)

            if any(item.index == idx for item in MENU_ITEMS):
                return next(item for item in MENU_ITEMS if item.index == idx)
            else:
                print(f"  无效选择: {idx}, 请输入 0~{len(MENU_ITEMS)-1}")
        except ValueError:
            print(f"  请输入数字")


def _prompt_motor_id(default: int = 1) -> int:
    """让用户输入电机 ID"""
    while True:
        try:
            raw = input(f"  电机 ID [默认:{default}]: ").strip()
            if raw == "":
                return default
            mid = int(raw)
            if 1 <= mid <= 127:
                return mid
            print("  ID 范围 1~127")
        except ValueError:
            print("  请输入数字")


def _prompt_data_type(default: str = "tint16") -> DataType:
    """让用户选择数据类型"""
    print(f"  数据类型: 1=TINT16  2=TINT32  3=TFLOAT")
    type_map = {"1": DataType.TINT16, "2": DataType.TINT32, "3": DataType.TFLOAT,
                "": DataType.TINT16}
    default_key = {"tint16": "1", "tint32": "2", "float": "3"}.get(default, "1")
    while True:
        try:
            raw = input(f"  选择数据类型 [默认:{default_key}]: ").strip()
            if raw == "":
                raw = default_key
            if raw in type_map:
                return type_map[raw]
            print("  请输入 1/2/3")
        except ValueError:
            print("  请输入数字")


def _prompt_params(item: MenuItem, data_type: DataType) -> List[float]:
    """根据菜单项提示用户输入参数"""
    if not item.param_names:
        return []
    print(f"  请输入{len(item.param_names)}个参数 ({', '.join(item.param_names)}):")
    params = []
    for i, name in enumerate(item.param_names):
        while True:
            try:
                raw = input(f"    [{i+1}/{len(item.param_names)}] {name}: ").strip()
                if raw == "":
                    print("    参数不能为空")
                    continue
                params.append(float(raw))
                break
            except ValueError:
                print("    请输入数字")
    return params


def _interactive_many(mode_name: str):
    """互动式一拖多报文生成"""
    mm = MANY_MODES[mode_name]
    print(f"\n  一拖多模式: {mm.label} (TINT16, 每帧{mm.motors_per_frame}台电机)")
    print(f"  参数: {', '.join(mm.param_names)}")

    # 询问电机数量
    while True:
        try:
            raw = input(f"  控制几台电机? [默认:1]: ").strip()
            count = 1 if raw == "" else int(raw)
            if count >= 1:
                break
            print("  至少需要 1 台")
        except ValueError:
            print("  请输入数字")

    motors = []
    for i in range(1, count + 1):
        while True:
            try:
                prompt = f"  电机[{i}]> "
                line = input(prompt).strip()
                parts = line.split()
                params = [float(x) for x in parts]

                if len(params) != len(mm.param_names):
                    print(f"  需要 {len(mm.param_names)} 个参数: {', '.join(mm.param_names)}")
                    continue

                motors.append((i, params))
                break
            except ValueError:
                print("  请输入数字")
            except (KeyboardInterrupt, EOFError):
                break

    if not motors:
        return

    print("  输入完成!\n")

    frames = build_many_cmd(mode_name, motors)
    print(f"  共 {len(motors)} 台电机, {len(frames)} 帧")
    for i, (can_id, data) in enumerate(frames):
        motor_range = f"{i * mm.motors_per_frame + 1}~{min((i+1) * mm.motors_per_frame, len(motors))}"
        print(f"  --- 帧 {i+1}: 电机 {motor_range} ---")
        print(f"  CAN ID   = 0x{can_id:05X}")
        print(f"  cmd hex  = {data.hex(' ').upper()}")
        print(f"  cmd bytes= [{', '.join(f'0x{b:02X}' for b in data)}]")
        print(f"  帧长     = {len(data)} 字节")
    print()


def _interactive_menu():
    """菜单驱动的交互式报文生成"""
    print("=" * 60)
    print("  高擎电机 FDCAN 协议工具 — 报文生成")
    print("=" * 60)
    print("  提示: 输入 'p' 可随时切换到解析模式; 'q' 退出\n")

    data_type = DataType.TINT16
    motor_id = 1

    while True:
        _print_menu()

        try:
            item = _prompt_mode(0)
        except (KeyboardInterrupt, EOFError):
            break

        if item.index == 0:
            continue

        # -- 一拖多模式 --
        if item.handler == "many":
            _interactive_many(item.handler_arg)
            input("  按 Enter 继续...")
            continue

        # -- 普通控制模式 --
        if item.handler == "mode":
            motor_id = _prompt_motor_id(motor_id)
            data_type = _prompt_data_type(
                "tint16" if data_type == DataType.TINT16 else
                "tint32" if data_type == DataType.TINT32 else "float"
            )

        # 输入参数
        if item.handler == "mode":
            params = _prompt_params(item, data_type)

            # 生成报文
            if item.handler_arg == Mode.POS_VEL_TQE and len(params) == 2:
                # 位置+速度 模式: 力矩不限制 (NaN)
                params.append(float('nan'))

            can_id_base, cmd = build_cmd(item.handler_arg, data_type, params)
            can_id = can_id_base | (motor_id & 0x7F)
            _print_result(can_id, cmd, data_type, motor_id)

        elif item.handler == "sys":
            motor_id = _prompt_motor_id(motor_id)
            can_id, cmd = build_sys_cmd(item.handler_arg, motor_id)
            _print_result(can_id, cmd, DataType.TINT16, motor_id)

        input("  按 Enter 继续...")


def _interactive_parse():
    """交互式报文解析"""
    print("=" * 60)
    print("  高擎电机 FDCAN 协议工具 — 报文解析")
    print("=" * 60)
    print("  输入十六进制电机响应数据 (可不加空格), 输入 'm' 返回菜单, 'q' 退出\n")

    type_map = {"tint16": DataType.TINT16, "tint32": DataType.TINT32, "float": DataType.TFLOAT}
    data_type = DataType.TINT16

    while True:
        try:
            line = input("  报文> ").strip()
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            if line.lower() == "m":
                return

            if line.startswith("type "):
                t = line[5:].strip().lower()
                if t in type_map:
                    data_type = type_map[t]
                    print(f"  数据类型切换为: {data_type.label}")
                else:
                    print("  可选: tint16 | tint32 | float")
                continue

            state = parse_response(line, data_type)
            print(state)
            print()

        except (KeyboardInterrupt, EOFError):
            break
        except Exception as e:
            print(f"  解析错误: {e}")

    print()


def interactive():
    """交互式命令行 (默认进入菜单模式)"""
    print("=" * 60)
    print("  高擎电机 FDCAN 协议工具")
    print("=" * 60)
    print("  命令:  Enter=生成报文  p=解析报文  h=帮助  q=退出")
    print()

    while True:
        try:
            cmd = input("> ").strip().lower()

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "p":
                _interactive_parse()
            elif cmd == "h":
                _print_help()
            elif cmd == "":
                _interactive_menu()
            else:
                # 兼容旧的 gen/parse/sys 命令
                parts = cmd.split()
                sub = parts[0]
                type_map = {"tint16": DataType.TINT16, "tint32": DataType.TINT32, "float": DataType.TFLOAT}
                mode_map = {m.name.lower(): m for m in Mode}

                if sub == "gen":
                    _handle_gen(parts[1:], mode_map, type_map)
                elif sub == "parse":
                    _handle_parse(parts[1:], type_map)
                elif sub == "sys":
                    _handle_sys(parts[1:])
                else:
                    print(f"未知命令, 输入 h 查看帮助")

        except (KeyboardInterrupt, EOFError):
            break
        except Exception as e:
            print(f"错误: {e}")

    print("退出.")


def _print_help():
    print("""
=== 生成控制报文 ===
  gen <mode> <type> <id> [params]
    mode:   volt | cur | tqe | vel | pos | vel_acc | pos_vel_tqe | pos_vel_acc | mit
    type:   tint16 | tint32 | float
    id:     电机 ID (1~127)
    params: 物理值, 空格分隔

    例:
      gen vel tint16 1 0.5              → 电机1, TINT16 速度 0.5 圈/s
      gen volt float 2 12.0             → 电机2, TFLOAT 电压 12V (d=0)
      gen cur tint16 1 2.5              → 电机1, TINT16 电流 2.5A (d=0)
      gen pos_vel_tqe tint32 3 1 0.5 2  → 电机3, TINT32 位置1圈 速度0.5圈/s 力矩2Nm
      gen mit float 1 0 0 1 10 1        → 电机1, TFLOAT MIT模式
      gen stop tint16 1                 → 电机1, 停止
      gen brake tint16 1                → 电机1, 刹车

=== 解析响应报文 ===
  parse <hex> [type]
    hex:  十六进制字符串, 如 "0B 1C 00 C8 00 90 01 03 00"
    type: tint16(默认) | tint32 | float

    例:
      parse "0B 1C 00 C8 00 90 01 03 00"
      parse "070835303033365F3032"          → 型号查询响应

=== 系统命令 ===
  sys <cmd> [id] [args]
    cmd: read_state | read_version | read_model | read_hardware |
         stop | brake | soft_reset | save_config | set_zero |
         change_id <id> <new_id> | set_timeout <id> <ms>

    例:
      sys read_version 2              → 查询电机2固件版本
      sys read_model 1                → 查询电机1型号
      sys change_id 1 3               → 电机1 改为 ID=3
      sys set_timeout 1 500           → 电机1 超时500ms
""")


def _handle_gen(args, mode_map, type_map):
    if len(args) < 3:
        print("用法: gen <mode> <type> <id> [params]")
        return

    mode_name = args[0].lower()
    type_name = args[1].lower()
    mid = int(args[2])
    params = [float(x) for x in args[3:]]

    if mode_name not in mode_map:
        print(f"未知模式: {mode_name}, 可选: {list(mode_map.keys())}")
        return
    if type_name not in type_map:
        print(f"未知类型: {type_name}, 可选: tint16 | tint32 | float")
        return

    mode = mode_map[mode_name]
    dtype = type_map[type_name]

    can_id_base, cmd = build_cmd(mode, dtype, params)
    can_id = can_id_base | (mid & 0x7F)

    _print_result(can_id, cmd, dtype, mid)


def _handle_parse(args, type_map):
    if len(args) < 1:
        print("用法: parse <hex> [type]")
        return

    hex_str = args[0]
    dtype = DataType.TINT16
    if len(args) >= 2 and args[1].lower() in type_map:
        dtype = type_map[args[1].lower()]

    state = parse_response(hex_str, dtype)
    print(state)


def _handle_sys(args):
    if len(args) < 1:
        print("用法: sys <cmd> [id] [args]")
        print(f"可选命令: read_state, read_version, read_hardware, read_model, "
              f"stop, brake, soft_reset, save_config, set_zero, change_id, set_timeout")
        return

    cmd_name = args[0]
    mid = int(args[1]) if len(args) > 1 else 1
    kwargs = {}

    if cmd_name == "change_id" and len(args) >= 3:
        kwargs["new_id"] = int(args[2])
    elif cmd_name == "set_timeout" and len(args) >= 3:
        kwargs["timeout_ms"] = int(args[2])

    can_id, cmd = build_sys_cmd(cmd_name, mid, **kwargs)
    _print_result(can_id, cmd, DataType.TINT16, mid)


def _print_result(can_id, cmd, dtype, mid):
    print(f"\nCAN ID   = 0x{can_id:05X} ({dtype.label}, 电机{mid})")
    print(f"cmd hex  = {cmd.hex(' ').upper()}")
    print(f"cmd bytes= [{', '.join(f'0x{b:02X}' for b in cmd)}]")
    print(f"帧长     = {len(cmd)} 字节\n")


# ============================================================
#  CLI 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="高擎电机 FDCAN 协议解析与报文生成工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s interactive                                  # 交互模式
  %(prog)s parse "0B 1C 00 C8 00 90 01 03 00"           # 解析响应
  %(prog)s gen vel tint16 1 0.5                          # 生成速度指令
  %(prog)s gen volt float 2 12.0                         # 生成电压指令
  %(prog)s gen cur tint32 3 2.5                          # 生成电流指令
  %(prog)s many vel 1,0.5 2,0.3 3,0.1                    # 一拖多: 3台电机速度控制
  %(prog)s many pos_vel_tqe 1,1,0.5,2 2,2,1,1            # 一拖多: 2台电机位置+速度+力矩
  %(prog)s sys read_version 1                            # 查询版本
        """
    )

    sub = parser.add_subparsers(dest="command", help="子命令")

    # ---- interactive ----
    sub.add_parser("interactive", help="交互模式")

    # ---- parse ----
    p_parse = sub.add_parser("parse", help="解析电机响应报文")
    p_parse.add_argument("hex", help="十六进制数据 (如 '0B1C00C800...')")
    p_parse.add_argument("--type", "-t", dest="dtype", default="tint16",
                         choices=["tint16", "tint32", "float"],
                         help="数据类型 (默认: tint16)")

    # ---- gen ----
    p_gen = sub.add_parser("gen", help="生成控制报文")
    p_gen.add_argument("mode", help="控制模式: volt|cur|tqe|vel|pos|vel_acc|pos_vel_tqe|pos_vel_acc|mit|stop|brake")
    p_gen.add_argument("dtype", help="数据类型: tint16|tint32|float")
    p_gen.add_argument("id", type=int, help="电机 ID (1~127)")
    p_gen.add_argument("params", nargs="*", type=float, help="物理参数 (空格分隔)")

    # ---- sys ----
    p_sys = sub.add_parser("sys", help="生成系统命令")
    p_sys.add_argument("cmd", help="系统命令")
    p_sys.add_argument("args", nargs="*", help="参数")

    # ---- many (一拖多) ----
    p_many = sub.add_parser(
        "many",
        help="生成一拖多控制报文 (TINT16)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
格式说明:
  每个电机参数用逗号分隔: 电机ID,值1,值2,...
  不同电机之间用空格分隔。

示例:
  python tools/motor_protocol.py many vel 1,0.5 2,0.3 3,0.1
    → 3台电机速度控制 (0.5 / 0.3 / 0.1 圈/s)

  python tools/motor_protocol.py many pos_vel_tqe 1,1,0.5,2 2,2,1,1
    → 2台电机: 电机1(位置1圈,速度0.5圈/s,力矩2Nm) 电机2(位置2圈,速度1圈/s,力矩1Nm)

  python tools/motor_protocol.py many mit 1,0,0,1,10,1
    → 1台电机MIT运控
"""
    )
    p_many.add_argument("mode", help="一拖多模式: " + "|".join(MANY_MODES.keys()))
    p_many.add_argument("motors", nargs="+", help="电机参数: 电机ID,值1,值2,... (电机间用空格分隔)")

    args = parser.parse_args()

    type_map = {"tint16": DataType.TINT16, "tint32": DataType.TINT32, "float": DataType.TFLOAT}
    mode_map = {m.name.lower(): m for m in Mode}

    if args.command == "interactive" or args.command is None:
        interactive()

    elif args.command == "parse":
        dtype = type_map.get(args.dtype, DataType.TINT16)
        state = parse_response(args.hex, dtype)
        print(state)

    elif args.command == "gen":
        mode_name = args.mode.lower()
        dtype = type_map.get(args.dtype, DataType.TINT16)
        mid = args.id
        params = args.params

        if mode_name in ("stop", "brake"):
            can_id, cmd = build_sys_cmd(mode_name, mid)
            print(f"CAN ID   = 0x{can_id:05X} ({dtype.label}, 电机{mid})")
            print(f"cmd hex  = {cmd.hex(' ').upper()}")
            print(f"cmd bytes= [{', '.join(f'0x{b:02X}' for b in cmd)}]")
            print(f"帧长     = {len(cmd)} 字节")
        else:
            mode = mode_map.get(mode_name)
            if not mode:
                print(f"未知模式: {mode_name}")
                sys.exit(1)
            can_id_base, cmd = build_cmd(mode, dtype, params)
            can_id = can_id_base | (mid & 0x7F)
            _print_result(can_id, cmd, dtype, mid)

    elif args.command == "sys":
        cmd_name = args.cmd
        sub_args = args.args
        mid = int(sub_args[0]) if len(sub_args) > 0 else 1
        kwargs = {}
        if cmd_name == "change_id" and len(sub_args) >= 2:
            kwargs["new_id"] = int(sub_args[1])
        elif cmd_name == "set_timeout" and len(sub_args) >= 2:
            kwargs["timeout_ms"] = int(sub_args[1])

        can_id, cmd = build_sys_cmd(cmd_name, mid, **kwargs)
        _print_result(can_id, cmd, DataType.TINT16, mid)

    elif args.command == "many":
        mode_name = args.mode.lower()
        if mode_name not in MANY_MODES:
            print(f"未知一拖多模式: {mode_name}, 可选: {list(MANY_MODES.keys())}")
            sys.exit(1)

        mm = MANY_MODES[mode_name]
        motors = []
        for entry in args.motors:
            parts = entry.split(",")
            motor_id = int(parts[0])
            params = [float(x) for x in parts[1:]]
            if len(params) != len(mm.param_names):
                print(f"电机{motor_id}: 需要 {len(mm.param_names)} 个参数 ({', '.join(mm.param_names)}), "
                      f"提供了 {len(params)} 个")
                sys.exit(1)
            motors.append((motor_id, params))

        frames = build_many_cmd(mode_name, motors)
        print(f"\n一拖多-{mm.label}: {len(motors)} 台电机, {len(frames)} 帧")
        for i, (can_id, data) in enumerate(frames):
            start_motor = i * mm.motors_per_frame + 1
            end_motor = min((i+1) * mm.motors_per_frame, len(motors))
            print(f"--- 帧 {i+1} (电机 {start_motor}~{end_motor}) ---")
            print(f"CAN ID   = 0x{can_id:05X}  (TINT16)")
            print(f"cmd hex  = {data.hex(' ').upper()}")
            print(f"cmd bytes= [{', '.join(f'0x{b:02X}' for b in data)}]")
            print(f"帧长     = {len(data)} 字节\n")


if __name__ == "__main__":
    main()
