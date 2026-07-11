#!/usr/bin/env python3
"""
树莓派与 STM32 串口通信的统一数据帧工具。

约定帧格式：
  $PAYLOAD*CS\r\n

字段说明：
  $       : 帧头
  PAYLOAD : ASCII 文本载荷，例如 ARM,001,SERVO,1,1500,1000
  *       : 载荷与校验码分隔符
  CS      : 两位 ASCII 十六进制校验码
  \r\n    : 帧结束符

校验规则：
  只累加 PAYLOAD 中每个 ASCII 字节，不包含 $、*、CS、\r、\n。
  checksum = sum(payload_bytes) & 0xFF
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import serial


DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1.0


@dataclass
class RobotFrame:
    """解析后的数据帧。"""

    target: str
    seq: str
    command: str
    params: list[str]
    checksum: int
    raw_payload: str


def calc_checksum(payload: str) -> int:
    """
    计算载荷的 8 位累加和校验码。

    参数：
      payload: 不包含 $、*、校验码、换行符的载荷字符串

    返回：
      0~255 之间的整数，例如 0x3A。
    """

    total = 0
    for byte in payload.encode("ascii"):
        total = (total + byte) & 0xFF
    return total


def checksum_to_text(checksum: int) -> str:
    """
    把 8 位校验码转换成两位大写十六进制字符串。

    例如：
      0x3A -> "3A"
      0x05 -> "05"
    """

    return f"{checksum & 0xFF:02X}"


def text_to_checksum(text: str) -> int:
    """
    把两位 ASCII 十六进制校验码转换成整数。

    例如：
      "3A" -> 0x3A
      "05" -> 0x05
    """

    if len(text) != 2:
        raise ValueError("校验码长度必须是 2 个字符")
    return int(text, 16)


def build_payload(target: str, seq: int | str, command: str, params: Iterable[object] = ()) -> str:
    """
    生成数据帧载荷。

    参数：
      target : 目标模块，当前使用 ARM 或 BASE
      seq    : 命令序号，用于匹配发送帧和返回帧
      command: 命令字，例如 STOP、HOME、SERVO、MOVE
      params : 命令参数，会转成字符串后用逗号拼接
    """

    if isinstance(seq, int):
        seq_text = f"{seq:03d}"
    else:
        seq_text = str(seq)

    fields = [target.upper(), seq_text, command.upper()]
    fields.extend(str(param) for param in params)
    return ",".join(fields)


def build_frame(target: str, seq: int | str, command: str, params: Iterable[object] = ()) -> str:
    """
    按约定格式生成完整发送帧。

    返回值包含帧头、校验码和 \r\n，可直接编码后写入串口。
    """

    payload = build_payload(target, seq, command, params)
    checksum = checksum_to_text(calc_checksum(payload))
    return f"${payload}*{checksum}\r\n"


def parse_frame(line: str) -> RobotFrame:
    """
    解析并校验一行返回帧。

    参数：
      line: 串口读到的一行文本，可以包含 \r\n

    返回：
      RobotFrame 对象

    异常：
      ValueError 表示帧头、分隔符、字段数量或校验码错误。
    """

    text = line.strip()
    if not text.startswith("$"):
        raise ValueError("缺少帧头 $")
    if "*" not in text:
        raise ValueError("缺少校验分隔符 *")

    payload, checksum_text = text[1:].split("*", 1)
    received_checksum = text_to_checksum(checksum_text)
    calculated_checksum = calc_checksum(payload)
    if received_checksum != calculated_checksum:
        raise ValueError(
            f"校验失败: received=0x{received_checksum:02X}, calculated=0x{calculated_checksum:02X}"
        )

    fields = payload.split(",")
    if len(fields) < 3:
        raise ValueError("载荷字段数量不足")

    return RobotFrame(
        target=fields[0],
        seq=fields[1],
        command=fields[2],
        params=fields[3:],
        checksum=received_checksum,
        raw_payload=payload,
    )


def open_serial(port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = DEFAULT_TIMEOUT) -> serial.Serial:
    """
    打开树莓派上的 USB-TTL 串口。

    树莓派看到的是 /dev/ttyUSB0、/dev/ttyUSB1 这类 Linux 设备名；
    STM32 侧对应的是 USART1 RX/TX。
    """

    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
    )


def send_frame(ser: serial.Serial, frame: str) -> None:
    """把完整数据帧写入串口。"""

    ser.write(frame.encode("ascii"))
    ser.flush()


def read_line(ser: serial.Serial) -> str:
    """从串口读取一行 ASCII 文本。"""

    return ser.readline().decode("ascii", errors="replace").strip()
