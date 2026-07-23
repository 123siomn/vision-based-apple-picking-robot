#!/usr/bin/env python3
"""
树莓派到底盘 STM32 的统一数据帧诊断工具。

注意：
  本脚本发送的是新的 $PAYLOAD*CS 帧格式。
  当前底盘 STM32 代码如果还没有升级新协议，就不会执行这些帧。
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

from robot_protocol import DEFAULT_BAUDRATE
from robot_protocol import DEFAULT_TIMEOUT
from robot_protocol import build_frame
from robot_protocol import open_serial
from robot_protocol import parse_frame
from robot_protocol import read_line
from robot_protocol import send_frame


DEFAULT_PORT = "/dev/ttyUSB0"


def build_base_command(args: argparse.Namespace) -> tuple[str, list[object]]:
    """
    根据命令行参数生成底盘命令字和参数。

    当前规划动作：
      STOP        -> 底盘停车
      IDLE        -> 底盘进入空闲状态
      MOVE,L,R    -> 设置左右轮目标速度
      SAFE,ON/OFF -> 开启或关闭超声波安全检查
      TRACK,ON/OFF -> 开启或关闭循迹模式
      AVOID,ON    -> 进入避障模式
      STATUS      -> 查询底盘当前状态
    """

    action = args.action.upper()
    if action in ("STOP", "IDLE"):
        return action, []
    if action == "MOVE":
        return "MOVE", [args.left, args.right]
    if action in ("SAFE", "TRACK"):
        return action, [args.switch.upper()]
    if action == "AVOID":
        return "AVOID", ["ON"]
    if action in (
        "STATUS", "STATUSDBG", "FORWARD", "BACKWARD", "ROTATE_LEFT",
        "ROTATE_RIGHT", "RETURN_RIGHT",
    ):
        return action, []
    raise ValueError(f"不支持的底盘动作: {args.action}")


def print_expected_reply(seq: int, command: str) -> None:
    """打印后续 STM32 新协议建议返回的内容，方便联调时对照。"""

    print("期望回传示例:")
    print(f"  成功: $BASE,{seq:03d},ACK,{command}*CS")
    print(f"  失败: $BASE,{seq:03d},ERR,REASON*CS")
    print(f"  状态: $BASE,{seq:03d},STATUS,STATE=IDLE,SAFE=ON,DIST=35.2*CS")


def run_once(args: argparse.Namespace) -> int:
    """打开串口，发送一帧底盘命令，并按需读取应答。"""

    command, params = build_base_command(args)
    frame = build_frame("BASE", args.seq, command, params)

    print(f"目标: 底盘 STM32 USART1")
    print(f"树莓派串口设备: {args.port}, {args.baudrate} 8N1")
    print(f"发送帧: {frame.strip()}")
    print_expected_reply(args.seq, command)

    try:
        with open_serial(args.port, args.baudrate, args.timeout) as ser:
            time.sleep(args.settle_time)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            send_frame(ser, frame)
            if args.no_read:
                print("已发送，不读取应答。")
                return 0

            reply = read_line(ser)
            if not reply:
                print("未收到应答。当前 STM32 若尚未升级新协议，这是正常现象。")
                return 1

            print(f"收到原始应答: {reply}")
            if reply.startswith("$"):
                parsed = parse_frame(reply)
                print(f"解析结果: target={parsed.target}, seq={parsed.seq}, command={parsed.command}, params={parsed.params}")
            else:
                print("收到的是旧文本应答，不是新数据帧。")
            return 0

    except (serial.SerialException, ValueError) as exc:
        print(f"通信错误: {exc}", file=sys.stderr)
        return 2


def build_parser() -> argparse.ArgumentParser:
    """创建底盘测试脚本的命令行参数。"""

    parser = argparse.ArgumentParser(description="底盘 STM32 数据帧串口诊断")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"串口设备，默认 {DEFAULT_PORT}")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help=f"波特率，默认 {DEFAULT_BAUDRATE}")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="读取应答超时时间，单位秒")
    parser.add_argument("--settle-time", type=float, default=0.2, help="打开串口后的等待时间，单位秒")
    parser.add_argument("--seq", type=int, default=1, help="命令序号，默认 1")
    parser.add_argument("--no-read", action="store_true", help="只发送，不等待 STM32 应答")

    parser.add_argument(
        "--action",
        choices=[
            "STOP", "IDLE", "MOVE", "SAFE", "TRACK", "AVOID", "STATUS", "STATUSDBG",
            "FORWARD", "BACKWARD", "ROTATE_LEFT", "ROTATE_RIGHT", "RETURN_RIGHT",
        ],
        default="STOP",
        help="底盘动作，默认 STOP",
    )
    parser.add_argument("--left", type=float, default=0.0, help="MOVE 动作的左轮速度")
    parser.add_argument("--right", type=float, default=0.0, help="MOVE 动作的右轮速度")
    parser.add_argument("--switch", choices=["ON", "OFF"], default="ON", help="SAFE/TRACK 动作的开关状态")
    return parser


def main() -> int:
    """程序入口。"""

    return run_once(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
