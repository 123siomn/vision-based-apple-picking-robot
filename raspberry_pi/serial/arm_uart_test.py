#!/usr/bin/env python3
"""
树莓派到机械臂 STM32 的 USART1 串口测试脚本。

连接方向：
  树莓派 Python 程序 -> /dev/ttyUSB0 -> USB-TTL -> STM32 USART1 RX

如果要读取 STM32 返回的 OK/ERR，还需要：
  STM32 USART1 TX -> USB-TTL RX
  STM32 GND       -> USB-TTL GND / 树莓派 GND
"""

import argparse
import sys
import time

import serial


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1.0


def open_serial(port: str, baudrate: int, timeout: float) -> serial.Serial:
    """打开树莓派上的 USB-TTL 串口。"""
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
    )


def send_line(ser: serial.Serial, command: str) -> None:
    """向 STM32 发送一行文本命令，协议要求以换行符结束。"""
    line = command.strip() + "\r\n"
    ser.write(line.encode("ascii"))
    ser.flush()


def read_reply(ser: serial.Serial) -> str:
    """读取 STM32 返回的一行应答，例如 OK STOP 或 ERR CMD。"""
    data = ser.readline()
    return data.decode("ascii", errors="replace").strip()


def run_once(args: argparse.Namespace) -> int:
    """执行一次串口发送测试。"""
    try:
        with open_serial(args.port, args.baudrate, args.timeout) as ser:
            time.sleep(args.settle_time)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            print(f"串口已打开: {args.port}, {args.baudrate} 8N1")
            print(f"发送命令: {args.command!r}")
            send_line(ser, args.command)

            if args.no_read:
                print("已发送，不读取应答。")
                return 0

            reply = read_reply(ser)
            if reply:
                print(f"收到应答: {reply!r}")
                return 0

            print("未收到应答。若只连接了 STM32 RX，这是正常的；若要读回 OK/ERR，请连接 STM32 TX 到 USB-TTL RX。")
            return 1
    except serial.SerialException as exc:
        print(f"串口错误: {exc}", file=sys.stderr)
        print("请检查 /dev/ttyUSB0 是否存在、权限是否足够、USB-TTL 是否插好。", file=sys.stderr)
        return 2


def build_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。"""
    parser = argparse.ArgumentParser(description="树莓派到机械臂 STM32 的串口收发测试")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"串口设备，默认 {DEFAULT_PORT}")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help=f"波特率，默认 {DEFAULT_BAUDRATE}")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="读取应答超时时间，单位秒")
    parser.add_argument("--settle-time", type=float, default=0.2, help="打开串口后等待时间，单位秒")
    parser.add_argument("--command", default="STOP", help="发送给机械臂 STM32 的命令，默认 STOP")
    parser.add_argument("--no-read", action="store_true", help="只发送命令，不等待 STM32 应答")
    return parser


def main() -> int:
    """程序入口。"""
    parser = build_parser()
    args = parser.parse_args()
    return run_once(args)


if __name__ == "__main__":
    raise SystemExit(main())
