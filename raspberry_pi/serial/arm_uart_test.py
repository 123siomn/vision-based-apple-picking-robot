#!/usr/bin/env python3
# 上面这一行叫 shebang。
# 在 Linux/树莓派上，如果给这个文件加了可执行权限，可以直接用 ./arm_uart_test.py 运行。
# 现在你用 python3 arm_uart_test.py 运行也完全可以。

"""
树莓派到机械臂 STM32 的 USART1 串口测试脚本。

这个脚本只做一件事：
  通过树莓派上的 /dev/ttyUSB0，把一条文本命令发送给机械臂 STM32。

当前机械臂 STM32 支持的测试命令有：
  STOP
  HOME 1000
  SERVO 1 1500 1000

硬件连接方向：
  树莓派 Python 程序
    -> /dev/ttyUSB0
    -> USB-TTL 模块
    -> STM32 USART1 RX

如果只想验证 STM32 能不能收到命令，至少需要：
  USB-TTL TXD -> STM32 USART1 RX
  USB-TTL GND -> STM32 GND

如果还想在树莓派终端看到 STM32 返回的 OK/ERR，需要再接：
  STM32 USART1 TX -> USB-TTL RXD

机械臂 STM32 当前串口参数：
  波特率 115200
  数据位 8
  无校验
  停止位 1
  简称 115200 8N1
"""

# argparse 用来解析命令行参数。
# 例如：
#   --command "HOME 1000"
#   --port /dev/ttyUSB0
import argparse

# sys 用来访问系统相关功能。
# 这里主要用 sys.stderr 把错误信息打印到错误输出。
import sys

# time 用来做简单延时。
# 打开串口后稍微等一下，可以让 USB-TTL 状态更稳定。
import time

# serial 来自 pyserial 库。
# 它负责真正打开 /dev/ttyUSB0，并进行串口读写。
import serial


# 默认串口设备。
# 树莓派插入 CH340 / CP2102 这类 USB-TTL 模块后，常见设备名就是 /dev/ttyUSB0。
DEFAULT_PORT = "/dev/ttyUSB0"

# 默认波特率。
# 这个必须和 STM32 机械臂 USART1 初始化里的波特率一致。
DEFAULT_BAUDRATE = 115200

# 默认读取超时时间，单位是秒。
# 如果 1 秒内没有读到 STM32 返回内容，ser.readline() 就会返回空数据。
DEFAULT_TIMEOUT = 1.0


def open_serial(port: str, baudrate: int, timeout: float) -> serial.Serial:
    """
    打开树莓派上的 USB-TTL 串口。

    参数：
      port: 串口设备名，例如 /dev/ttyUSB0
      baudrate: 波特率，例如 115200
      timeout: 读取超时时间，单位秒

    返回：
      serial.Serial 对象，后面用它来 write() 发送数据、readline() 读取数据。
    """

    # serial.Serial(...) 会打开串口。
    # 如果串口不存在、权限不够、设备被占用，这里会抛出 serial.SerialException。
    return serial.Serial(
        # 指定要打开哪个串口设备。
        port=port,

        # 指定波特率，必须和 STM32 一致。
        baudrate=baudrate,

        # 数据位设置为 8 位。
        bytesize=serial.EIGHTBITS,

        # 不使用奇偶校验。
        parity=serial.PARITY_NONE,

        # 停止位设置为 1 位。
        stopbits=serial.STOPBITS_ONE,

        # 读取超时时间。
        # 如果没有这个 timeout，readline() 可能一直卡住等待。
        timeout=timeout,

        # 写入超时时间。
        # 如果 USB-TTL 异常，write() 不会无限卡住。
        write_timeout=timeout,
    )


def send_line(ser: serial.Serial, command: str) -> None:
    """
    向 STM32 发送一行文本命令。

    机械臂 STM32 端的协议是“按行接收”：
      收到 \r 或 \n 后，才认为一条命令结束。

    所以这里必须在命令后面补上 \r\n。
    """

    # command.strip() 去掉用户输入命令前后的空格和换行。
    # 例如 " HOME 1000 " 会变成 "HOME 1000"。
    # 后面加 "\r\n" 是为了告诉 STM32：这一行命令已经结束，可以开始解析了。
    line = command.strip() + "\r\n"

    # Python 字符串不能直接通过串口发送，需要先编码成字节。
    # 当前协议只使用英文命令和数字，所以 ascii 编码就够了。
    data = line.encode("ascii")

    # ser.write(data) 把字节真正写入 /dev/ttyUSB0。
    # 数据路径是：Python -> /dev/ttyUSB0 -> USB-TTL TXD -> STM32 RX。
    ser.write(data)

    # flush() 等待系统把缓冲区里的数据尽量发出去。
    # 这样可以减少“程序结束太快，数据还没完全发完”的风险。
    ser.flush()


def read_reply(ser: serial.Serial) -> str:
    """
    从 STM32 读取一行返回信息。

    STM32 当前会返回类似：
      OK STOP
      OK HOME
      OK SERVO
      ERR CMD
      ERR SERVO

    如果你没有接 STM32 TX -> USB-TTL RX，这里就读不到返回。
    """

    # readline() 会一直读，直到遇到换行符，或者 timeout 超时。
    # 返回值是 bytes 类型。
    data = ser.readline()

    # 把 bytes 解码成字符串。
    # errors="replace" 表示如果遇到无法解码的字节，就用替代符号显示，避免程序崩溃。
    # strip() 去掉末尾的 \r\n，打印时更清楚。
    return data.decode("ascii", errors="replace").strip()


def run_once(args: argparse.Namespace) -> int:
    """
    执行一次完整测试：
      1. 打开串口
      2. 清空串口收发缓冲区
      3. 发送一条命令
      4. 根据参数决定是否读取 STM32 应答

    返回值：
      0 表示发送成功，并且在需要读取时读到了应答
      1 表示发送成功，但没有读到应答
      2 表示串口打开或读写出错
    """

    try:
        # with 语句会在进入时打开串口，在退出时自动关闭串口。
        # 这样即使中途出错，串口资源也会被释放。
        with open_serial(args.port, args.baudrate, args.timeout) as ser:

            # 打开串口后等待一小会。
            # 有些 USB-TTL 模块刚打开时需要一点时间稳定。
            time.sleep(args.settle_time)

            # 清空输入缓冲区，避免读到上一次测试残留的数据。
            ser.reset_input_buffer()

            # 清空输出缓冲区，避免上一次残留数据影响本次发送。
            ser.reset_output_buffer()

            # 打印当前串口配置，方便你确认程序确实打开了 /dev/ttyUSB0。
            print(f"串口已打开: {args.port}, {args.baudrate} 8N1")

            # 打印将要发送的命令。
            # !r 会带引号显示，例如 'HOME 1000'，便于看清空格。
            print(f"发送命令: {args.command!r}")

            # 真正发送命令到 STM32。
            send_line(ser, args.command)

            # 如果用户加了 --no-read，就只发送，不等待 STM32 返回。
            # 这个模式适合只接了 USB-TTL TXD -> STM32 RX 的情况。
            if args.no_read:
                print("已发送，不读取应答。")
                return 0

            # 如果没有 --no-read，就尝试读取 STM32 返回的一行内容。
            reply = read_reply(ser)

            # 如果 reply 不是空字符串，说明读到了 STM32 返回。
            if reply:
                print(f"收到应答: {reply!r}")
                return 0

            # 如果超时后仍然没有读到返回，就会走到这里。
            # 这不一定代表 STM32 没收到，因为你可能没有接 STM32 TX 线。
            print("未收到应答。若只连接了 STM32 RX，这是正常的；若要读回 OK/ERR，请连接 STM32 TX 到 USB-TTL RX。")
            return 1

    # 如果打开串口失败、串口权限不足、设备被拔掉，pyserial 会抛出 SerialException。
    except serial.SerialException as exc:
        # 错误信息打印到 stderr，方便区分正常输出和错误输出。
        print(f"串口错误: {exc}", file=sys.stderr)

        # 给出常见排查方向。
        print("请检查 /dev/ttyUSB0 是否存在、权限是否足够、USB-TTL 是否插好。", file=sys.stderr)
        return 2


def build_parser() -> argparse.ArgumentParser:
    """
    创建命令行参数解析器。

    有了 argparse，就可以这样运行：
      python3 arm_uart_test.py --command "HOME 1000"
      python3 arm_uart_test.py --port /dev/ttyUSB1
      python3 arm_uart_test.py --no-read
    """

    # 创建解析器，并设置帮助说明。
    parser = argparse.ArgumentParser(description="树莓派到机械臂 STM32 的串口收发测试")

    # --port 用来指定串口设备，不写时默认使用 /dev/ttyUSB0。
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"串口设备，默认 {DEFAULT_PORT}",
    )

    # --baudrate 用来指定波特率，不写时默认 115200。
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"波特率，默认 {DEFAULT_BAUDRATE}",
    )

    # --timeout 用来指定读取应答时最多等多久。
    # 如果 STM32 没有返回，超过这个时间就放弃等待。
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="读取应答超时时间，单位秒",
    )

    # --settle-time 是打开串口后的等待时间，一般不用改。
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.2,
        help="打开串口后等待时间，单位秒",
    )

    # --command 是真正发送给 STM32 的命令。
    # 默认发 STOP，比较安全，不会让舵机乱动。
    parser.add_argument(
        "--command",
        default="STOP",
        help="发送给机械臂 STM32 的命令，默认 STOP",
    )

    # --no-read 表示只发送，不读取 STM32 的返回。
    # 如果你暂时只接了 TXD -> RX，这个参数很有用。
    parser.add_argument(
        "--no-read",
        action="store_true",
        help="只发送命令，不等待 STM32 应答",
    )

    # 返回解析器对象，main() 里会使用它。
    return parser


def main() -> int:
    """
    程序入口函数。

    Python 文件从命令行运行时，会先进入最下面的：
      if __name__ == "__main__":

    然后调用这里的 main()。
    """

    # 创建命令行参数解析器。
    parser = build_parser()

    # 读取并解析用户在命令行输入的参数。
    # 例如：--command "HOME 1000"
    args = parser.parse_args()

    # 使用解析出来的参数执行一次串口测试。
    return run_once(args)


# 这个判断表示：
#   只有当本文件被直接运行时，才执行 main()。
#   如果以后别的 Python 文件 import 这个文件，就不会自动运行 main()。
if __name__ == "__main__":

    # main() 返回 0/1/2。
    # raise SystemExit(...) 会把这个返回值作为程序退出码。
    raise SystemExit(main())
