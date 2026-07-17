"""固定演示场景下的循迹、视觉、深度与一次抓取控制器。"""

from collections import deque
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np


RASPBERRYPI_DIR = Path(__file__).resolve().parents[1]
SERIAL_DIR = RASPBERRYPI_DIR / "serial"
if str(SERIAL_DIR) not in sys.path:
    sys.path.insert(0, str(SERIAL_DIR))

from robot_uart_frame import build_frame, open_serial, parse_frame, read_line, send_frame


# ------------------------------ 可标定参数 ------------------------------

# 当前树莓派实测：ttyUSB1 连接机械臂，ttyUSB0 连接底盘。
# USB 枚举号可能随插拔顺序变化，必要时可通过命令行参数覆盖。
DEFAULT_ARM_PORT = "/dev/ttyUSB1"
DEFAULT_BASE_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
BASE_STATUS_INTERVAL_SEC = 0.5

CAMERA_PATH = "/dev/video0"
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SERVER_PORT = 8080
JPEG_QUALITY = 80

MIN_TARGET_AREA = 500
TARGET_STABLE_FRAMES = 5
TARGET_LOST_TIMEOUT_SEC = 2.0

# 固定抓取位置的实测视觉参考点与深度范围。
TARGET_CX = 316
TARGET_CY = 235
CX_TOL = 20
CY_TOL = 20
Z_TRACK_EXIT_MAX_MM = 820.0
Z_EXPECTED_ALIGN_MIN_MM = 760.0
Z_GRAB_MIN_MM = 700.0
Z_GRAB_MAX_MM = 745.0

# 只在已经发现目标后读取深度。每次 helper 调用会暂时释放 RGB 节点。
DEPTH_EXE = RASPBERRYPI_DIR / "depth_helper" / "depth_xyz_reader"
DEPTH_RADIUS = 5
DEPTH_FRAMES = 4
DEPTH_TIMEOUT_SEC = 8.0
DEPTH_INTERVAL_SEC = 1.5
DEPTH_MIN_VALID = 20
# depth helper 本身会在一次调用内读取多帧并对有效 ROI 深度取中位数。
# 当前 Astra SDK 与 V4L2 RGB 节点不能稳定地反复切换，因此固定演示只采用一次有效深度结果。
DEPTH_MEDIAN_WINDOW = 1
MAX_CONSECUTIVE_DEPTH_FAILURES = 3
CAMERA_RELEASE_SETTLE_SEC = 0.5
CAMERA_REOPEN_TIMEOUT_SEC = 12.0

# MOVE 协议接收 -5.0~5.0 的速度值，STM32 再映射到 -99~99 开环 PWM。
# 当前带载演示使用直行 PWM 50、转向低速轮 40 和高速轮 60。
# 2.02 约映射为 PWM 40，2.53 约映射为 PWM 50，3.03 约映射为 PWM 60；每次短脉冲后必须停下并等待相机稳定。
TRACK_PWM = 50
ALIGN_FORWARD_SPEED = 2.53
ALIGN_TURN_SLOW_SPEED = 2.02
ALIGN_TURN_FAST_SPEED = 3.03
ALIGN_PULSE_MS = 60
ALIGN_SETTLE_SEC = 0.8
MAX_ALIGN_PULSES = 40
FINE_ALIGN_TIMEOUT_SEC = 90.0

# 固定演示用机械臂动作：ID1=500 张开，ID1=1201 夹紧。
HOME_TIME_MS = 3000
OPEN_TIME_MS = 1000
READY_TIME_MS = 5000
CLOSE_TIME_MS = 1500
READY_OPEN_POSE = (500, 1500, 1069, 817, 2309, 2327)
CLOSE_PULSE = 1201


class DemoState(str, Enum):
    """树莓派上层演示状态。底盘 STM32 内部状态保持不变。"""

    IDLE = "IDLE"
    HOME_WAIT = "HOME_WAIT"
    LINE_TRACK = "LINE_TRACK"
    TARGET_DETECTED = "TARGET_DETECTED"
    FINE_ALIGN = "FINE_ALIGN"
    READY_WAIT = "READY_WAIT"
    CLOSE_WAIT = "CLOSE_WAIT"
    RETURN_HOME_WAIT = "RETURN_HOME_WAIT"
    DONE = "DONE"
    FAILSAFE = "FAILSAFE"


def find_ok_line(output):
    """从 Astra warning 与程序输出中找出真正的 OK 深度结果行。"""
    for line in output.splitlines():
        if line.startswith("OK "):
            return line
    return None


def parse_value(line, name, value_type=float):
    """解析 depth helper 输出中 x/y/z/valid 等字段。"""
    match = re.search(rf"(?:^|\s){name}=([-0-9.]+)", line)
    if match is None:
        return None
    return value_type(match.group(1))


def read_depth_xyz(cx, cy):
    """在 RGB 摄像头已释放的条件下，读取目标点附近 ROI 深度。"""
    if not DEPTH_EXE.exists():
        return {"ok": False, "error": "DEPTH_HELPER_NOT_FOUND"}

    started_at = time.monotonic()
    try:
        result = subprocess.run(
            [str(DEPTH_EXE), str(cx), str(cy), str(DEPTH_RADIUS), str(DEPTH_FRAMES)],
            capture_output=True,
            text=True,
            timeout=DEPTH_TIMEOUT_SEC,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "DEPTH_TIMEOUT"}
    except OSError as exc:
        return {"ok": False, "error": f"DEPTH_LAUNCH_FAILED {exc}"}

    output = (result.stdout + "\n" + result.stderr).strip()
    ok_line = find_ok_line(output)
    if ok_line is None:
        return {
            "ok": False,
            "error": f"DEPTH_FAILED {result.returncode}",
            "detail": output[-300:],
        }

    x_mm = parse_value(ok_line, "x")
    y_mm = parse_value(ok_line, "y")
    z_mm = parse_value(ok_line, "z")
    valid = parse_value(ok_line, "valid", int)
    if x_mm is None or y_mm is None or z_mm is None or valid is None:
        return {"ok": False, "error": "DEPTH_PARSE_FAILED"}

    return {
        "ok": True,
        "x": x_mm,
        "y": y_mm,
        "z": z_mm,
        "valid": valid,
        "elapsed": time.monotonic() - started_at,
    }


def open_rgb_camera():
    """通过经验证可用的编号方式打开 Astra RGB 的 /dev/video0。"""
    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开 RGB 摄像头：{CAMERA_PATH} index={CAMERA_INDEX}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    ok, frame = camera.read()
    if not ok or frame is None:
        camera.release()
        raise RuntimeError(f"RGB 摄像头读取失败：{CAMERA_PATH} index={CAMERA_INDEX}")

    print(f"RGB camera opened: {CAMERA_PATH} index={CAMERA_INDEX}", flush=True)
    return camera


def reopen_camera_with_retry(stop_event):
    """depth helper 退出后等待 V4L2 设备恢复，再重新打开 RGB。"""
    deadline = time.monotonic() + CAMERA_REOPEN_TIMEOUT_SEC
    last_error = None
    while not stop_event.is_set() and time.monotonic() < deadline:
        try:
            return open_rgb_camera()
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.2)
    if last_error is not None:
        raise last_error
    raise RuntimeError("重新打开 RGB 摄像头失败")


def build_red_mask(frame):
    """在 HSV 空间提取草莓的两段红色范围，并进行基础降噪。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_low = cv2.inRange(
        hsv,
        np.array([0, 80, 50], dtype=np.uint8),
        np.array([10, 255, 255], dtype=np.uint8),
    )
    mask_high = cv2.inRange(
        hsv,
        np.array([170, 80, 50], dtype=np.uint8),
        np.array([180, 255, 255], dtype=np.uint8),
    )
    mask = cv2.bitwise_or(mask_low, mask_high)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def detect_largest_red_target(frame):
    """检测最大有效红色轮廓，并用轮廓质心作为草莓中心。"""
    mask = build_red_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contour = None
    area = 0.0
    for candidate in contours:
        candidate_area = cv2.contourArea(candidate)
        if candidate_area >= MIN_TARGET_AREA and candidate_area > area:
            contour = candidate
            area = candidate_area

    if contour is None:
        return None

    x, y, width, height = cv2.boundingRect(contour)
    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
    else:
        cx = x + width // 2
        cy = y + height // 2

    return {"bbox": (x, y, width, height), "cx": cx, "cy": cy, "area": area}


class RobotLinks:
    """复用既有文本帧协议，管理底盘与机械臂两路 USB-TTL 串口。"""

    def __init__(self, arm_port, base_port):
        self.arm_port = arm_port
        self.base_port = base_port
        self.arm_serial = None
        self.base_serial = None
        self.sequence = 1

    def connect(self, target=None):
        """按目标模块打开对应串口，避免一侧掉线阻塞另一侧的安全命令。"""
        if target not in (None, "ARM", "BASE"):
            raise ValueError(f"未知串口目标：{target}")

        if target in (None, "ARM") and self.arm_serial is None:
            self.arm_serial = open_serial(self.arm_port, BAUDRATE, 1.0)
            time.sleep(0.2)
        if target in (None, "BASE") and self.base_serial is None:
            self.base_serial = open_serial(self.base_port, BAUDRATE, 1.0)
            time.sleep(0.2)

    def close(self):
        """关闭串口设备。"""
        for serial_port in (self.arm_serial, self.base_serial):
            if serial_port is not None:
                serial_port.close()
        self.arm_serial = None
        self.base_serial = None

    def _send(self, target, command, params=()):
        """发送一条既有协议帧，并检查 STM32 的 ACK/ERR 应答。"""
        self.connect(target)
        serial_port = self.arm_serial if target == "ARM" else self.base_serial
        seq = self.sequence
        self.sequence = 1 if self.sequence >= 999 else self.sequence + 1
        frame = build_frame(target, seq, command, params)
        send_frame(serial_port, frame)
        reply = read_line(serial_port)
        if not reply:
            raise RuntimeError(f"{target} 命令无应答：{command}")

        parsed = parse_frame(reply)
        if parsed.target != target or parsed.seq != f"{seq:03d}":
            raise RuntimeError(f"{target} 应答序号不匹配：{reply}")
        if parsed.command == "ERR":
            raise RuntimeError(f"{target} 返回错误：{reply}")
        expected_reply = "STATUS" if command == "STATUS" else "ACK"
        if parsed.command != expected_reply:
            raise RuntimeError(f"{target} 应答类型异常：{reply}")
        return reply

    def base_track(self, enabled):
        """启用或关闭 STM32 现有红外循迹状态。"""
        return self._send("BASE", "TRACK", ("ON" if enabled else "OFF",))

    def base_move(self, left_speed, right_speed):
        """使用底盘既有 MOVE 命令发送 -5.0~5.0 的左右轮速度值。"""
        return self._send("BASE", "MOVE", (left_speed, right_speed))

    def base_stop(self):
        """让底盘立即停止。"""
        return self._send("BASE", "STOP")

    def base_status(self):
        """读取底盘 STATUS 中的循迹传感器、PWM、编码器和轮速诊断信息。"""
        reply = self._send("BASE", "STATUS")
        return ",".join(parse_frame(reply).params)

    def arm_home(self, move_time):
        """让六个舵机通过现有 HOME 命令回到中位。"""
        return self._send("ARM", "HOME", (move_time,))

    def arm_servo(self, servo_id, pulse, move_time):
        """通过既有 SERVO 命令设置单个 PWM 舵机。"""
        return self._send("ARM", "SERVO", (servo_id, pulse, move_time))

    def arm_pose(self, pulses, move_time):
        """连续下发六个已有 SERVO 命令，以相同动作时间组成固定姿态。"""
        for servo_id, pulse in enumerate(pulses, start=1):
            self.arm_servo(servo_id, pulse, move_time)

    def arm_return_home_keep_gripper(self, move_time):
        """仅让 2~6 号舵机回中位，保持 1 号夹爪当前位置不变。"""
        for servo_id in range(2, 7):
            self.arm_servo(servo_id, 1500, move_time)


class DemoController:
    """一次抓取演示的上层状态机，只在相机线程内发送串口控制命令。"""

    def __init__(self, links):
        self.links = links
        self.lock = threading.RLock()
        self.state = DemoState.IDLE
        self.message = "等待开始"
        self.start_requested = False
        self.reset_requested = False
        self.state_deadline = 0.0
        self.target_stable_count = 0
        self.target_lost_since = None
        self.depth_history = deque(maxlen=DEPTH_MEDIAN_WINDOW)
        self.latest_depth = None
        self.last_depth_request_time = 0.0
        self.depth_failure_count = 0
        self.depth_generation = 0
        self.last_forward_depth_generation = -1
        self.align_pulse_count = 0
        self.fine_align_deadline = 0.0
        self.pulse_stop_at = 0.0
        self.settle_until = 0.0
        self.last_base_action = "STOP"
        self.base_status_detail = "waiting"
        self.last_base_status_time = 0.0

    def request_start(self):
        """由网页请求开始；真正串口动作由相机线程安全执行。"""
        with self.lock:
            if self.state in (DemoState.IDLE, DemoState.DONE, DemoState.FAILSAFE):
                self.start_requested = True

    def request_reset(self):
        """由网页请求停止当前轮次并回到待机状态。"""
        with self.lock:
            self.reset_requested = True

    def snapshot(self):
        """为网页叠加层和状态接口提供只读状态。"""
        with self.lock:
            depth = dict(self.latest_depth) if self.latest_depth is not None else None
            return {
                "state": self.state.value,
                "message": self.message,
                "depth": depth,
                "depth_samples": len(self.depth_history),
                "align_pulses": self.align_pulse_count,
                "base_action": self.last_base_action,
                "base_status": self.base_status_detail,
            }

    def needs_base_status(self):
        """仅在循迹阶段按固定周期查询一次底盘诊断状态，避免占满串口。"""
        if self.state != DemoState.LINE_TRACK:
            return False
        return time.monotonic() - self.last_base_status_time >= BASE_STATUS_INTERVAL_SEC

    def accept_base_status(self, detail):
        """保存最新底盘诊断文本，供网页叠加显示。"""
        self.last_base_status_time = time.monotonic()
        self.base_status_detail = detail

    def _set_state(self, state, message):
        """切换状态并将关键过程打印到终端。"""
        self.state = state
        self.message = message
        print(f"STATE {state.value}: {message}", flush=True)

    def _safe_stop_home(self, reason, final_state=DemoState.FAILSAFE):
        """异常时停止底盘并让机械臂回中位；串口故障时继续保留失败状态。"""
        try:
            self.links.base_stop()
            self.last_base_action = "STOP"
        except Exception as exc:
            print(f"BASE_STOP_FAILED: {exc}", flush=True)
        try:
            self.links.arm_home(HOME_TIME_MS)
        except Exception as exc:
            print(f"ARM_HOME_FAILED: {exc}", flush=True)
        self._set_state(final_state, reason)

    def _begin_home_then_track(self):
        """每轮开始先确保底盘静止、机械臂回中位，再启动循迹。"""
        try:
            self.links.base_stop()
            self.last_base_action = "STOP"
            self.links.arm_home(HOME_TIME_MS)
        except Exception as exc:
            self._safe_stop_home(f"串口启动失败：{exc}")
            return

        self._reset_cycle_data()
        self.state_deadline = time.monotonic() + HOME_TIME_MS / 1000.0 + 0.3
        self._set_state(DemoState.HOME_WAIT, "机械臂回中位")

    def _reset_cycle_data(self):
        """清除上一轮视觉、深度和微调记录。"""
        self.target_stable_count = 0
        self.target_lost_since = None
        self.depth_history.clear()
        self.latest_depth = None
        self.last_depth_request_time = 0.0
        self.depth_failure_count = 0
        self.depth_generation = 0
        self.last_forward_depth_generation = -1
        self.align_pulse_count = 0
        self.fine_align_deadline = 0.0
        self.pulse_stop_at = 0.0
        self.settle_until = 0.0

    def _start_tracking(self):
        """进入正常红外循迹阶段。"""
        try:
            self.links.base_track(True)
            self.last_base_action = "TRACK_ON"
            self._set_state(DemoState.LINE_TRACK, f"循迹中 PWM={TRACK_PWM}")
        except Exception as exc:
            self._safe_stop_home(f"无法启动循迹：{exc}")

    def _open_gripper(self):
        """首次稳定发现目标后只张开夹爪，机械臂其它关节保持中位。"""
        try:
            self.links.arm_servo(1, READY_OPEN_POSE[0], OPEN_TIME_MS)
            self._set_state(DemoState.TARGET_DETECTED, "发现目标，夹爪已张开，采样深度")
        except Exception as exc:
            self._safe_stop_home(f"夹爪张开失败：{exc}")

    def _stop_tracking_for_fine_align(self):
        """目标进入预对准距离后退出循迹，开始短脉冲视觉微调。"""
        try:
            self.links.base_track(False)
            self.links.base_stop()
            self.last_base_action = "STOP"
        except Exception as exc:
            self._safe_stop_home(f"停止循迹失败：{exc}")
            return

        self.fine_align_deadline = time.monotonic() + FINE_ALIGN_TIMEOUT_SEC
        self.settle_until = time.monotonic() + ALIGN_SETTLE_SEC
        self._set_state(DemoState.FINE_ALIGN, "退出循迹，等待图像稳定后微调")

    def _start_fixed_grab_after_depth(self, target, depth):
        """使用最后一帧 RGB 与一次可靠深度结果，执行固定位置的一次抓取。"""
        error_x = target["cx"] - TARGET_CX
        error_y = target["cy"] - TARGET_CY

        try:
            self.links.base_track(False)
            self.links.base_stop()
            self.last_base_action = "STOP"
        except Exception as exc:
            self._safe_stop_home(f"停止循迹失败：{exc}")
            return

        if abs(error_x) > CX_TOL or abs(error_y) > CY_TOL:
            self._safe_stop_home(
                f"目标未进入固定抓取框 ex={error_x}px ey={error_y}px"
            )
            return

        if Z_GRAB_MIN_MM <= depth["z"] <= Z_GRAB_MAX_MM:
            self._start_ready_pose()
            return

        self._safe_stop_home(f"固定抓取距离不匹配 Z={depth['z']:.1f}mm")

    def _start_base_pulse(self, left_speed, right_speed, action):
        """执行一次有限时长的开环底盘脉冲，结束后必定 STOP。"""
        try:
            self.links.base_move(left_speed, right_speed)
            self.last_base_action = action
            self.pulse_stop_at = time.monotonic() + ALIGN_PULSE_MS / 1000.0
            self.align_pulse_count += 1
            self.message = f"{action} 第 {self.align_pulse_count}/{MAX_ALIGN_PULSES} 次"
        except Exception as exc:
            self._safe_stop_home(f"底盘微调失败：{exc}")

    def _finish_pulse_if_due(self, now):
        """到达脉冲时间后停止底盘，并等待相机画面稳定。"""
        if self.pulse_stop_at <= 0.0 or now < self.pulse_stop_at:
            return False
        try:
            self.links.base_stop()
            self.last_base_action = "STOP"
            self.pulse_stop_at = 0.0
            self.settle_until = now + ALIGN_SETTLE_SEC
            self.message = "底盘已停止，等待图像稳定"
        except Exception as exc:
            self._safe_stop_home(f"底盘停止失败：{exc}")
        return True

    def _start_ready_pose(self):
        """目标满足最终视觉与深度条件后，让机械臂缓慢到达张开抓取姿态。"""
        try:
            self.links.base_stop()
            self.last_base_action = "STOP"
            self.links.arm_pose(READY_OPEN_POSE, READY_TIME_MS)
        except Exception as exc:
            self._safe_stop_home(f"抓取准备姿态失败：{exc}")
            return

        self.state_deadline = time.monotonic() + READY_TIME_MS / 1000.0 + 0.4
        self._set_state(DemoState.READY_WAIT, "机械臂正在到达抓取姿态")

    def _start_close(self):
        """机械臂到位后夹紧 ID1，其他五个关节保持抓取姿态。"""
        try:
            self.links.arm_servo(1, CLOSE_PULSE, CLOSE_TIME_MS)
        except Exception as exc:
            self._safe_stop_home(f"夹爪夹紧失败：{exc}")
            return

        self.state_deadline = time.monotonic() + CLOSE_TIME_MS / 1000.0 + 0.3
        self._set_state(DemoState.CLOSE_WAIT, "夹爪正在夹紧")

    def _start_return_home(self):
        """夹紧完成后仅收回 2~6 号关节，夹爪保持夹紧以保护水果。"""
        try:
            self.links.arm_return_home_keep_gripper(HOME_TIME_MS)
        except Exception as exc:
            self._safe_stop_home(f"机械臂回中位失败：{exc}")
            return

        self.state_deadline = time.monotonic() + HOME_TIME_MS / 1000.0 + 0.3
        self._set_state(DemoState.RETURN_HOME_WAIT, "机械臂回中位，夹爪保持夹紧")

    def _depth_median(self):
        """返回最近有效深度样本的中位数；未满窗口时返回 None。"""
        if len(self.depth_history) < DEPTH_MEDIAN_WINDOW:
            return None
        values = sorted(sample["z"] for sample in self.depth_history)
        return values[len(values) // 2]

    def needs_depth(self, target):
        """只在已发现目标且尚未抓取时按周期发起一次深度读取。"""
        if target is None:
            return False
        if self.state not in (DemoState.TARGET_DETECTED, DemoState.FINE_ALIGN):
            return False
        # 微调脉冲与停车稳定阶段不切换 Astra 接口，避免用运动中的旧像素读深度。
        if self.pulse_stop_at > 0.0 or time.monotonic() < self.settle_until:
            return False
        return time.monotonic() - self.last_depth_request_time >= DEPTH_INTERVAL_SEC

    def accept_depth(self, depth, target):
        """处理一次深度结果，决定继续循迹或执行固定位置的一次抓取。"""
        self.last_depth_request_time = time.monotonic()
        depth["cx"] = target["cx"]
        depth["cy"] = target["cy"]
        self.latest_depth = depth

        if not depth.get("ok"):
            self.depth_failure_count += 1
            self.message = depth.get("error", "DEPTH_FAILED")
            print(f"DEPTH_FAILED: {self.message}", flush=True)
            if self.depth_failure_count >= MAX_CONSECUTIVE_DEPTH_FAILURES:
                self._safe_stop_home("深度连续读取失败")
            return

        if depth["valid"] < DEPTH_MIN_VALID:
            self.depth_failure_count += 1
            self.message = f"深度有效点过少 valid={depth['valid']}"
            print(f"DEPTH_IGNORED valid={depth['valid']}", flush=True)
            if self.depth_failure_count >= MAX_CONSECUTIVE_DEPTH_FAILURES:
                self._safe_stop_home("深度有效点连续不足")
            return

        self.depth_failure_count = 0
        self.depth_history.append(depth)
        self.depth_generation += 1
        print(
            f"DEPTH_OK z={depth['z']:.1f}mm valid={depth['valid']} "
            f"samples={len(self.depth_history)}/{DEPTH_MEDIAN_WINDOW}",
            flush=True,
        )

        if self.state == DemoState.TARGET_DETECTED:
            if depth["z"] < Z_GRAB_MIN_MM:
                self._safe_stop_home(f"目标过近 Z={depth['z']:.1f}mm")
            elif depth["z"] <= Z_TRACK_EXIT_MAX_MM:
                self._start_fixed_grab_after_depth(target, depth)
            else:
                self.message = f"继续循迹 Z={depth['z']:.1f}mm"

    def is_post_depth_sequence(self):
        """判断是否已进入不再依赖 RGB 相机的机械臂执行阶段。"""
        return self.state in (
            DemoState.READY_WAIT,
            DemoState.CLOSE_WAIT,
            DemoState.RETURN_HOME_WAIT,
            DemoState.DONE,
            DemoState.FAILSAFE,
        )

    def _fine_align_task(self, target, now):
        """使用 RGB 中心误差和分段深度结果进行一次一次的底盘微调。"""
        if now >= self.fine_align_deadline:
            self._safe_stop_home("视觉微调超时")
            return
        if self.align_pulse_count >= MAX_ALIGN_PULSES:
            self._safe_stop_home("视觉微调次数达到上限")
            return
        if self.pulse_stop_at > 0.0 or now < self.settle_until:
            return

        z_mm = self._depth_median()
        if z_mm is None:
            self.message = "等待有效深度"
            return
        if z_mm < Z_GRAB_MIN_MM:
            self._safe_stop_home(f"目标过近 Z={z_mm:.1f}mm")
            return

        error_x = target["cx"] - TARGET_CX
        error_y = target["cy"] - TARGET_CY
        if error_x < -CX_TOL:
            # 右轮更快，车体沿左弧线前进，使画面偏左的目标向中心靠近。
            self._start_base_pulse(ALIGN_TURN_SLOW_SPEED, ALIGN_TURN_FAST_SPEED, "TURN_LEFT")
            return
        if error_x > CX_TOL:
            self._start_base_pulse(ALIGN_TURN_FAST_SPEED, ALIGN_TURN_SLOW_SPEED, "TURN_RIGHT")
            return
        if abs(error_y) > CY_TOL:
            # 底盘不能独立修正图像垂直误差，固定演示场景中应由摆放高度保证。
            self.message = f"等待固定高度 cy误差={error_y}px"
            return
        if Z_GRAB_MIN_MM <= z_mm <= Z_GRAB_MAX_MM:
            self._start_ready_pose()
            return
        if z_mm > Z_GRAB_MAX_MM:
            # 每获得一轮新深度后最多前进一次，避免使用旧 Z 连续前冲。
            if self.last_forward_depth_generation != self.depth_generation:
                self.last_forward_depth_generation = self.depth_generation
                self._start_base_pulse(ALIGN_FORWARD_SPEED, ALIGN_FORWARD_SPEED, "FORWARD")
            else:
                self.message = f"等待新深度 Z={z_mm:.1f}mm"

    def update(self, target):
        """在每帧 RGB 后推进状态机，并处理网页发起的开始或复位请求。"""
        now = time.monotonic()
        with self.lock:
            if self.reset_requested:
                self.reset_requested = False
                self.start_requested = False
                self._safe_stop_home("已人工复位", DemoState.IDLE)
                self._reset_cycle_data()
                return

            if self.start_requested:
                self.start_requested = False
                self._begin_home_then_track()
                return

            if self.state == DemoState.HOME_WAIT:
                if now >= self.state_deadline:
                    self._start_tracking()
                return

            if self.state == DemoState.LINE_TRACK:
                if target is None:
                    self.target_stable_count = 0
                    self.message = "循迹中，未发现目标"
                else:
                    self.target_stable_count += 1
                    self.message = f"目标稳定 {self.target_stable_count}/{TARGET_STABLE_FRAMES}"
                    if self.target_stable_count >= TARGET_STABLE_FRAMES:
                        self._open_gripper()
                return

            if self.state in (DemoState.TARGET_DETECTED, DemoState.FINE_ALIGN):
                if target is None:
                    if self.target_lost_since is None:
                        self.target_lost_since = now
                    elif now - self.target_lost_since >= TARGET_LOST_TIMEOUT_SEC:
                        self._safe_stop_home("目标丢失")
                    return
                self.target_lost_since = None

            if self.state == DemoState.FINE_ALIGN:
                if self._finish_pulse_if_due(now):
                    return
                self._fine_align_task(target, now)
                return

            if self.state == DemoState.READY_WAIT and now >= self.state_deadline:
                self._start_close()
                return

            if self.state == DemoState.CLOSE_WAIT and now >= self.state_deadline:
                self._start_return_home()
                return

            if self.state == DemoState.RETURN_HOME_WAIT and now >= self.state_deadline:
                try:
                    self.links.base_stop()
                    self.last_base_action = "STOP"
                except Exception as exc:
                    self._safe_stop_home(f"最终停止失败：{exc}")
                    return
                self._set_state(DemoState.DONE, "本轮抓取完成，等待人工复位")


def draw_overlay(frame, target, snapshot):
    """在实时画面中显示目标、固定抓取参考点、深度和状态机状态。"""
    cv2.line(frame, (TARGET_CX, 0), (TARGET_CX, FRAME_HEIGHT), (0, 255, 255), 1)
    cv2.line(frame, (0, TARGET_CY), (FRAME_WIDTH, TARGET_CY), (0, 255, 255), 1)
    cv2.rectangle(
        frame,
        (TARGET_CX - CX_TOL, TARGET_CY - CY_TOL),
        (TARGET_CX + CX_TOL, TARGET_CY + CY_TOL),
        (255, 180, 0),
        1,
    )

    lines = [
        f"state={snapshot['state']}",
        snapshot["message"],
        f"base={snapshot['base_action']} align={snapshot['align_pulses']}/{MAX_ALIGN_PULSES}",
    ]

    base_fields = {}
    for item in snapshot["base_status"].split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            base_fields[key] = value
    if base_fields:
        lines.append(
            f"line={base_fields.get('LINE', '----')} act={base_fields.get('ACT', '--')}"
        )
        lines.append(
            f"pwm R/L={base_fields.get('PWM_R', '--')}/{base_fields.get('PWM_L', '--')} "
            f"enc R/L={base_fields.get('ENC_R', '--')}/{base_fields.get('ENC_L', '--')}"
        )
        lines.append(
            f"spd R/L={base_fields.get('SPD_R', '--')}/{base_fields.get('SPD_L', '--')}"
        )

    depth = snapshot["depth"]
    if depth is None:
        lines.append("Z=waiting")
    elif depth.get("ok"):
        lines.append(
            f"Z={depth['z']:.1f}mm valid={depth['valid']} samples={snapshot['depth_samples']}"
        )
    else:
        lines.append(f"Z=-- {depth.get('error', 'DEPTH_FAILED')}")

    if target is None:
        lines.append("NO_TARGET")
    else:
        x, y, width, height = target["bbox"]
        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.circle(frame, (target["cx"], target["cy"]), 5, (0, 0, 255), -1)
        lines.append(
            f"cx={target['cx']} cy={target['cy']} "
            f"err=({target['cx'] - TARGET_CX},{target['cy'] - TARGET_CY})"
        )

    for index, text in enumerate(lines):
        cv2.putText(
            frame,
            text,
            (12, 28 + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return frame


def get_local_ip():
    """获取当前局域网 IP，用于终端提示网页地址。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


class VideoPublisher:
    """将相机线程产生的 JPEG 帧共享给所有 MJPEG 浏览器连接。"""

    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.frame_id = 0

    def publish(self, frame):
        """压缩并发布一张处理后的 RGB 帧。"""
        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if not ok:
            return
        with self.condition:
            self.jpeg = encoded.tobytes()
            self.frame_id += 1
            self.condition.notify_all()

    def error(self, message):
        """相机线程异常时仍发布可读错误帧，避免网页显示破图。"""
        frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            message,
            (20, FRAME_HEIGHT // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        self.publish(frame)

    def generator(self, stop_event):
        """按 MJPEG 边界格式持续输出最新 JPEG。"""
        seen_frame_id = -1
        while not stop_event.is_set():
            with self.condition:
                self.condition.wait_for(
                    lambda: self.frame_id != seen_frame_id or stop_event.is_set(), timeout=2.0
                )
                if stop_event.is_set():
                    return
                if self.jpeg is None or self.frame_id == seen_frame_id:
                    continue
                jpeg = self.jpeg
                seen_frame_id = self.frame_id
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"


class AppContext:
    """HTTP 服务与相机线程共用的运行对象。"""

    def __init__(self, controller, publisher, stop_event):
        self.controller = controller
        self.publisher = publisher
        self.stop_event = stop_event


def create_handler(context):
    """创建绑定当前控制器实例的 HTTP 请求处理器。"""

    class DemoHandler(BaseHTTPRequestHandler):
        """提供首页、MJPEG 视频流及开始/复位控制接口。"""

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                html = (
                    "<!doctype html><html><head><meta charset='utf-8'>"
                    "<title>Apple Picking Demo</title>"
                    "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;}"
                    "main{display:flex;flex-direction:column;align-items:center;padding:16px;gap:12px;}"
                    "img{max-width:100%;height:auto;border:1px solid #444;}"
                    "button{padding:9px 16px;margin:0 4px;}</style></head><body><main>"
                    "<h2>Apple Picking Demo</h2><img src='/stream.mjpg' alt='live stream'>"
                    "<div><button onclick=\"fetch('/start',{method:'POST'})\">Start</button>"
                    "<button onclick=\"fetch('/reset',{method:'POST'})\">Reset</button></div>"
                    "</main></body></html>"
                )
                self.wfile.write(html.encode("utf-8"))
                return

            if self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    for frame in context.publisher.generator(context.stop_event):
                        self.wfile.write(frame)
                except (BrokenPipeError, ConnectionResetError):
                    return
                return

            if self.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return

            self.send_error(404)

        def do_POST(self):
            if self.path == "/start":
                context.controller.request_start()
                self.send_response(204)
                self.end_headers()
                return
            if self.path == "/reset":
                context.controller.request_reset()
                self.send_response(204)
                self.end_headers()
                return
            self.send_error(404)

        def log_message(self, format_text, *args):
            """避免浏览器不断拉取视频时刷屏。"""
            return

    return DemoHandler


def camera_worker(context):
    """独占 RGB 摄像头，执行视觉状态机并在需要时安全切换到 Depth。"""
    camera = None
    try:
        camera = open_rgb_camera()
        while not context.stop_event.is_set():
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("RGB 摄像头读取失败")

            target = detect_largest_red_target(frame)
            context.controller.update(target)

            if context.controller.needs_base_status():
                try:
                    detail = context.controller.links.base_status()
                except Exception as exc:
                    detail = f"STATUS_ERROR={exc}"
                context.controller.accept_base_status(detail)

            if context.controller.needs_depth(target):
                preview = draw_overlay(frame.copy(), target, context.controller.snapshot())
                context.publisher.publish(preview)
                camera.release()
                camera = None
                time.sleep(CAMERA_RELEASE_SETTLE_SEC)

                depth = read_depth_xyz(target["cx"], target["cy"])
                context.controller.accept_depth(depth, target)

                if context.controller.is_post_depth_sequence():
                    # Astra depth helper 退出后，当前设备的 V4L2 RGB 节点可能无法立即恢复。
                    # 固定演示的抓取动作只依赖深度前最后一帧画面，因此冻结该画面并继续推进机械臂状态机。
                    frozen_frame = frame.copy()
                    while not context.stop_event.is_set():
                        context.controller.update(None)
                        result = draw_overlay(
                            frozen_frame.copy(), target, context.controller.snapshot()
                        )
                        context.publisher.publish(result)
                        time.sleep(0.05)
                    break

                time.sleep(CAMERA_RELEASE_SETTLE_SEC)
                camera = reopen_camera_with_retry(context.stop_event)
                continue

            result = draw_overlay(frame, target, context.controller.snapshot())
            context.publisher.publish(result)
    except Exception as exc:
        print(f"CAMERA_WORKER_FAILED: {exc}", flush=True)
        with context.controller.lock:
            context.controller._safe_stop_home(f"相机任务异常：{exc}")
        context.publisher.error("CAMERA WORKER FAILED")
    finally:
        if camera is not None:
            camera.release()


def main():
    """启动网页、相机线程；网页 Start 后才会发送任何底盘或机械臂动作。"""
    import argparse

    parser = argparse.ArgumentParser(description="固定位置草莓一次抓取演示")
    parser.add_argument("--arm-port", default=DEFAULT_ARM_PORT)
    parser.add_argument("--base-port", default=DEFAULT_BASE_PORT)
    parser.add_argument("--port", type=int, default=SERVER_PORT, help="网页服务端口")
    args = parser.parse_args()

    stop_event = threading.Event()
    links = RobotLinks(args.arm_port, args.base_port)
    controller = DemoController(links)
    publisher = VideoPublisher()
    context = AppContext(controller, publisher, stop_event)

    worker = threading.Thread(target=camera_worker, args=(context,), name="demo-camera", daemon=True)
    worker.start()

    ip = get_local_ip()
    print(f"Open browser: http://{ip}:{args.port}", flush=True)
    print(f"ARM={args.arm_port} BASE={args.base_port} baud={BAUDRATE}", flush=True)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), create_handler(context))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("server stopped", flush=True)
    finally:
        stop_event.set()
        with publisher.condition:
            publisher.condition.notify_all()
        try:
            with controller.lock:
                controller._safe_stop_home("程序退出", DemoState.IDLE)
        finally:
            server.server_close()
            worker.join(timeout=2.0)
            links.close()


if __name__ == "__main__":
    main()
