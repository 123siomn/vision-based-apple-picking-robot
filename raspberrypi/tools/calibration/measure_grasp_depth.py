"""测量固定抓取位置的草莓图像坐标与深度。"""

from pathlib import Path
import re
import subprocess
import time

import cv2
import numpy as np


# 本脚本必须在主控网页停止后运行，避免两个程序同时占用 /dev/video0。
RASPBERRYPI_DIR = Path(__file__).resolve().parents[2]
DEPTH_EXE = RASPBERRYPI_DIR / "depth_helper" / "depth_xyz_reader"
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
RAW_IMAGE_PATH = OUTPUT_DIR / "grasp_depth_raw.jpg"
RESULT_IMAGE_PATH = OUTPUT_DIR / "grasp_depth_result.jpg"

CAMERA_PATH = "/dev/video0"
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
WARMUP_FRAMES = 10
MIN_TARGET_AREA = 300

# helper 在同一次 SDK 会话中读取 10 帧，并对结果取中位数。
DEPTH_RADIUS = 5
DEPTH_FRAMES = 10
DEPTH_TIMEOUT_SEC = 12.0


def find_ok_line(output):
    """从 SDK warning 与程序输出中找出真正的深度结果行。"""
    for line in output.splitlines():
        if line.startswith("OK "):
            return line
    return None


def parse_value(line, name, value_type=float):
    """解析 OK 行中的指定数值字段。"""
    match = re.search(rf"(?:^|\s){name}=([-0-9.]+)", line)
    if match is None:
        return None
    return value_type(match.group(1))


def open_rgb_camera():
    """固定打开 Astra RGB 的 /dev/video0 对应编号。"""
    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开 RGB 摄像头：{CAMERA_PATH} index={CAMERA_INDEX}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    return camera


def capture_rgb_frame():
    """预热 RGB 摄像头并取得一张用于 HSV 检测的稳定画面。"""
    camera = open_rgb_camera()
    try:
        frame = None
        for _ in range(WARMUP_FRAMES + 1):
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("RGB 摄像头读取失败")
        return frame
    finally:
        # 深度 helper 启动前必须释放 RGB 的 V4L2 设备。
        camera.release()


def build_red_mask(frame):
    """在 HSV 空间提取红色草莓区域，并用开闭运算降低噪声。"""
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
    """返回最大有效红色轮廓的边界框与中心点。"""
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
    return {
        "bbox": (x, y, width, height),
        "cx": x + width // 2,
        "cy": y + height // 2,
        "area": area,
    }


def read_depth_xyz(cx, cy):
    """在 RGB 设备已释放的前提下，读取目标 ROI 的多帧中位数深度。"""
    if not DEPTH_EXE.exists():
        raise RuntimeError(f"未找到 depth helper：{DEPTH_EXE}")

    try:
        result = subprocess.run(
            [str(DEPTH_EXE), str(cx), str(cy), str(DEPTH_RADIUS), str(DEPTH_FRAMES)],
            capture_output=True,
            text=True,
            timeout=DEPTH_TIMEOUT_SEC,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"深度读取超时：{DEPTH_TIMEOUT_SEC:.1f}s") from exc

    output = (result.stdout + "\n" + result.stderr).strip()
    ok_line = find_ok_line(output)
    if ok_line is None:
        raise RuntimeError(f"深度读取失败 returncode={result.returncode}\n{output}")

    values = {
        "x": parse_value(ok_line, "x"),
        "y": parse_value(ok_line, "y"),
        "z": parse_value(ok_line, "z"),
        "valid": parse_value(ok_line, "valid", int),
    }
    if any(value is None for value in values.values()):
        raise RuntimeError(f"无法解析深度结果：{ok_line}")
    return values


def draw_result(frame, target, depth):
    """将目标框、目标中心和测得深度写入结果图。"""
    x, y, width, height = target["bbox"]
    cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
    cv2.circle(frame, (target["cx"], target["cy"]), 5, (0, 0, 255), -1)

    lines = [
        f"cx={target['cx']} cy={target['cy']} area={target['area']:.0f}",
        f"X={depth['x']:.1f}mm Y={depth['y']:.1f}mm Z={depth['z']:.1f}mm",
        f"valid={depth['valid']}",
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            frame,
            text,
            (12, 28 + index * 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main():
    """完成一次 RGB 检测与 Depth 测量，并保存用于标定的结果图。"""
    print("START grasp depth measurement")
    frame = capture_rgb_frame()
    raw_frame = frame.copy()
    target = detect_largest_red_target(frame)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(RAW_IMAGE_PATH), raw_frame)

    if target is None:
        cv2.putText(frame, "NO_TARGET", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imwrite(str(RESULT_IMAGE_PATH), frame)
        print("NO_TARGET")
        return

    print(
        f"TARGET cx={target['cx']} cy={target['cy']} "
        f"bbox={target['bbox']} area={target['area']:.0f}"
    )
    # 给 V4L2 一点时间完全释放 RGB 接口，再启动 Astra SDK 的 depth stream。
    time.sleep(0.5)
    depth = read_depth_xyz(target["cx"], target["cy"])
    print(
        f"DEPTH X={depth['x']:.1f}mm Y={depth['y']:.1f}mm "
        f"Z={depth['z']:.1f}mm valid={depth['valid']}"
    )

    draw_result(frame, target, depth)
    cv2.imwrite(str(RESULT_IMAGE_PATH), frame)
    print(f"saved: {RESULT_IMAGE_PATH.relative_to(RASPBERRYPI_DIR)}")


if __name__ == "__main__":
    main()
