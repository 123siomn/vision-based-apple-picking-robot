from pathlib import Path
import re
import subprocess

import cv2
import numpy as np


# 根据当前文件位置定位 raspberrypi 目录，保证从项目根目录或 raspberrypi 目录运行都能找到 depth helper。
RASPBERRYPI_DIR = Path(__file__).resolve().parents[1]
DEPTH_EXE = RASPBERRYPI_DIR / "depth_helper" / "depth_xyz_reader"
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
RAW_IMAGE_PATH = OUTPUT_DIR / "rgb_raw.jpg"
RESULT_IMAGE_PATH = OUTPUT_DIR / "hsv_depth_result.jpg"

CAMERA_PATH = "/dev/video0"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
WARMUP_FRAMES = 10
MIN_TARGET_AREA = 300
DEPTH_RADIUS = 5
DEPTH_FRAMES = 20
DEPTH_TIMEOUT_SEC = 5.0


def parse_value(line, name, value_type=float):
    """从 depth helper 的 OK 行中解析 x/y/z/valid 等字段。"""
    pattern = rf"(?:^|\s){name}=([-0-9.]+)"
    match = re.search(pattern, line)
    if match is None:
        return None
    return value_type(match.group(1))


def find_ok_line(output):
    """Astra SDK 可能输出 warning，因此只从多行输出中寻找真正的 OK 结果行。"""
    for line in output.splitlines():
        if line.startswith("OK "):
            return line
    return None


def read_depth_xyz(cx, cy, radius=DEPTH_RADIUS, frames=DEPTH_FRAMES):
    """调用 C 语言 depth helper，读取目标像素附近的深度和相机坐标。"""
    if not DEPTH_EXE.exists():
        print(f"DEPTH_HELPER_NOT_FOUND path={DEPTH_EXE}")
        return None

    cmd = [
        str(DEPTH_EXE),
        str(cx),
        str(cy),
        str(radius),
        str(frames),
    ]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=DEPTH_TIMEOUT_SEC,
            check=False,
        )
    except subprocess.TimeoutExpired:
        print(f"DEPTH_TIMEOUT timeout={DEPTH_TIMEOUT_SEC:.1f}s")
        return None
    except OSError as exc:
        print(f"DEPTH_LAUNCH_FAILED error={exc}")
        return None

    output = (result.stdout + "\n" + result.stderr).strip()
    ok_line = find_ok_line(output)
    if ok_line is None:
        print(f"DEPTH_FAILED returncode={result.returncode}")
        if output:
            print(output)
        return None

    x_mm = parse_value(ok_line, "x")
    y_mm = parse_value(ok_line, "y")
    z_mm = parse_value(ok_line, "z")
    valid = parse_value(ok_line, "valid", int)

    if x_mm is None or y_mm is None or z_mm is None or valid is None:
        print(f"DEPTH_PARSE_FAILED line={ok_line}")
        return None

    return {
        "x": x_mm,
        "y": y_mm,
        "z": z_mm,
        "valid": valid,
        "line": ok_line,
    }


def open_camera():
    """打开 /dev/video0 并设置第一版联调用的 640x480 分辨率。"""
    camera = cv2.VideoCapture(CAMERA_PATH)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开 RGB 摄像头：{CAMERA_PATH}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    return camera


def capture_test_frame(camera):
    """丢弃前几帧自动曝光不稳定的画面，并返回最后一帧用于检测。"""
    frame = None
    for _ in range(WARMUP_FRAMES + 1):
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError("读取 RGB 摄像头画面失败")
    return frame


def build_red_mask(frame):
    """在 HSV 空间提取红色区域，红色跨越 0 度，因此需要低红和高红两段范围。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 80, 50], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([170, 80, 50], dtype=np.uint8)
    upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)

    mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask_1, mask_2)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_largest_red_target(frame):
    """选择面积最大的红色轮廓作为第一版测试目标。"""
    mask = build_red_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_TARGET_AREA:
            continue
        if area > best_area:
            best_area = area
            best_contour = contour

    if best_contour is None:
        return None

    x, y, w, h = cv2.boundingRect(best_contour)
    cx = x + w // 2
    cy = y + h // 2
    return {
        "bbox": (x, y, w, h),
        "cx": cx,
        "cy": cy,
        "area": best_area,
    }


def draw_no_target(frame):
    """没有检测到目标时仍保存结果图，便于 SSH 调试查看画面状态。"""
    cv2.putText(
        frame,
        "NO_TARGET",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )


def draw_target_result(frame, target, depth):
    """把检测框、中心点和 depth helper 返回的相机坐标画到调试图上。"""
    x, y, w, h = target["bbox"]
    cx = target["cx"]
    cy = target["cy"]

    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

    if depth is None:
        text_lines = [
            f"cx={cx} cy={cy}",
            "depth failed",
        ]
    else:
        text_lines = [
            f"cx={cx} cy={cy}",
            f"X={depth['x']:.1f} Y={depth['y']:.1f} Z={depth['z']:.1f} mm",
            f"valid={depth['valid']}",
        ]

    text_x = max(5, x)
    text_y = max(25, y - 45)
    for index, text in enumerate(text_lines):
        cv2.putText(
            frame,
            text,
            (text_x, text_y + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def save_images(raw_frame, result_frame):
    """保存原始 RGB 图和叠加结果图。"""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(RAW_IMAGE_PATH), raw_frame)
    cv2.imwrite(str(RESULT_IMAGE_PATH), result_frame)


def main():
    camera = open_camera()
    try:
        frame = capture_test_frame(camera)
    finally:
        camera.release()

    raw_frame = frame.copy()
    result_frame = frame.copy()

    target = detect_largest_red_target(frame)
    if target is None:
        print("NO_TARGET")
        draw_no_target(result_frame)
        save_images(raw_frame, result_frame)
        print(f"saved: {RESULT_IMAGE_PATH.relative_to(RASPBERRYPI_DIR)}")
        return

    cx = target["cx"]
    cy = target["cy"]
    bbox = target["bbox"]
    print(f"TARGET cx={cx} cy={cy} bbox={bbox}")

    depth = read_depth_xyz(cx, cy)
    if depth is not None:
        print(
            f"DEPTH X={depth['x']:.1f}mm "
            f"Y={depth['y']:.1f}mm "
            f"Z={depth['z']:.1f}mm "
            f"valid={depth['valid']}"
        )
    else:
        print("DEPTH_FAILED")

    draw_target_result(result_frame, target, depth)
    save_images(raw_frame, result_frame)
    print(f"saved: {RESULT_IMAGE_PATH.relative_to(RASPBERRYPI_DIR)}")


if __name__ == "__main__":
    main()
