from pathlib import Path
import re
import subprocess
import time

import cv2
import numpy as np


# 本脚本只用于测试 RGB 图像坐标到 Depth 图像坐标的像素偏移，不控制机械臂。
# 调试目标：找到让 depth helper 读到目标本体深度的 offset_x / offset_y。

RASPBERRYPI_DIR = Path(__file__).resolve().parents[1]
DEPTH_EXE = RASPBERRYPI_DIR / "depth_helper" / "depth_xyz_reader"
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
RESULT_IMAGE_PATH = OUTPUT_DIR / "depth_offset_scan.jpg"
MASK_IMAGE_PATH = OUTPUT_DIR / "depth_offset_mask.jpg"

CAMERA_PATH = "/dev/video0"
CAMERA_INDEX_LIST = list(range(10))
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
WARMUP_FRAMES = 10
MIN_TARGET_AREA = 300

# 默认运行 10 秒，每秒输出一次结果，方便观察目标不动时 Z 和 valid 是否稳定。
TEST_SECONDS = 10
SAMPLE_INTERVAL_SEC = 1.0

# ROI 半径越大，越能容忍 RGB/Depth 小偏移，但也越容易混入背景。
DEPTH_RADIUS = 5
DEPTH_FRAMES = 12
DEPTH_TIMEOUT_SEC = 4.0

# 第一版扫描范围不要太密，否则每秒内调用 depth helper 次数太多。
# 如果发现 best offset 总在边缘，例如 30 或 -30，说明需要扩大扫描范围。
OFFSET_X_LIST = [-30, -20, -10, 0, 10, 20, 30]
OFFSET_Y_LIST = [-20, -10, 0, 10, 20]


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
    """调用 C 语言 depth helper，读取指定 depth 像素附近 ROI 的深度。"""
    if not DEPTH_EXE.exists():
        return {
            "ok": False,
            "error": f"DEPTH_HELPER_NOT_FOUND path={DEPTH_EXE}",
        }

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
        return {
            "ok": False,
            "error": f"DEPTH_TIMEOUT timeout={DEPTH_TIMEOUT_SEC:.1f}s",
        }
    except OSError as exc:
        return {
            "ok": False,
            "error": f"DEPTH_LAUNCH_FAILED error={exc}",
        }

    output = (result.stdout + "\n" + result.stderr).strip()
    ok_line = find_ok_line(output)
    if ok_line is None:
        return {
            "ok": False,
            "error": f"DEPTH_FAILED returncode={result.returncode}",
        }

    x_mm = parse_value(ok_line, "x")
    y_mm = parse_value(ok_line, "y")
    z_mm = parse_value(ok_line, "z")
    valid = parse_value(ok_line, "valid", int)

    if x_mm is None or y_mm is None or z_mm is None or valid is None:
        return {
            "ok": False,
            "error": f"DEPTH_PARSE_FAILED line={ok_line}",
        }

    return {
        "ok": True,
        "x": x_mm,
        "y": y_mm,
        "z": z_mm,
        "valid": valid,
        "line": ok_line,
    }


def open_camera():
    """打开 RGB 摄像头，并设置为和 depth helper 当前测试一致的 640x480。"""
    candidates = []
    video_dir = Path("/dev")
    if video_dir.exists():
        candidates.extend(sorted(video_dir.glob("video*")))

    # 如果 /dev/video* 枚举不到，也继续尝试常见数字索引，兼容不同 OpenCV 后端。
    for index in CAMERA_INDEX_LIST:
        candidate = Path(f"/dev/video{index}")
        if candidate not in candidates:
            candidates.append(candidate)

    for candidate in candidates:
        camera = cv2.VideoCapture(str(candidate), cv2.CAP_V4L2)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            ok, _ = camera.read()
            if ok:
                print(f"RGB camera opened: {candidate}")
                return camera
            camera.release()

    for index in CAMERA_INDEX_LIST:
        camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            ok, _ = camera.read()
            if ok:
                print(f"RGB camera opened: index {index}")
                return camera
            camera.release()

    existing_devices = ", ".join(str(path) for path in sorted(video_dir.glob("video*"))) if video_dir.exists() else "none"
    raise RuntimeError(f"无法打开 RGB 摄像头，已扫描 {CAMERA_PATH} 和 0-9，当前设备：{existing_devices}")



def warmup_camera(camera):
    """丢弃前几帧，避开摄像头刚打开时自动曝光不稳定的问题。"""
    frame = None
    for _ in range(WARMUP_FRAMES):
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError("RGB 摄像头预热读取失败")
    return frame


def build_red_mask(frame):
    """在 HSV 空间提取红色目标，红色跨越 0 度，因此使用两段阈值。"""
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
    """选择面积最大的红色轮廓作为测试目标，返回目标框、质心和 mask。"""
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
        return None, mask

    x, y, w, h = cv2.boundingRect(best_contour)
    moments = cv2.moments(best_contour)
    if moments["m00"] != 0:
        # 用轮廓质心作为 RGB 目标中心，比外接矩形中心更不容易被过大的框带偏。
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
    else:
        cx = x + w // 2
        cy = y + h // 2

    return {
        "bbox": (x, y, w, h),
        "cx": cx,
        "cy": cy,
        "bbox_cx": x + w // 2,
        "bbox_cy": y + h // 2,
        "area": best_area,
    }, mask


def score_depth_result(result):
    """给扫描结果打分：第一版优先选择 valid 多、Z 有效的点。"""
    if not result["ok"]:
        return -1
    if result["z"] <= 0:
        return -1
    return result["valid"]


def scan_offsets(rgb_cx, rgb_cy):
    """围绕 RGB 中心点扫描多组 offset，寻找最可能落在目标本体上的 depth 点。"""
    records = []
    best_record = None
    best_score = -1

    for offset_y in OFFSET_Y_LIST:
        for offset_x in OFFSET_X_LIST:
            depth_cx = rgb_cx + offset_x
            depth_cy = rgb_cy + offset_y
            result = read_depth_xyz(depth_cx, depth_cy)

            record = {
                "offset_x": offset_x,
                "offset_y": offset_y,
                "depth_cx": depth_cx,
                "depth_cy": depth_cy,
                "result": result,
            }
            records.append(record)

            score = score_depth_result(result)
            if score > best_score:
                best_score = score
                best_record = record

    return records, best_record


def summarize_records(records):
    """统计扫描结果，辅助判断 BEST none 是 helper 失败还是 ROI 无有效深度。"""
    ok_count = 0
    valid_count = 0
    first_error = None

    for record in records:
        result = record["result"]
        if result["ok"]:
            ok_count += 1
            if result["valid"] > 0 and result["z"] > 0:
                valid_count += 1
        elif first_error is None:
            first_error = result.get("error", "UNKNOWN_ERROR")

    return {
        "ok_count": ok_count,
        "valid_count": valid_count,
        "total": len(records),
        "first_error": first_error,
    }


def draw_scan_result(frame, target, records, best_record, sample_index):
    """在调试图上画 RGB 中心、所有扫描点和推荐 best offset。"""
    x, y, w, h = target["bbox"]
    rgb_cx = target["cx"]
    rgb_cy = target["cy"]

    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (rgb_cx, rgb_cy), 5, (0, 255, 0), -1)
    cv2.putText(
        frame,
        "RGB",
        (rgb_cx + 6, rgb_cy - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )

    for record in records:
        point = (record["depth_cx"], record["depth_cy"])
        color = (180, 180, 180)
        if record["result"]["ok"] and record["result"]["valid"] > 0:
            color = (255, 120, 0)
        cv2.circle(frame, point, 2, color, -1)

    summary = summarize_records(records)
    text_lines = [f"sample={sample_index} ok={summary['ok_count']}/{summary['total']} valid={summary['valid_count']}"]
    if best_record is None or not best_record["result"]["ok"]:
        text_lines.append("best: none")
        if summary["first_error"]:
            text_lines.append(summary["first_error"][:42])
    else:
        result = best_record["result"]
        best_point = (best_record["depth_cx"], best_record["depth_cy"])
        cv2.circle(frame, best_point, 7, (255, 0, 0), 2)
        cv2.putText(
            frame,
            "DEPTH BEST",
            (best_point[0] + 6, best_point[1] + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
            cv2.LINE_AA,
        )
        text_lines.extend(
            [
                f"best offset=({best_record['offset_x']},{best_record['offset_y']})",
                f"Z={result['z']:.1f}mm valid={result['valid']}",
                f"X={result['x']:.1f} Y={result['y']:.1f}",
            ]
        )

    for index, text in enumerate(text_lines):
        cv2.putText(
            frame,
            text,
            (12, 28 + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def save_result_image(frame, mask=None):
    """保存最后一次扫描的调试图和红色 mask 图，便于通过 SSH 拉取查看。"""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(RESULT_IMAGE_PATH), frame)
    if mask is not None:
        cv2.imwrite(str(MASK_IMAGE_PATH), mask)


def print_scan_summary(sample_index, target, best_record):
    """每秒打印一次简要结果，方便在终端观察 offset 和深度稳定性。"""
    rgb_cx = target["cx"]
    rgb_cy = target["cy"]

    if best_record is None or not best_record["result"]["ok"]:
        print(f"[{sample_index:02d}] RGB cx={rgb_cx} cy={rgb_cy} BEST none")
        return

    result = best_record["result"]
    print(
        f"[{sample_index:02d}] "
        f"RGB cx={rgb_cx} cy={rgb_cy} "
        f"BEST offset=({best_record['offset_x']},{best_record['offset_y']}) "
        f"depth=({best_record['depth_cx']},{best_record['depth_cy']}) "
        f"X={result['x']:.1f}mm Y={result['y']:.1f}mm "
        f"Z={result['z']:.1f}mm valid={result['valid']}"
    )


def main():
    camera = open_camera()
    try:
        warmup_camera(camera)

        print("START depth offset scan")
        print(f"duration={TEST_SECONDS}s interval={SAMPLE_INTERVAL_SEC:.1f}s")
        print(f"result image: {RESULT_IMAGE_PATH.relative_to(RASPBERRYPI_DIR)}")
        print(f"mask image: {MASK_IMAGE_PATH.relative_to(RASPBERRYPI_DIR)}")

        sample_index = 0
        end_time = time.monotonic() + TEST_SECONDS
        while time.monotonic() < end_time:
            loop_start = time.monotonic()
            ok, frame = camera.read()
            if not ok:
                print(f"[{sample_index:02d}] CAMERA_READ_FAILED")
                break

            result_frame = frame.copy()
            target, mask = detect_largest_red_target(frame)
            if target is None:
                print(f"[{sample_index:02d}] NO_TARGET")
                cv2.putText(
                    result_frame,
                    "NO_TARGET",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                save_result_image(result_frame, mask)
            else:
                records, best_record = scan_offsets(target["cx"], target["cy"])
                summary = summarize_records(records)
                if best_record is None or not best_record["result"]["ok"]:
                    error = summary["first_error"] or "UNKNOWN_ERROR"
                    print(
                        f"[{sample_index:02d}] "
                        f"RGB cx={target['cx']} cy={target['cy']} "
                        f"bbox_center=({target['bbox_cx']},{target['bbox_cy']}) "
                        f"ok={summary['ok_count']}/{summary['total']} "
                        f"valid={summary['valid_count']} BEST none error={error}"
                    )
                else:
                    print_scan_summary(sample_index, target, best_record)
                draw_scan_result(result_frame, target, records, best_record, sample_index)
                save_result_image(result_frame, mask)

            sample_index += 1
            elapsed = time.monotonic() - loop_start
            sleep_time = SAMPLE_INTERVAL_SEC - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print(f"DONE saved: {RESULT_IMAGE_PATH.relative_to(RASPBERRYPI_DIR)}")
    finally:
        camera.release()


if __name__ == "__main__":
    main()
