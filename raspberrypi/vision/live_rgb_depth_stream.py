from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import socket
import subprocess
import threading
import time

import cv2
import numpy as np


# 实时网页调试工具：
# - RGB 画面持续刷新；
# - HSV 检测红色目标；
# - Depth 每隔一段时间读取一次目标中心点深度；
# - 通过浏览器显示叠加后的调试画面。

RASPBERRYPI_DIR = Path(__file__).resolve().parents[1]
DEPTH_EXE = RASPBERRYPI_DIR / "depth_helper" / "depth_xyz_reader"

CAMERA_PATH = "/dev/video0"
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SERVER_PORT = 8080

MIN_TARGET_AREA = 300
JPEG_QUALITY = 80

DEPTH_OFFSET_X = 0
DEPTH_OFFSET_Y = 0
DEPTH_RADIUS = 5
DEPTH_FRAMES = 4
DEPTH_TIMEOUT_SEC = 2.0
DEPTH_INTERVAL_SEC = 1.0

CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2
LEFT_LIMIT_X = 260
RIGHT_LIMIT_X = 380


latest_depth = None
LATEST_DEPTH_LOCK = threading.Lock()

DEPTH_REQUEST_CONDITION = threading.Condition()
pending_depth_point = None
last_depth_request_time = 0.0

FRAME_CONDITION = threading.Condition()
latest_jpeg = None
latest_frame_id = 0

STOP_EVENT = threading.Event()


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


def read_depth_xyz(cx, cy):
    """调用 depth helper 读取指定像素附近的深度和相机坐标。"""
    if not DEPTH_EXE.exists():
        return {
            "ok": False,
            "error": "DEPTH_HELPER_NOT_FOUND",
        }

    cmd = [
        str(DEPTH_EXE),
        str(cx),
        str(cy),
        str(DEPTH_RADIUS),
        str(DEPTH_FRAMES),
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
            "error": "DEPTH_TIMEOUT",
        }
    except OSError as exc:
        return {
            "ok": False,
            "error": f"DEPTH_LAUNCH_FAILED {exc}",
        }

    output = (result.stdout + "\n" + result.stderr).strip()
    ok_line = find_ok_line(output)
    if ok_line is None:
        return {
            "ok": False,
            "error": f"DEPTH_FAILED {result.returncode}",
        }

    x_mm = parse_value(ok_line, "x")
    y_mm = parse_value(ok_line, "y")
    z_mm = parse_value(ok_line, "z")
    valid = parse_value(ok_line, "valid", int)

    if x_mm is None or y_mm is None or z_mm is None or valid is None:
        return {
            "ok": False,
            "error": "DEPTH_PARSE_FAILED",
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
    """固定打开 Astra RGB 摄像头，优先尝试 /dev/video0，失败后退回 index 0。"""
    camera = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
    if not camera.isOpened():
        # OpenCV 5 在部分树莓派环境中不支持 V4L2 按路径打开，但支持按 index 打开。
        # 这里仍然固定 index=0，不扫描 /dev/video1，避免误打开 depth/metadata 节点。
        camera.release()
        camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开 RGB 摄像头：{CAMERA_PATH} index={CAMERA_INDEX}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    ok, frame = camera.read()
    if not ok or frame is None:
        camera.release()
        raise RuntimeError(f"RGB 摄像头读取失败：{CAMERA_PATH} index={CAMERA_INDEX}")

    print(f"RGB camera opened: {CAMERA_PATH} index={CAMERA_INDEX}")
    return camera


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
    """选择面积最大的红色轮廓作为当前调试目标。"""
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
    moments = cv2.moments(best_contour)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
    else:
        cx = x + w // 2
        cy = y + h // 2

    return {
        "bbox": (x, y, w, h),
        "cx": cx,
        "cy": cy,
        "area": best_area,
    }


def choose_action(target):
    """根据目标中心点给出第一版动作建议。"""
    if target is None:
        return "NO_TARGET"

    cx = target["cx"]
    if cx < LEFT_LIMIT_X:
        return "LEFT_GRAB"
    if cx > RIGHT_LIMIT_X:
        return "RIGHT_GRAB"
    return "CENTER_GRAB"


def choose_base_hint(target, depth):
    """根据 cx 和 Z 给出底盘调试建议，只用于人工观察。"""
    if target is None:
        return "SEARCH_TARGET"

    cx = target["cx"]
    if cx < LEFT_LIMIT_X:
        return "TURN_LEFT"
    if cx > RIGHT_LIMIT_X:
        return "TURN_RIGHT"

    if depth is not None and depth.get("ok"):
        z_mm = depth["z"]
        if z_mm > 850:
            return "SLOW_FORWARD"
        if z_mm < 600:
            return "STOP_TOO_CLOSE"
        return "STOP_READY"

    return "CENTER_NO_DEPTH"


def request_depth_if_needed(target):
    """按固定周期提交深度请求，并立即返回最近一次结果。"""
    global latest_depth
    global pending_depth_point
    global last_depth_request_time

    if target is None:
        with LATEST_DEPTH_LOCK:
            latest_depth = None
        with DEPTH_REQUEST_CONDITION:
            pending_depth_point = None
        return None

    now = time.monotonic()
    depth_cx = target["cx"] + DEPTH_OFFSET_X
    depth_cy = target["cy"] + DEPTH_OFFSET_Y
    if not (0 <= depth_cx < FRAME_WIDTH and 0 <= depth_cy < FRAME_HEIGHT):
        with LATEST_DEPTH_LOCK:
            latest_depth = {
                "ok": False,
                "error": "DEPTH_POINT_OUT_OF_RANGE",
            }
            return latest_depth

    with DEPTH_REQUEST_CONDITION:
        if now - last_depth_request_time >= DEPTH_INTERVAL_SEC:
            pending_depth_point = (depth_cx, depth_cy)
            last_depth_request_time = now
            DEPTH_REQUEST_CONDITION.notify()

    with LATEST_DEPTH_LOCK:
        return latest_depth


def depth_worker():
    """后台执行 depth helper，避免深度读取阻塞 RGB 视频帧。"""
    global latest_depth
    global pending_depth_point

    while not STOP_EVENT.is_set():
        with DEPTH_REQUEST_CONDITION:
            DEPTH_REQUEST_CONDITION.wait_for(
                lambda: pending_depth_point is not None or STOP_EVENT.is_set(),
                timeout=0.5,
            )
            if STOP_EVENT.is_set():
                return
            if pending_depth_point is None:
                continue
            depth_cx, depth_cy = pending_depth_point
            pending_depth_point = None

        result = read_depth_xyz(depth_cx, depth_cy)
        result["cx"] = depth_cx
        result["cy"] = depth_cy
        with LATEST_DEPTH_LOCK:
            latest_depth = result


def draw_overlay(frame, target, depth):
    """绘制中心线、目标框、深度和动作建议。"""
    action = choose_action(target)
    base_hint = choose_base_hint(target, depth)

    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (0, 255, 255), 1)
    cv2.line(frame, (0, CENTER_Y), (FRAME_WIDTH, CENTER_Y), (0, 255, 255), 1)
    cv2.line(frame, (LEFT_LIMIT_X, 0), (LEFT_LIMIT_X, FRAME_HEIGHT), (255, 180, 0), 1)
    cv2.line(frame, (RIGHT_LIMIT_X, 0), (RIGHT_LIMIT_X, FRAME_HEIGHT), (255, 180, 0), 1)

    text_lines = []
    if target is None:
        text_lines.append("NO_TARGET")
        text_lines.append(f"action={action}")
        text_lines.append(f"base={base_hint}")
    else:
        x, y, w, h = target["bbox"]
        cx = target["cx"]
        cy = target["cy"]
        error_x = cx - CENTER_X
        error_y = cy - CENTER_Y

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.circle(frame, (cx + DEPTH_OFFSET_X, cy + DEPTH_OFFSET_Y), 7, (255, 0, 0), 2)

        text_lines.append(f"cx={cx} cy={cy}")
        text_lines.append(f"err_x={error_x} err_y={error_y}")

        if depth is not None and depth.get("ok"):
            text_lines.append(
                f"Z={depth['z']:.1f}mm X={depth['x']:.1f} Y={depth['y']:.1f} valid={depth['valid']}"
            )
        elif depth is not None:
            text_lines.append(f"Z=-- {depth.get('error', 'DEPTH_FAILED')}")
        else:
            text_lines.append("Z=waiting")

        text_lines.append(f"action={action}")
        text_lines.append(f"base={base_hint}")

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

    return frame


def get_local_ip():
    """获取树莓派当前局域网 IP，用于提示浏览器访问地址。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def publish_frame(frame):
    """编码并发布最新 JPEG，供所有浏览器连接共享。"""
    global latest_jpeg
    global latest_frame_id

    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not ok:
        return

    with FRAME_CONDITION:
        latest_jpeg = encoded.tobytes()
        latest_frame_id += 1
        FRAME_CONDITION.notify_all()


def publish_error_frame(message):
    """摄像头线程异常时发布提示帧，避免网页只显示破图。"""
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
    publish_frame(frame)


def camera_worker():
    """独占摄像头并持续发布处理后的 RGB 帧。"""
    camera = None
    try:
        # 摄像头必须在同一个线程中完成打开、读取和释放。
        # 有些 V4L2/OpenCV 组合把 VideoCapture 从主线程交给子线程后，
        # 会在读取时重新申请缓冲区并失败，表现为 VIDIOC_REQBUFS errno=19。
        camera = open_camera()
        while not STOP_EVENT.is_set():
            ok, frame = camera.read()
            if not ok or frame is None:
                publish_error_frame("RGB CAMERA READ FAILED")
                return

            target = detect_largest_red_target(frame)
            depth = request_depth_if_needed(target)
            result_frame = draw_overlay(frame, target, depth)
            publish_frame(result_frame)
    except Exception as exc:
        print(f"CAMERA_WORKER_FAILED: {exc}", flush=True)
        publish_error_frame("CAMERA WORKER FAILED")
    finally:
        if camera is not None:
            camera.release()


def frame_generator():
    """将采集线程发布的最新 JPEG 转换为 MJPEG 数据流。"""
    seen_frame_id = -1
    while not STOP_EVENT.is_set():
        with FRAME_CONDITION:
            FRAME_CONDITION.wait_for(
                lambda: latest_frame_id != seen_frame_id or STOP_EVENT.is_set(),
                timeout=2.0,
            )
            if STOP_EVENT.is_set():
                return
            if latest_jpeg is None or latest_frame_id == seen_frame_id:
                continue
            frame = latest_jpeg
            seen_frame_id = latest_frame_id

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            frame +
            b"\r\n"
        )


class StreamHandler(BaseHTTPRequestHandler):
    """HTTP 处理器：根页面显示视频流，/stream.mjpg 输出 MJPEG。"""

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            html = (
                "<!doctype html><html><head><meta charset='utf-8'>"
                "<title>RGB Depth Live Debug</title>"
                "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;}"
                "main{display:flex;flex-direction:column;align-items:center;padding:16px;}"
                "img{max-width:100%;height:auto;border:1px solid #444;}</style>"
                "</head><body><main>"
                "<h2>RGB Depth Live Debug</h2>"
                "<img src='/stream.mjpg' alt='live stream'>"
                "</main></body></html>"
            )
            self.wfile.write(html.encode("utf-8"))
            return

        if self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            try:
                for frame in frame_generator():
                    self.wfile.write(frame)
            except (BrokenPipeError, ConnectionResetError):
                return
            return

        self.send_error(404)

    def log_message(self, format_text, *args):
        """减少浏览器持续拉流时的终端刷屏。"""
        return


def main():
    camera_thread = threading.Thread(
        target=camera_worker,
        name="rgb-camera",
        daemon=True,
    )
    depth_thread = threading.Thread(
        target=depth_worker,
        name="depth-reader",
        daemon=True,
    )
    camera_thread.start()
    depth_thread.start()

    ip = get_local_ip()
    print(f"Open browser: http://{ip}:{SERVER_PORT}")
    server = ThreadingHTTPServer(("0.0.0.0", SERVER_PORT), StreamHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("server stopped")
    finally:
        STOP_EVENT.set()
        with FRAME_CONDITION:
            FRAME_CONDITION.notify_all()
        with DEPTH_REQUEST_CONDITION:
            DEPTH_REQUEST_CONDITION.notify_all()
        server.server_close()
        camera_thread.join(timeout=2.0)
        depth_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
