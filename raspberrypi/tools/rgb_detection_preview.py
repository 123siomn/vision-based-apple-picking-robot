"""用于调整相机角度的纯 RGB 红色目标检测网页。"""

from argparse import ArgumentParser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import socket
import threading
import time

import cv2
import numpy as np


# 当前 Astra 重枚举后，RGB 主图像节点为 /dev/video1；/dev/video2 不作为 RGB 使用。
CAMERA_PATH = "/dev/video1"
CAMERA_INDEX = 1
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MIN_TARGET_AREA = 600
JPEG_QUALITY = 80
BOUNDARY = b"frame"


class VideoPublisher:
    """只保存最新 JPEG 帧，并唤醒等待画面的网页客户端。"""

    def __init__(self):
        self._condition = threading.Condition()
        self._jpeg = None
        self._sequence = 0
        self._stopped = False

    def publish(self, jpeg):
        """覆盖旧帧，避免积压过期图像导致网页延迟。"""
        with self._condition:
            self._jpeg = jpeg
            self._sequence += 1
            self._condition.notify_all()

    def wait_for_frame(self, last_sequence):
        """等待一张新图，并返回其序号和 JPEG 数据。"""
        with self._condition:
            while not self._stopped and (self._jpeg is None or self._sequence == last_sequence):
                self._condition.wait(timeout=1.0)
            if self._stopped:
                return None, None
            return self._sequence, self._jpeg

    def stop(self):
        """唤醒所有网页客户端，使其退出视频流循环。"""
        with self._condition:
            self._stopped = True
            self._condition.notify_all()


def build_red_mask(frame):
    """建立双红色 HSV 掩膜，并通过开闭运算降低噪声。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, np.array([0, 80, 50], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8))
    high_red = cv2.inRange(hsv, np.array([170, 80, 50], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
    mask = cv2.bitwise_or(low_red, high_red)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def find_largest_red_target(frame):
    """返回超过面积阈值的最大红色轮廓；没有则返回 None。"""
    mask = build_red_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [contour for contour in contours if cv2.contourArea(contour) >= MIN_TARGET_AREA]
    if not candidates:
        return None
    contour = max(candidates, key=cv2.contourArea)
    x, y, width, height = cv2.boundingRect(contour)
    return {"x": x, "y": y, "width": width, "height": height, "cx": x + width // 2, "cy": y + height // 2, "area": int(cv2.contourArea(contour))}


def draw_overlay(frame, target, status):
    """绘制图像中心线和当前 HSV 检测结果。"""
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 255), 1)
    cv2.line(frame, (0, center_y), (width, center_y), (0, 255, 255), 1)
    cv2.circle(frame, (center_x, center_y), 5, (255, 255, 0), 2)
    if target is None:
        cv2.putText(frame, status, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return frame

    x, y = target["x"], target["y"]
    target_width, target_height = target["width"], target["height"]
    cx, cy = target["cx"], target["cy"]
    cv2.rectangle(frame, (x, y), (x + target_width, y + target_height), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    cv2.putText(frame, f"TARGET cx={cx} cy={cy} area={target['area']}", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(frame, f"dx={cx - center_x} dy={cy - center_y}", (16, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    return frame


def make_error_frame(message):
    """RGB 设备不可用时，生成可在浏览器中看到的错误画面。"""
    frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.putText(frame, message, (30, FRAME_HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    return frame


def camera_worker(publisher, stop_event):
    """唯一占用 /dev/video0，并向所有网页客户端发布处理后的 RGB 帧。"""
    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not camera.isOpened():
        frame = make_error_frame(f"RGB CAMERA OPEN FAILED: {CAMERA_PATH}")
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            publisher.publish(encoded.tobytes())
        return

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"RGB camera opened: {CAMERA_PATH} index={CAMERA_INDEX}", flush=True)
    try:
        while not stop_event.is_set():
            ok, frame = camera.read()
            if not ok or frame is None:
                frame = make_error_frame("RGB CAMERA READ FAILED")
                time.sleep(0.1)
            else:
                frame = draw_overlay(frame, find_largest_red_target(frame), "NO_TARGET")
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                publisher.publish(encoded.tobytes())
    finally:
        camera.release()


def build_handler(publisher):
    """创建绑定共享最新帧发布器的 HTTP 请求处理器。"""
    class PreviewHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = b"<!doctype html><html><head><meta charset=\"utf-8\"><title>RGB Detection Preview</title><style>body{margin:0;background:#111;color:#eee;font-family:Arial,sans-serif;text-align:center}h1{font-size:24px;margin:24px 0 14px}img{width:min(96vw,960px);height:auto;border:1px solid #444}</style></head><body><h1>RGB Detection Preview</h1><img src=\"/stream.mjpg\" alt=\"RGB detection stream\"></body></html>"
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/stream.mjpg":
                self._send_stream()
                return
            if self.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return
            self.send_error(404)

        def _send_stream(self):
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            sequence = 0
            try:
                while True:
                    sequence, jpeg = publisher.wait_for_frame(sequence)
                    if jpeg is None:
                        return
                    self.wfile.write(b"--" + BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return

        def log_message(self, _format, *_args):
            return
    return PreviewHandler


def get_local_ip():
    """获取启动提示中使用的局域网地址。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "<raspberry-pi-ip>"


def main():
    """启动相机线程和浏览器预览服务。"""
    parser = ArgumentParser(description="Pure RGB HSV detection preview")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port, default: 8080")
    args = parser.parse_args()
    publisher = VideoPublisher()
    stop_event = threading.Event()
    worker = threading.Thread(target=camera_worker, args=(publisher, stop_event), daemon=True)
    worker.start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), build_handler(publisher))
    print(f"Open browser: http://{get_local_ip()}:{args.port}", flush=True)
    print("Mode: RGB only. Depth and serial ports are disabled.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        publisher.stop()
        server.shutdown()
        server.server_close()
        worker.join(timeout=2.0)
        print("server stopped", flush=True)


if __name__ == "__main__":
    main()
