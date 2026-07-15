import struct
import time

import cv2
import serial
from ultralytics import YOLO


# ------------------------- 可修改参数 -------------------------
MODEL_PATH = r"D:\vision-based-apple-picking-robot\models\yolo11n.pt"
SERIAL_PORT = "COM7"
BAUD_RATE = 115200
CAMERA_INDEX = 0

CONFIRM_FRAMES = 5       # 连续五帧为同一目标后进入锁定状态
SAME_TARGET_IOU = 0.3    # 相邻帧框的 IoU 不低于此值，视为同一目标
SEND_INTERVAL = 0.1      # 串口发送周期，0.1 秒即 10 Hz
MIN_CONFIDENCE = 0.7

FRAME_HEADER = b"\x55\x55"
FRAME_LENGTH = 0x0D
CMD_YOLO_TARGET = 0x30
STATUS_NO_TARGET = 0
STATUS_FOUND = 1
STATUS_LOCKED = 2
NO_CLASS = 0xFF


def limit_uint16(value):
    """把数值限制在无符号 16 位整数范围内。"""
    return max(0, min(int(value), 0xFFFF))


def build_frame(status, class_id, confidence, center_x, center_y, box_h, box_w):
    """
    构造 15 字节数据帧，与机械臂 USART1 的 55 55 协议兼容。

    55 55 | 长度1 | 命令1 | 状态1 | 类别1 | 置信度1 |
    中心X2 | 中心Y2 | 框高2 | 框宽2
    多字节字段使用小端序，不包含校验码和帧尾。
    """
    status = max(0, min(int(status), 0xFF))
    class_id = max(0, min(int(class_id), 0xFF))
    confidence_byte = max(0, min(round(float(confidence) * 100), 100))

    return struct.pack(
        "<2sBBBBBHHHH",
        FRAME_HEADER,
        FRAME_LENGTH,
        CMD_YOLO_TARGET,
        status,
        class_id,
        confidence_byte,
        limit_uint16(center_x),
        limit_uint16(center_y),
        limit_uint16(box_h),
        limit_uint16(box_w),
    )


def calculate_iou(box_a, box_b):
    """计算两个 xyxy 检测框的交并比，用来判断相邻帧是否为同一目标。"""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    intersection_w = max(0, min(ax2, bx2) - max(ax1, bx1))
    intersection_h = max(0, min(ay2, by2) - max(ay1, by1))
    intersection = intersection_w * intersection_h

    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def print_sent_frame(data):
    """以十六进制打印已经发送的数据，方便和下位机联调。"""
    print("TX:", " ".join(f"{byte:02X}" for byte in data))


model = YOLO(r"D:\python project\YOLO project\models\yolo11n.pt")
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    raise RuntimeError("无法打开摄像头，请检查摄像头权限或占用情况。")

try:
    uart = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception:
    camera.release()
    raise

same_target_frames = 0
previous_box = None
last_send_time = 0.0

print(f"摄像头已打开，串口 {SERIAL_PORT} @ {BAUD_RATE} 已打开，按 q 退出。")

try:
    while True:
        success, frame = camera.read()
        if not success:
            print("无法读取摄像头画面。")
            break

        # 类别 0 是 COCO 数据集中的 person；conf 在模型端过滤低置信度结果
        result = model(
            frame,
            classes=[0],
            conf=MIN_CONFIDENCE,
            verbose=False,
        )[0]
        boxes = result.boxes

        if len(boxes) > 0:
            # 当前帧只选择置信度最高的人
            best_index = int(boxes.conf.argmax())
            best_box = boxes[best_index]

            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            confidence = float(best_box.conf[0])
            class_id = int(best_box.cls[0])
            class_name = model.names[class_id]
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            box_w = x2 - x1
            box_h = y2 - y1
            current_box = (x1, y1, x2, y2)

            # 最高置信度框必须与上一帧框重叠，才累计为同一个目标。
            if previous_box is not None and calculate_iou(previous_box, current_box) >= SAME_TARGET_IOU:
                same_target_frames += 1
            else:
                same_target_frames = 1
            previous_box = current_box

            if same_target_frames >= CONFIRM_FRAMES:
                status = STATUS_LOCKED
                state_text = "LOCKED"
                color = (0, 255, 0)
            else:
                status = STATUS_FOUND
                state_text = f"FOUND {same_target_frames}/{CONFIRM_FRAMES}"
                color = (0, 255, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

            label = (
                f"{class_name} center=({center_x},{center_y}) "
                f"conf={confidence:.2f} {state_text}"
            )
            text_y = y1 - 10 if y1 > 30 else y1 + 25
            cv2.putText(
                frame,
                label,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

            # 发现和锁定状态均发送；只有连续五帧同一目标时状态才为锁定。
            now = time.monotonic()
            if now - last_send_time >= SEND_INTERVAL:
                data = build_frame(
                    status,
                    class_id,
                    confidence,
                    center_x,
                    center_y,
                    box_h,
                    box_w,
                )
                uart.write(data)
                print_sent_frame(data)
                last_send_time = now
        else:
            # 没有达到 0.7 置信度的目标时，立即清除连续帧记录。
            same_target_frames = 0
            previous_box = None

            now = time.monotonic()
            if now - last_send_time >= SEND_INTERVAL:
                data = build_frame(STATUS_NO_TARGET, NO_CLASS, 0, 0, 0, 0, 0)
                uart.write(data)
                print_sent_frame(data)
                last_send_time = now

        cv2.imshow("YOLO Person Serial Detection - Press q to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    camera.release()
    uart.close()
    cv2.destroyAllWindows()
