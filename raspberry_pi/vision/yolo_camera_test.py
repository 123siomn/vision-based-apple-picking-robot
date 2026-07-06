from ultralytics import YOLO
import cv2
from pathlib import Path


# 项目根目录：D:\vision-based-apple-picking-robot
PROJECT_ROOT = Path(__file__).resolve().parents[2]

# 模型路径
MODEL_PATH = PROJECT_ROOT / "models" / "yolo11n.pt"


def main():
    print("MODEL_PATH =", MODEL_PATH)

    if not MODEL_PATH.exists():
        print("模型文件不存在，请确认 yolo11n.pt 是否放在 models 文件夹里")
        return

    # 加载 YOLO 模型
    model = YOLO(str(MODEL_PATH))

    # 打开笔记本摄像头
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("摄像头打开失败")
        return

    print("摄像头打开成功，按 q 退出")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("读取摄像头画面失败")
            break

        # YOLO 推理
        results = model(frame, conf=0.5, verbose=False)

        # 画检测框
        annotated_frame = results[0].plot()

        cv2.imshow("YOLO Camera Test", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()