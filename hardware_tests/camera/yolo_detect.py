#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 相机+YOLO 单独测试：对单张图片做目标检测
# 功能：加载 YOLOv8 模型，对图片检测，打印类别/置信度/框中心，
#       可选保存标注图。用于确认模型与目标类别(如瓶子 class 39)可用。
# 依赖：ultralytics, opencv-python, numpy
# 运行：python3 yolo_detect.py captured_color.jpg [--model yolov8n.pt]
#       python3 yolo_detect.py captured_color.jpg --class-id 39 --save-out out.jpg
# ============================================================
import argparse

import cv2
import numpy as np
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="YOLO 单图检测测试")
    parser.add_argument("image", help="输入图片路径")
    parser.add_argument("--model", default="yolov8n.pt", help="模型路径(.pt/.engine)")
    parser.add_argument("--conf", type=float, default=0.40, help="置信度阈值")
    parser.add_argument("--class-id", type=int, default=None,
                        help="只显示该类别(如 39=瓶子)")
    parser.add_argument("--save-out", default=None, help="保存标注图路径")
    parser.add_argument("--device", default="0", help="推理设备(0/cpu)")
    args = parser.parse_args()

    print(f"[YOLO] 加载模型: {args.model} (device={args.device})")
    model = YOLO(args.model)
    model.fuse()
    model.to(args.device)
    names = model.names
    print(f"[YOLO] 类别数量: {len(names)}")

    img = cv2.imread(args.image)
    if img is None:
        print(f"[YOLO] ❌ 无法读取图片: {args.image}")
        return
    h, w = img.shape[:2]
    print(f"[YOLO] 图片尺寸: {w}x{h}")

    results = model(img, conf=args.conf, device=args.device, verbose=False)
    print(f"[YOLO] 检测到 {len(results[0].boxes) if results[0].boxes is not None else 0} 个目标:")

    found = 0
    for r in results:
        boxes = r.boxes
        if boxes is None or len(boxes) == 0:
            continue
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if args.class_id is not None and cls_id != args.class_id:
                continue
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            print(f"  class={cls_id} ({names.get(cls_id, 'unknown')}) "
                  f"conf={conf:.3f} box=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) "
                  f"center=({cx:.1f},{cy:.1f})px")
            found += 1
    if found == 0:
        print("  (无目标或未匹配指定类别)")

    if args.save_out:
        annotated = results[0].plot()
        cv2.imwrite(args.save_out, annotated)
        print(f"[YOLO] 标注图已保存: {args.save_out}")


if __name__ == "__main__":
    main()
