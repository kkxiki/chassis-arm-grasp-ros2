# 相机单独测试

用于单独验证 Orbbec 相机（`192.168.30.20`）的彩色/深度取流与 YOLO 检测。

## 前提

- 相机驱动（Orbbec ROS2 驱动）已运行，发布标准话题：
  - 彩色：`/camera/color/image_raw`
  - 深度：`/camera/depth/image_raw`
- 依赖：`rclpy`、`cv_bridge`、`sensor_msgs`、`opencv-python`、`numpy`
- YOLO 检测可选，需要 `ultralytics`（虚拟机当前未装，可按需安装）

```bash
pip3 install ultralytics opencv-python numpy
```

## 1. 取流 + 存图 + 深度采样

```bash
python3 camera_capture.py --duration 10
```

- 收到第一帧彩色图后自动保存为 `captured_color.jpg`
- 每秒打印统计（彩色帧/深度帧数量），并周期采样画面中心深度
- 若一直提示"尚未收到任何图像帧"，说明驱动未启动或话题名不对，
  可先执行 `ros2 topic list | grep camera` 查看实际话题名

## 2. YOLO 单图检测

先准备好模型（如 `yolov8n.pt`），然后对刚保存的图做检测：

```bash
python3 yolo_detect.py captured_color.jpg --model yolov8n.pt --class-id 39 --save-out out.jpg
```

- `--class-id 39` 只显示瓶子类别（COCO）
- 不带 `--class-id` 显示所有类别
- 输出每个目标的类别、置信度、框坐标与画面中心像素坐标

> 提示：`camera_capture.py` 保存的彩色图分辨率即相机当前分辨率，
> 与视觉抓取配置里的 `image_width/height`（1280x720）应保持一致。
