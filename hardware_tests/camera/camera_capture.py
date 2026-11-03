#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 相机单独测试：取流 + 存图 + 深度采样
# 功能：
#   1. 订阅相机彩色图像话题，保存一帧到 captured_color.jpg；
#   2. 订阅深度图像话题，周期性采样画面中心的有效深度；
#   3. 打印相机相关话题列表，方便确认驱动是否正常运行。
# 依赖：rclpy, cv_bridge, sensor_msgs, opencv-python, numpy
# 运行：python3 camera_capture.py [--duration 10]
# ============================================================
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraCapture(Node):
    def __init__(self, color_topic, depth_topic, save_path, duration_s):
        super().__init__("camera_capture")
        self.bridge = CvBridge()
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.save_path = save_path
        self.duration_s = duration_s

        # 状态统计
        self.color_frames = 0
        self.depth_frames = 0
        self.color_saved = False
        self.depth_sample_count = 0

        # 话题列表
        topics = self.get_topic_names_and_types()
        self.get_logger().info("相机相关话题:")
        for t, types in sorted(topics):
            if "camera" in t.lower():
                self.get_logger().info(f"  {t}  {types}")

        # 订阅彩色图与深度图
        self.color_sub = self.create_subscription(
            Image, color_topic, self.color_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_cb, 10)

        # 每秒打印一次状态
        self.create_timer(1.0, self.status_timer)
        # 到时自动退出
        self.create_timer(duration_s, self.done_timer)

    def color_cb(self, msg: Image):
        self.color_frames += 1
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"彩色图转换失败: {e}")
            return
        if not self.color_saved:
            cv2.imwrite(self.save_path, img)
            h, w = img.shape[:2]
            self.color_saved = True
            self.get_logger().info(
                f"✅ 已保存彩色帧: {self.save_path} ({w}x{h})")

    def depth_cb(self, msg: Image):
        self.depth_frames += 1
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"深度图转换失败: {e}")
            return
        if img.ndim != 2:
            self.get_logger().warn(f"深度图维度异常: {img.shape}")
            return
        h, w = img.shape
        # 采样中心 9x9 区域的有效深度（取中位数）
        roi = img[h // 2 - 4:h // 2 + 5, w // 2 - 4:w // 2 + 5]
        values = roi.astype(np.float64).reshape(-1)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            self.get_logger().info(f"[深度] 中心区域无有效值 (shape={img.shape}, dtype={img.dtype})")
            return
        # uint16 一般按 0.001 换算成米；float 类型直接视为米
        if np.issubdtype(img.dtype, np.integer):
            depth_m = float(np.median(values)) * 0.001
        else:
            depth_m = float(np.median(values))
        self.depth_sample_count += 1
        if self.depth_sample_count % 5 == 1:
            self.get_logger().info(
                f"[深度] 中心深度 = {depth_m:.3f} m (shape={w}x{h})")

    def status_timer(self):
        if self.color_frames == 0 and self.depth_frames == 0:
            self.get_logger().warn(
                f"⚠️ 尚未收到任何图像帧，请确认相机驱动已启动且话题正确")
            return
        self.get_logger().info(
            f"[统计] 彩色帧={self.color_frames} 深度帧={self.depth_frames}")

    def done_timer(self):
        self.get_logger().info(
            f"测试结束: 彩色帧={self.color_frames} 深度帧={self.depth_frames} "
            f"存图={self.color_saved} 深度采样={self.depth_sample_count}次")
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="相机取流/存图/深度采样测试")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--save-path", default="captured_color.jpg")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = CameraCapture(args.color_topic, args.depth_topic,
                         args.save_path, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("手动中断，退出")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
