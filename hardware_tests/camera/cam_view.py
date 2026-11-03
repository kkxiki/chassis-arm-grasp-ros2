#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Gemini 335Le 网络相机实时显示（VM :0 桌面窗口）
import sys

import cv2
import numpy as np
from pyorbbecsdk import (Config, Context, OBFormat, OBStreamType, Pipeline)


def main():
    ctx = Context()
    dev = ctx.create_net_device("192.168.30.120", 8090)
    print("device ok:", dev.get_device_info().get_name())

    config = Config()
    # 先尝试 RGB；若失败回退 MJPG
    config.enable_video_stream(OBStreamType.COLOR_STREAM, 640, 480, 30, OBFormat.RGB)
    pipe = Pipeline(dev)
    pipe.start(config)
    print("stream started")

    frame_count = 0
    while True:
        fs = pipe.wait_for_frames(500)
        if fs is None:
            continue
        cf = fs.get_color_frame()
        if cf is None:
            continue
        data = cf.get_data()
        if data is None:
            continue
        img = np.asarray(data)
        if img.ndim == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow("Gemini335Le", img)
        frame_count += 1
        if frame_count % 30 == 0:
            print("frames:", frame_count, "shape:", img.shape)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    pipe.stop()
    cv2.destroyAllWindows()
    print("done, total frames:", frame_count)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
        sys.exit(1)
