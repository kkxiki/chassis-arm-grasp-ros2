#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 诊断：列出相机支持的流配置
from pyorbbecsdk import Context, OBStreamType, Pipeline

ctx = Context()
dev = ctx.create_net_device("192.168.30.120", 8090)
print("device:", dev.get_device_info().get_name())
pipe = Pipeline(dev)

for st in (OBStreamType.COLOR_STREAM, OBStreamType.DEPTH_STREAM, OBStreamType.IR_STREAM, OBStreamType.LEFT_IR_STREAM):
    try:
        profiles = pipe.get_stream_profile_list(st)
        n = profiles.get_count()
        print("== stream:", st, "count:", n)
        for i in range(n):
            p = profiles.get_profile_by_index(i)
            try:
                print("   ", p.get_width(), "x", p.get_height(), "@", p.get_fps(), "fps", p.get_format())
            except Exception:
                print("   ", p)
    except Exception as e:
        print("stream", st, "error:", type(e).__name__, str(e)[:120])
