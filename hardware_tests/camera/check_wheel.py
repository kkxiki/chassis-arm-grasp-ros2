#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 检查 wheel 内的二进制平台文件（.so=Linux, .dylib=macOS）
import sys
import zipfile

whl = sys.argv[1] if len(sys.argv) > 1 else "/tmp/tuna.whl"
z = zipfile.ZipFile(whl)
print("== wheel files ==")
for n in z.namelist():
    if n.endswith((".so", ".dylib", ".pyd")):
        print(" ", n)
print("== total entries ==", len(z.namelist()))
