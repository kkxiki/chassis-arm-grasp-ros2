#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 探测 pyorbbecsdk2 取流相关 API
import pyorbbecsdk

print("== OBStreamType ==")
print([m for m in dir(pyorbbecsdk.OBStreamType) if not m.startswith("_")])
print("== OBFormat ==")
print([m for m in dir(pyorbbecsdk.OBFormat) if not m.startswith("_")][:30])
print("== Pipeline ==")
print([m for m in dir(pyorbbecsdk.Pipeline) if not m.startswith("_")][:30])
print("== Config ==")
print([m for m in dir(pyorbbecsdk.Config) if not m.startswith("_")][:30])
print("== FrameSet ==")
print([m for m in dir(pyorbbecsdk.FrameSet) if not m.startswith("_")][:25])
print("== ColorFrame ==")
print([m for m in dir(pyorbbecsdk.ColorFrame) if not m.startswith("_")][:25])
