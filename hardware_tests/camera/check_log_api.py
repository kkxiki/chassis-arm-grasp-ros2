#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyorbbecsdk

print("== log related APIs ==")
for m in dir(pyorbbecsdk):
    if "log" in m.lower() or "set" in m.lower():
        print("  ", m)
print("== module-level functions ==")
for m in dir(pyorbbecsdk):
    if not m.startswith("_") and not hasattr(getattr(pyorbbecsdk, m), "__dict__") and callable(getattr(pyorbbecsdk, m, None)):
        print("  fn:", m)
