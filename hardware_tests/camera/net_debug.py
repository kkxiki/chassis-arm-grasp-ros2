#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

os.environ["OB_LOG_LEVEL"] = "debug"

from pyorbbecsdk import Context


def main():
    ctx = Context()
    try:
        dev = ctx.create_net_device("192.168.30.120", 8090)
        print("CREATE_OK:", dev.get_device_info().get_name())
    except Exception as e:
        print("CREATE_ERR:", type(e).__name__, str(e)[:300])


if __name__ == "__main__":
    main()
