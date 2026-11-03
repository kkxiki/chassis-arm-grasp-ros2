#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 因时 Inspire 4B4C 夹爪单独测试（串口协议）
# 功能：打开串口 → 清错 → 松开/抓取/移动到指定开口 → 查询开口/状态/限位
# 依赖：pyserial（本目录 requirements.txt）
# 运行：python3 gripper_test.py [串口名，默认 /dev/ttyUSB0]
# ============================================================
import sys

from inspire_4b4c import Inspire4B4C, ProtocolError

DEFAULT_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
GRIPPER_ID = 1


def show_help():
    print("\n========== 夹爪测试命令 ==========")
    print("  open [port]         打开串口(默认 /dev/ttyUSB0)")
    print("  clear               清除错误")
    print("  release speed       松开到最大(速度1~1000)")
    print("  grasp speed force [hold]  抓取(速度1~1000, 力50~1000, hold=保持)")
    print("  move opening        移动到指定开口(0~1000)")
    print("  opening             查询当前开口")
    print("  status              查询运行状态/错误码/温度")
    print("  limits              查询开口限位(max, min)")
    print("  setlimits max min   设置开口限位")
    print("  stop                急停")
    print("  h / help            帮助")
    print("  q / quit            退出")
    print("===================================\n")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    gripper = None
    print(f"\n========== 因时 4B4C 夹爪单独测试 ==========")
    print(f"串口: {port}  波特率: {BAUDRATE}  夹爪ID: {GRIPPER_ID}")
    show_help()

    while True:
        try:
            line = input("夹爪> ").strip()
        except EOFError:
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("open", "connect"):
                p = parts[1] if len(parts) > 1 else port
                gripper = Inspire4B4C.open_serial(p, BAUDRATE, GRIPPER_ID)
                print(f"✅ 串口已打开: {p}")
            elif cmd == "clear":
                _require(gripper)
                print("✅ 清除错误" if gripper.clear_error() else "❌ 清除失败")
            elif cmd == "release":
                _require(gripper)
                speed = int(parts[1]) if len(parts) > 1 else 200
                print("✅ 已松开" if gripper.release(speed) else "❌ 松开失败")
            elif cmd == "grasp":
                _require(gripper)
                speed = int(parts[1]) if len(parts) > 1 else 150
                force = int(parts[2]) if len(parts) > 2 else 300
                hold = len(parts) > 3 and parts[3].lower() in ("hold", "1", "true")
                print(f"抓取中(speed={speed} force={force} hold={hold})..."
                      if gripper.grasp(speed, force, hold=hold) else "❌ 抓取失败")
            elif cmd == "move":
                _require(gripper)
                opening = int(parts[1])
                print("✅ 已移动" if gripper.move_to(opening) else "❌ 移动失败")
            elif cmd == "opening":
                _require(gripper)
                print(f"当前开口: {gripper.get_opening()}")
            elif cmd == "status":
                _require(gripper)
                st = gripper.get_status()
                print(f"运行状态={st.running_status} 错误码={st.error_code} "
                      f"温度={st.temperature_c}°C")
            elif cmd == "limits":
                _require(gripper)
                mx, mn = gripper.get_open_limits()
                print(f"开口限位: max={mx} min={mn}")
            elif cmd == "setlimits":
                _require(gripper)
                mx, mn = int(parts[1]), int(parts[2])
                print("✅ 限位已设置" if gripper.set_open_limits(mx, mn) else "❌ 设置失败")
            elif cmd == "stop":
                _require(gripper)
                print("✅ 急停" if gripper.emergency_stop() else "❌ 急停失败")
            elif cmd in ("h", "help"):
                show_help()
            elif cmd in ("q", "quit", "exit"):
                print("退出")
                break
            else:
                print(f"❓ 未知命令: {cmd}，输入 h 查看帮助")
        except ProtocolError as e:
            print(f"❌ 协议错误: {e}")
        except ValueError as e:
            print(f"❌ 参数错误: {e}")
        except Exception as e:
            print(f"❌ 异常: {e}")


def _require(gripper):
    if gripper is None:
        raise RuntimeError("请先执行 open 打开串口")


if __name__ == "__main__":
    main()
