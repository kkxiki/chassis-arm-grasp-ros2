#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 底盘收尾：将朝向转回初始角度（绝对角度旋转），并恢复到测试前位置附近
import json
import time
import urllib.request

CHASSIS = "http://192.168.30.200:8767"
DISPATCH = "http://192.168.30.200:8008"

HOME_X, HOME_Y, HOME_ANGLE = -3.182, -2.446, -86.14  # 测试前初始位姿


def post(path, body, timeout=15):
    req = urllib.request.Request(CHASSIS + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        return json.loads(urllib.request.urlopen(req, timeout=timeout).read())
    except Exception as e:
        return {"error": str(e)}


def get_status():
    d = json.loads(urllib.request.urlopen(DISPATCH + "/dispatch/get_all_robot_status", timeout=10).read())
    return d["data"]["robotStatusList"][0]


def pose_str(s):
    return f"x={s['x']:.3f} y={s['y']:.3f} angle={s['angle']:.2f} 任务={s['taskStatus']}"


def wait_angle(target, timeout_s=20):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout_s:
        time.sleep(2)
        last = get_status()
        diff = abs(((last['angle'] - target + 180) % 360) - 180)
        print(f"    {int(time.time()-t0)}s: {pose_str(last)} (偏差{diff:.1f}°)")
        if diff < 3.0:
            return True, last
    return False, last


def wait_arrive(tx, ty, timeout_s=30):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(2)
        s = get_status()
        print(f"    {int(time.time()-t0)}s: {pose_str(s)}")
        if abs(s['x'] - tx) < 0.06 and abs(s['y'] - ty) < 0.06:
            return True, s
    return False, get_status()


s = get_status()
print("当前:", pose_str(s))

# 1. 绝对角度旋转回 -86°（isNavAngle=true）
print("\n[1] 绝对旋转到 -86.14° ...")
print("    响应:", post("/startRotateRobot", {"angle": HOME_ANGLE, "isNavAngle": True, "speed": 30}))
ok, s1 = wait_angle(HOME_ANGLE)
print("    停止:", post("/stopRotateRobot", {}))
time.sleep(1)
print("    " + ("✅ 角度到位" if ok else "⚠ 角度未到位"), pose_str(get_status()))

# 2. 位置微调回测试前位置
s = get_status()
body = {"px": HOME_X, "py": HOME_Y, "angle": s['angle'], "speed": 0.3, "wayType": 3,
        "navScoreLow": 0.2, "type": 0, "isContinue": False,
        "targetCode": "tar_home", "targetId": 9, "startId": 0,
        "startX": s['x'], "startY": s['y'], "path": [], "pathIndex": -1}
print(f"\n[2] 移回 ({HOME_X},{HOME_Y}) ...")
print("    响应:", post("/startMoveController", body))
ok, s2 = wait_arrive(HOME_X, HOME_Y)
print("    停止:", post("/stopMoveController", {}))
print("    " + ("✅ 到位" if ok else "⚠ 未到位"), pose_str(get_status()))

print("\n=== 最终 ===", pose_str(get_status()))
