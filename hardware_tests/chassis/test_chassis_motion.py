#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 底盘运动测试：原地旋转 +15°/回正 + 小距离移动/返回
# 通过直控(8767)下发动作，调度(8008)查询位姿，全程打印过程
import json
import time
import urllib.request

CHASSIS = "http://192.168.30.200:8767"
DISPATCH = "http://192.168.30.200:8008"


def post(path, body):
    req = urllib.request.Request(CHASSIS + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=10).read())


def get_status():
    d = json.loads(urllib.request.urlopen(DISPATCH + "/dispatch/get_all_robot_status", timeout=10).read())
    return d["data"]["robotStatusList"][0]


def pose_str(s):
    return f"x={s['x']:.3f} y={s['y']:.3f} angle={s['angle']:.2f} 任务={s['taskStatus']} 急停={s['isForceStop']}"


def wait_arrive(tx, ty, timeout_s=30, step=2):
    """轮询等待底盘到位（位置误差<0.05m）"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(step)
        s = get_status()
        print(f"    {int(time.time()-t0)}s: {pose_str(s)}")
        if abs(s['x'] - tx) < 0.05 and abs(s['y'] - ty) < 0.05:
            return True, s
    return False, get_status()


def main():
    s0 = get_status()
    print("=== 初始 ===", pose_str(s0))

    # ---------- 1. 原地旋转 +15° ----------
    print("\n[1] 原地旋转 +15° ...")
    r = post("/startRotateRobot", {"angle": 15, "isNavAngle": False, "speed": 30})
    print("    响应:", r)
    time.sleep(6)  # 30°/s 转 15° 约 0.5s，多等确保完成
    print("    停止:", post("/stopRotateRobot", {}))
    time.sleep(1)
    s1 = get_status()
    print("    旋转后:", pose_str(s1))

    # ---------- 2. 回正 -15° ----------
    print("\n[2] 原地旋转 -15°（回正）...")
    print("    响应:", post("/startRotateRobot", {"angle": -15, "isNavAngle": False, "speed": 30}))
    time.sleep(6)
    print("    停止:", post("/stopRotateRobot", {}))
    time.sleep(1)
    s2 = get_status()
    print("    回正后:", pose_str(s2))

    # ---------- 3. 小距离移动（x+0.3m，保持同角度） ----------
    sx, sy, angle = s2["x"], s2["y"], s2["angle"]
    tx, ty = round(sx + 0.3, 3), round(sy, 3)
    body = {"px": tx, "py": ty, "angle": angle, "speed": 0.3, "wayType": 3,
            "navScoreLow": 0.2, "type": 0, "isContinue": False,
            "targetCode": "tar_1", "targetId": 1, "startId": 0,
            "startX": sx, "startY": sy, "path": [], "pathIndex": -1}
    print(f"\n[3] 移动: ({sx:.3f},{sy:.3f}) -> ({tx},{ty}) ...")
    print("    响应:", post("/startMoveController", body))
    ok, s3 = wait_arrive(tx, ty)
    print("    停止:", post("/stopMoveController", {}))
    print("    " + ("✅ 到位" if ok else "⚠ 超时未到位（位置误差"), pose_str(s3))

    # ---------- 4. 移回原位 ----------
    s = get_status()
    body2 = {"px": sx, "py": sy, "angle": s["angle"], "speed": 0.3, "wayType": 3,
             "navScoreLow": 0.2, "type": 0, "isContinue": False,
             "targetCode": "tar_2", "targetId": 2, "startId": 0,
             "startX": s["x"], "startY": s["y"], "path": [], "pathIndex": -1}
    print(f"\n[4] 移回: -> ({sx},{sy}) ...")
    print("    响应:", post("/startMoveController", body2))
    ok, s4 = wait_arrive(sx, sy)
    print("    停止:", post("/stopMoveController", {}))
    print("    " + ("✅ 到位" if ok else "⚠ 超时未到位"), pose_str(s4))

    print("\n=== 最终 ===", pose_str(get_status()))


if __name__ == "__main__":
    main()
