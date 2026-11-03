#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 底盘恢复：导航回测试前位姿（位置+角度都到位后不主动停止，给足时间）
import json
import time
import urllib.request

CHASSIS = "http://192.168.30.200:8767"
DISPATCH = "http://192.168.30.200:8008"

HOME_X, HOME_Y, HOME_ANGLE = -3.182, -2.446, -86.14


def post(path, body, timeout=20):
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


def ang_diff(a, b):
    return abs(((a - b + 180) % 360) - 180)


s = get_status()
print("当前:", pose_str(s))
print("目标: x=%s y=%s angle=%s" % (HOME_X, HOME_Y, HOME_ANGLE))

body = {"px": HOME_X, "py": HOME_Y, "angle": HOME_ANGLE, "speed": 0.3, "wayType": 3,
        "navScoreLow": 0.2, "type": 0, "isContinue": False,
        "targetCode": "tar_restore", "targetId": 10, "startId": 0,
        "startX": s['x'], "startY": s['y'], "path": [], "pathIndex": -1}
print("响应:", post("/startMoveController", body))

t0 = time.time()
pos_ok = ang_ok = False
while time.time() - t0 < 90:
    time.sleep(3)
    st = get_status()
    pos_ok = abs(st['x'] - HOME_X) < 0.06 and abs(st['y'] - HOME_Y) < 0.06
    ang_ok = ang_diff(st['angle'], HOME_ANGLE) < 3.0
    print(f"    {int(time.time()-t0)}s: {pose_str(st)} 位置={'✅' if pos_ok else '—'} 角度={'✅' if ang_ok else '—'}")
    if pos_ok and ang_ok:
        print("✅ 位置+角度均到位")
        break

print("最终:", pose_str(get_status()))
