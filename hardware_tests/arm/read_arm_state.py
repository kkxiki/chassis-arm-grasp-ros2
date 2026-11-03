#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 读取机械臂当前状态（关节角 + 笛卡尔坐标）
# 原理：通过 ZMQ 订阅控制器 19091 端口的 overall_system_rtstate
#       状态帧，手工解析 protobuf 里的 model[].current_point
#       (robottarget=笛卡尔位姿, jointtarget=关节角) 与 model[].joint。
# 不依赖 ROS2 / protobuf 生成代码，仅需 pyzmq。
# 用法：python3 read_arm_state.py [--robot-ip 192.168.30.11] [--duration 2]
# ============================================================
import argparse
import math
import struct
import time


def read_varint(buf, pos):
    """读 protobuf varint，返回 (value, new_pos)。"""
    shift = 0
    value = 0
    while True:
        if pos >= len(buf):
            raise ValueError("bad varint")
        b = buf[pos]
        pos += 1
        value |= (b & 0x7F) << shift
        if not (b & 0x80):
            return value, pos
        shift += 7


def parse_frame(frame):
    """拆 ZMQ 帧：[topic]\x00\x00 [meta_len varint][meta] [proto_len BE32][proto]"""
    sep = frame.find(b"\x00\x00")
    if sep < 0:
        raise ValueError("no topic separator")
    topic = frame[:sep]
    rest = frame[sep + 2:]
    meta_len, pos = read_varint(rest, 0)
    pos += meta_len
    if len(rest) - pos < 4:
        raise ValueError("missing proto length prefix")
    proto_len = struct.unpack(">I", rest[pos:pos + 4])[0]
    proto = rest[pos + 4:pos + 4 + proto_len]
    if len(proto) != proto_len:
        raise ValueError("truncated proto payload")
    return topic, proto


def skip(buf, i, wire):
    """按 wire type 跳过一段字段，返回新位置。"""
    if wire == 0:
        _, i = read_varint(buf, i)
    elif wire == 1:
        i += 8
    elif wire == 2:
        ln, i = read_varint(buf, i)
        i += ln
    elif wire == 5:
        i += 4
    else:
        raise ValueError("bad wire type %d" % wire)
    return i


def extract_doubles(buf, field_no):
    """提取某个 repeated double 字段（兼容 packed 与 unpacked）。"""
    out = []
    i = 0
    n = len(buf)
    while i < n:
        tag, i = read_varint(buf, i)
        f = tag >> 3
        wire = tag & 7
        if f == field_no and wire == 2:      # packed
            ln, i = read_varint(buf, i)
            chunk = buf[i:i + ln]
            i += ln
            out += struct.unpack("<%dd" % (len(chunk) // 8), chunk)
        elif f == field_no and wire == 1:    # unpacked 单个 double
            out.append(struct.unpack("<d", buf[i:i + 8])[0])
            i += 8
        else:
            i = skip(buf, i, wire)
    return out


def get_joint_fields(buf, want=(2, 11)):
    """从 Joint 消息里提取指定 double 字段，返回 {field_no: value}。
    position=2(当前关节角 rad), target_position=11(目标关节角 rad)。"""
    out = {}
    i = 0
    n = len(buf)
    while i < n:
        tag, i = read_varint(buf, i)
        f = tag >> 3
        wire = tag & 7
        if wire == 1:
            if f in want:
                out[f] = struct.unpack("<d", buf[i:i + 8])[0]
            i += 8
        else:
            i = skip(buf, i, wire)
    return out


def quat_to_rpy(qx, qy, qz, qw):
    """四元数 → 欧拉角(rad)，ZYX 约定 (roll, pitch, yaw)。"""
    import math
    roll = math.atan2(2 * (qw * qx + qy * qz),
                      1 - 2 * (qx * qx + qy * qy))
    sinp = 2 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (qw * qz + qx * qy),
                     1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def parse_model(buf):
    """解析 Model：返回 (name, joints[], robottarget[], jointtarget[],
    error_code, error_msg, func_name, func_info)。"""
    name = ""
    joints = []          # 每项 (当前关节角rad, 目标关节角rad)，可能为 None
    cp = None
    error_code = None
    error_msg = ""
    func_name = ""
    func_info = ""
    i = 0
    n = len(buf)
    while i < n:
        tag, i = read_varint(buf, i)
        f = tag >> 3
        wire = tag & 7
        if f == 1 and wire == 2:
            ln, i = read_varint(buf, i)
            name = buf[i:i + ln].decode("utf-8", "ignore")
            i += ln
        elif f == 3 and wire == 2:           # joint
            ln, i = read_varint(buf, i)
            jbuf = buf[i:i + ln]
            i += ln
            fd = get_joint_fields(jbuf)
            joints.append((fd.get(2), fd.get(11)))
        elif f == 4 and wire == 2:           # current_point
            ln, i = read_varint(buf, i)
            cp = buf[i:i + ln]
            i += ln
        elif f == 5 and wire == 0:           # error_code
            error_code, i = read_varint(buf, i)
        elif f == 6 and wire == 2:           # error_msg
            ln, i = read_varint(buf, i)
            error_msg = buf[i:i + ln].decode("utf-8", "ignore")
            i += ln
        elif f == 9 and wire == 2:           # current_func_name
            ln, i = read_varint(buf, i)
            func_name = buf[i:i + ln].decode("utf-8", "ignore")
            i += ln
        elif f == 10 and wire == 2:          # current_func_info
            ln, i = read_varint(buf, i)
            func_info = buf[i:i + ln].decode("utf-8", "ignore")
            i += ln
        else:
            i = skip(buf, i, wire)
    rt = extract_doubles(cp, 4) if cp else []
    jt = extract_doubles(cp, 5) if cp else []
    return name, joints, rt, jt, error_code, error_msg, func_name, func_info


def walk_models(proto):
    """SystemRtState.model 是 field 3。返回所有模型解析结果。"""
    models = []
    i = 0
    n = len(proto)
    while i < n:
        tag, i = read_varint(proto, i)
        f = tag >> 3
        wire = tag & 7
        if f == 3 and wire == 2:
            ln, i = read_varint(proto, i)
            models.append(parse_model(proto[i:i + ln]))
            i += ln
        else:
            i = skip(proto, i, wire)
    return models


def main():
    ap = argparse.ArgumentParser(description="读取机械臂当前状态")
    ap.add_argument("--robot-ip", default="192.168.30.11")
    ap.add_argument("--state-port", type=int, default=19091)
    ap.add_argument("--duration", type=float, default=2.0,
                    help="采样时长(秒)，默认2秒读最新一帧")
    args = ap.parse_args()

    try:
        import zmq
    except ImportError:
        print("❌ 缺少 pyzmq，请先安装: pip3 install pyzmq")
        return

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.RCVTIMEO, 1000)
    sub.connect("tcp://%s:%d" % (args.robot_ip, args.state_port))
    sub.setsockopt(zmq.SUBSCRIBE, b"overall_system_rtstate")

    print("连接 %s:%d 等待状态帧..." % (args.robot_ip, args.state_port))
    got = False
    deadline = time.time() + args.duration
    while time.time() < deadline:
        try:
            frame = sub.recv()
        except zmq.Again:
            continue
        topic, proto = parse_frame(frame)
        if topic != b"overall_system_rtstate":
            continue
        models = walk_models(proto)
        for idx, (name, joints, rt, jt, err, errmsg, fname, finfo) in enumerate(models):
            print("[model %d] 名称=%s" % (idx, name))
            if err:
                print("  错误码=%s 错误信息=%s" % (err, errmsg))
            if fname:
                print("  当前指令=%s  %s" % (fname, finfo))
            if joints:
                deg = [round(math.degrees(p), 3) if p is not None else None
                       for p, _ in joints]
                tdeg = [round(math.degrees(t), 3) if t is not None else None
                        for _, t in joints]
                print("  当前关节角(°): %s" % deg)
                print("  目标关节角(°): %s" % tdeg)
            if rt:
                print("  笛卡尔位姿 robottarget: %s" % [round(v, 4) for v in rt])
                if len(rt) == 7:   # [x,y,z, qx,qy,qz,qw]
                    roll, pitch, yaw = quat_to_rpy(rt[3], rt[4], rt[5], rt[6])
                    print("  姿态 RPY(°, ZYX推算): [%.2f, %.2f, %.2f]"
                          % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
            if jt:
                print("  目标关节角(rad): %s" % [round(v, 4) for v in jt])
            print("---")
        got = True
        break  # 读一帧即可
    sub.close(linger=0)
    ctx.term()
    if not got:
        print("❌ 未收到状态帧，请确认机械臂在线且端口可访问")


if __name__ == "__main__":
    main()
