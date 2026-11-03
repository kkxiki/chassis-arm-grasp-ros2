#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 脚本：send_arm_cmd.py
# 作用：直连机械臂控制器(192.168.30.11:5868)，从 stdin 逐行
#       读取命令并发送，保持连接直到输入 q 退出。
#       支持特殊行：sleep N  等待 N 毫秒（期间保持连接，
#       让异步运动指令继续执行，避免断开导致运动中断）。
# 用法：
#   printf '{Clear}\n{Enable}\nsleep 3000\nq\n' | python3 send_arm_cmd.py [ip]
# ============================================================
import socket
import struct
import sys
import time
import json

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.30.11"
PORT = 5868
seq = 0


def pack_frame(cmd, seq_id):
    # 与厂商协议一致：0x22 + MsgHeader(40B) + 命令体
    header = struct.pack("<IIqqqq", len(cmd), 8888, 1, seq_id,
                         int(time.time() * 1e6), 202401)
    return b"\x22" + header + cmd.encode()


def recv_frame(sock):
    # 帧同步：丢弃状态推送帧，找到 0x22
    while True:
        b = sock.recv(1)
        if not b:
            raise TimeoutError("连接关闭")
        if b == b"\x22":
            break
    hdr = b""
    while len(hdr) < 40:
        chunk = sock.recv(40 - len(hdr))
        if not chunk:
            raise TimeoutError("接收应答头失败")
        hdr += chunk
    size = struct.unpack("<I", hdr[:4])[0]
    body = b""
    while len(body) < size:
        chunk = sock.recv(size - len(body))
        if not chunk:
            raise TimeoutError("接收应答体失败")
        body += chunk
    return body.decode(errors="replace")


def main():
    global seq
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((IP, PORT))
    print(f"[TCP] 已连接 {IP}:{PORT}", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        if line in ("q", "quit"):
            break
        low = line.lower()
        if low.startswith("sleep"):
            ms = int(line.split()[1])
            print(f"[等待] {ms}ms (保持连接)", flush=True)
            time.sleep(ms / 1000.0)
            continue
        seq += 1
        sock.sendall(pack_frame(line, seq))
        try:
            reply = recv_frame(sock)
        except TimeoutError as e:
            print(f"[超时] {line} -> {e}", flush=True)
            continue
        try:
            j = json.loads(reply)
            print(f"[发送] {line}\n[返回] code={j.get('return_code')} "
                  f"{str(j.get('return_message',''))[:100]}", flush=True)
        except Exception:
            print(f"[发送] {line}\n[返回] raw={reply[:200]}", flush=True)
    sock.close()
    print("[TCP] 已断开", flush=True)


main()
