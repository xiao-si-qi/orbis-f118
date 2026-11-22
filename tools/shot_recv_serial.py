#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口取图（备用路线，设备端 {"cmd":"shot_serial"}）。

设备端（main/screenshot.c）打印的格式，115200 8N1：
    ===IMG BEGIN w=360 h=360 size=12345===
    0,<512 个 base64 字符>
    1,<...>
    ...
    ===IMG END===

每行是独立 base64（384 原始字节 → 512 字符，正好是 3 的倍数，所以除最后一行外没有 padding），
所以也是逐行解码再拼。行首的 seq 用来发现"被日志插掉的行"：日志里混进来的行不匹配
数字开头的格式，会直接跳过；如果最后有缺号会明确报出来。

用法：
    pip install pyserial
    python shot_recv_serial.py COM7 out.jpg
    python shot_recv_serial.py COM7 out.jpg --send     # 自己先在串口发命令 + 抓图
    （注意：端口自己填。本工程没有把 COM9 写死在任何地方，也没有脚本会去开 COM9。）

发命令（任选其一）：
    mosquitto_pub ... -t switchyard/orbis-f118/cmd -m "{\"cmd\":\"shot_serial\"}"
    或 python shot_recv_serial.py COM7 out.jpg --send
"""

import argparse
import base64
import pathlib
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺 pyserial：pip install pyserial")

RE_BEGIN = re.compile(r"^===IMG BEGIN w=(\d+) h=(\d+) size=(\d+)===$")
RE_LINE = re.compile(r"^(\d+),([A-Za-z0-9+/=]+)$")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="串口，例如 COM7（本项目默认不碰 COM9）")
    ap.add_argument("out", nargs="?", default="shot_serial.jpg")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--send", action="store_true", help="开始前先往串口写 {\"cmd\":\"shot_serial\"}")
    ap.add_argument("--timeout", type=float, default=30.0, help="等 BEGIN 的秒数")
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()
    if args.send:
        ser.write(b'{"cmd":"shot_serial"}\n')
        ser.flush()
        print(f"[>] 已发 {{\"cmd\":\"shot_serial\"}} 到 {args.port}")

    print(f"[+] 监听 {args.port} @ {args.baud}，等 ===IMG BEGIN=== ...")
    t0 = time.time()
    size = None
    parts = {}
    width = height = 0

    while True:
        raw = ser.readline()
        if not raw:
            if size is None and time.time() - t0 > args.timeout:
                ser.close()
                sys.exit(f"[!] {args.timeout:.0f}s 内没等到 ===IMG BEGIN===（命令没到？看图传？）")
            continue
        line = raw.decode("utf-8", "replace").strip()

        if size is None:
            m = RE_BEGIN.match(line)
            if m:
                width, height, size = int(m.group(1)), int(m.group(2)), int(m.group(3))
                parts = {}
                t0 = time.time()
                print(f"[+] BEGIN {width}x{height} size={size}B，开始收 base64 行")
            continue

        if line == "===IMG END===":
            break

        m = RE_LINE.match(line)
        if not m:
            continue                    # 日志行：跳过
        idx = int(m.group(1))
        try:
            parts[idx] = base64.b64decode(m.group(2))
        except Exception as e:
            print(f"[!] 第 {idx} 行解码失败: {e}")
        if len(parts) % 20 == 0:
            print(f"    {len(parts)} 行 / {sum(len(v) for v in parts.values())} B")

    ser.close()

    missing = [i for i in range(max(parts) + 1) if i not in parts] if parts else []
    data = b"".join(parts[i] for i in sorted(parts))
    if missing:
        print(f"[!] 缺 {len(missing)} 行: {missing[:10]}...（日志插进来了，重发一次）")
    if len(data) != size:
        print(f"[!] 收到 {len(data)}B，头里写 {size}B")
    if data[:2] != b"\xff\xd8":
        print(f"[!] 不是 JPEG 开头（{data[:2].hex()}）")
    out.write_bytes(data)
    print(f"[OK] -> {out.resolve()}  {len(data)}B  用时 {time.time() - t0:.1f}s")

if __name__ == "__main__":
    main()
