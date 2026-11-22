#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
接收 orbis-f118 的 MQTT 截图并拼回 JPEG 文件（PC 端）。

设备端（main/screenshot.c）发的东西：
    switchyard/<DEV>/shot            {"id":N,"size":S,"chunks":C,"chunk_raw":3072,
                                      "w":360,"h":360,"format":"jpeg","quality":80}
    switchyard/<DEV>/shot/<N>/<i>    第 i 片，payload = 这一片的 base64 文本
                                     （每片单独 base64/带 padding，所以逐片解码再拼）

用法：
    pip install paho-mqtt
    set MQTT_HOST=ha.xiaosiqi.cn
    set MQTT_USER=xxx
    set MQTT_PASS=yyy
    set DEV=orbis-f118

    python shot_recv_mqtt.py --trigger            # 自己订阅 + 发命令 + 收图（推荐）
    python shot_recv_mqtt.py                      # 只等（另开终端发命令）
    python shot_recv_mqtt.py --trigger -o ui1.jpg # 指定输出文件
    python shot_recv_mqtt.py --trigger --keep     # 收多张不退出

等价的手工发命令：
    mosquitto_pub -h HOST -p 1883 -u USER -P PASS ^
      -t switchyard/orbis-f118/cmd -m "{\"cmd\":\"screenshot\"}"

设备端日志会打 "截图头已发" 和 "截图上传完成: id=N ..."；没有这两行就是设备侧失败
（PSRAM 不够 / MQTT 没连上），那时改用 {"cmd":"shot_serial"} 走 shot_recv_serial.py。
"""

import argparse
import base64
import json
import os
import pathlib
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("缺 paho-mqtt：pip install paho-mqtt")


def log(msg):
    print(msg, flush=True)


class Receiver:
    """存一帧的各个分片，齐了就叫醒主线程。"""

    def __init__(self):
        self.frames = {}                  # id -> {"size","chunks","parts":{i:bytes},"t0"}
        self.done = threading.Event()
        self.saved_path = None

    def on_header(self, payload: bytes):
        try:
            h = json.loads(payload.decode("utf-8", "replace"))
            sid, size, chunks = int(h["id"]), int(h["size"]), int(h["chunks"])
        except Exception as e:
            log(f"[!] 头解析失败: {e} / {payload!r}")
            return
        if size <= 0 or chunks <= 0:
            log(f"[!] 头字段不合理: {h}")
            return
        self.frames[sid] = {"size": size, "chunks": chunks, "parts": {}, "t0": time.time()}
        log(f"[+] 头 id={sid} {h.get('w')}x{h.get('h')} size={size}B chunks={chunks} "
            f"chunk_raw={h.get('chunk_raw')} q={h.get('quality')}")

    def on_chunk(self, sid: int, idx: int, payload: bytes):
        f = self.frames.get(sid)
        if f is None:
            log(f"[!] 分片 {sid}/{idx} 没有对应的头（错过了头？），忽略")
            return
        try:
            f["parts"][idx] = base64.b64decode(payload)
        except Exception as e:
            log(f"[!] 分片 {sid}/{idx} base64 解码失败: {e}")
            return
        got, need = len(f["parts"]), f["chunks"]
        if got % 5 == 0 or got == need:
            log(f"    id={sid}: {got}/{need} 片")
        if got == need:
            self.finish(sid)

    def finish(self, sid: int):
        f = self.frames.pop(sid)
        missing = [i for i in range(f["chunks"]) if i not in f["parts"]]
        if missing:
            log(f"[!] id={sid} 缺片 {missing[:10]}，放弃")
            return
        data = b"".join(f["parts"][i] for i in range(f["chunks"]))
        if len(data) != f["size"]:
            log(f"[!] id={sid} 拼出 {len(data)}B，头里写 {f['size']}B（仍写文件，可能最后一片被截）")
        if not (data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"):
            log(f"[!] id={sid} 不是完整 JPEG：头 {data[:2].hex()} 尾 {data[-2:].hex()}")
        self.saved_path = OUT_PATH if OUT_PATH else pathlib.Path(
            f"shot_{sid}_{time.strftime('%H%M%S')}.jpg")
        self.saved_path.write_bytes(data)
        log(f"[OK] id={sid} -> {self.saved_path.resolve()}  {len(data)} B，"
            f"用时 {time.time() - f['t0']:.1f}s")
        self.done.set()


def main():
    global OUT_PATH
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("MQTT_HOST", "ha.xiaosiqi.cn"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    ap.add_argument("--user", default=os.environ.get("MQTT_USER", ""))
    ap.add_argument("--password", default=os.environ.get("MQTT_PASS", ""))
    ap.add_argument("--dev", default=os.environ.get("DEV", "orbis-f118"))
    ap.add_argument("-o", "--out", default=None, help="输出文件路径（收一张时用）")
    ap.add_argument("--trigger", action="store_true",
                    help="订阅成功后自动发 {\"cmd\":\"screenshot\"}")
    ap.add_argument("--quality", type=int, default=0, help="0 = 用设备默认 80")
    ap.add_argument("--timeout", type=float, default=60.0, help="等一张图的最长秒数")
    ap.add_argument("--keep", action="store_true", help="收到一张后继续等（默认收一张就退出）")
    args = ap.parse_args()

    OUT_PATH = pathlib.Path(args.out) if args.out else None
    base = f"switchyard/{args.dev}/shot"
    rx = Receiver()

    if hasattr(mqtt, "CallbackAPIVersion"):        # paho-mqtt >= 2.0
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id=f"shot-pc-{os.getpid()}")
    else:                                          # paho-mqtt 1.x
        client = mqtt.Client(client_id=f"shot-pc-{os.getpid()}")
    if args.user:
        client.username_pw_set(args.user, args.password)

    def on_connect(c, userdata, flags, rc, props=None):
        if int(rc) != 0:
            log(f"[!] 连接失败 rc={rc}")
            return
        c.subscribe([(base, 0), (base + "/#", 0)])
        log(f"[+] 已连 {args.host}:{args.port}，订阅 {base} 与 {base}/#")

    def on_subscribe(c, userdata, mid, granted_qos, props=None):
        # SUBACK 之后才发命令，保证头/分片不会漏
        if args.trigger:
            cmd = {"cmd": "screenshot"}
            if args.quality:
                cmd["quality"] = args.quality
            topic = f"switchyard/{args.dev}/cmd"
            c.publish(topic, json.dumps(cmd), qos=0)
            log(f"[>] 已发 {topic} {json.dumps(cmd)}")

    def on_message(c, userdata, msg):
        topic = msg.topic
        if topic == base:
            rx.on_header(msg.payload)
            return
        if topic.startswith(base + "/"):
            parts = topic[len(base) + 1:].split("/")
            if len(parts) != 2:
                return
            try:
                rx.on_chunk(int(parts[0]), int(parts[1]), msg.payload)
            except ValueError:
                pass

    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message

    try:
        client.connect(args.host, args.port, keepalive=30)
    except Exception as e:
        sys.exit(f"[!] 连不上 {args.host}:{args.port}: {e}")

    client.loop_start()
    t0 = time.time()
    try:
        while not rx.done.wait(0.2):
            if time.time() - t0 > args.timeout:
                log(f"[!] {args.timeout:.0f}s 内没收到完整截图")
                for sid, f in list(rx.frames.items()):
                    log(f"    未完成 id={sid}: {len(f['parts'])}/{f['chunks']} 片")
                break
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()

    if not rx.done.is_set() and rx.saved_path is None:
        sys.exit(1)


if __name__ == "__main__":
    main()
