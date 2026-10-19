#!/bin/bash
# 底盘或雷达接上后，扫一遍常见串口，看是不是塔克协议。
python3 - << 'PY'
import glob, os, time
try:
    import serial
except ImportError:
    raise SystemExit("缺少 pyserial，请先安装 python3-serial")

candidates = ["/dev/openctr", "/dev/ldlidar"] + sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
seen = []
for path in candidates:
    if path not in seen and os.path.exists(path):
        seen.append(path)
if not seen:
    print("没有发现串口设备。OpenCTR 和 LD06 目前都还没接到香橙派。")
    raise SystemExit(0)
for path in seen:
    for baud in (115200, 230400):
        try:
            ser = serial.Serial(path, baud, timeout=0.3)
        except Exception as exc:
            print(f"{path} @ {baud}: 打不开 ({exc})")
            continue
        time.sleep(0.2)
        data = ser.read(400)
        ser.close()
        ok = b"\xaa\x55" in data
        print(f"{path} @ {baud}: 读到 {len(data)} 字节, 塔克帧头={'有' if ok else '无'}")
PY
