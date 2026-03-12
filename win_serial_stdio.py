#!/usr/bin/env python3
"""
Windows 侧串口 <-> stdin/stdout 桥。
供 WSL 主程序通过子进程管道访问 COM 口。
"""

import argparse
import os
import sys
import threading
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--com", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    # Windows 下把 stdin/stdout 设为二进制模式
    if os.name == "nt":
        try:
            import msvcrt

            msvcrt.setmode(sys.stdin.fileno(), os.O_BINARY)
            msvcrt.setmode(sys.stdout.fileno(), os.O_BINARY)
        except Exception:
            pass

    ser = serial.Serial(args.com, args.baud, timeout=0.02)
    print(f"opened {args.com} @ {args.baud}", file=sys.stderr, flush=True)

    stop = threading.Event()

    def serial_to_stdout():
        while not stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except Exception as e:
                print(f"serial read error: {e}", file=sys.stderr, flush=True)
                stop.set()
                break
            if not chunk:
                continue
            try:
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
            except Exception as e:
                print(f"stdout write error: {e}", file=sys.stderr, flush=True)
                stop.set()
                break

    t = threading.Thread(target=serial_to_stdout, daemon=True)
    t.start()

    try:
        stdin_fd = sys.stdin.fileno()
        while not stop.is_set():
            data = os.read(stdin_fd, 1024)
            # 在 Windows 子进程里，管道有时会短暂返回空字节，不代表真正 EOF。
            # 为避免误退出，空读时小睡并继续。
            if not data:
                time.sleep(0.01)
                continue
            ser.write(data)
            ser.flush()
    except Exception as e:
        print(f"stdin->serial error: {e}", file=sys.stderr, flush=True)
    finally:
        stop.set()
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
