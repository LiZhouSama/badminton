#!/usr/bin/env python3
"""
Windows 侧 COM<->TCP 原始转发桥。
用法示例：
  python win_com_tcp_bridge.py --com COM8 --tcp-port 7008 --baud 115200
  python win_com_tcp_bridge.py --com COM4 --tcp-port 7004 --baud 115200
"""

import argparse
import socket
import threading
import time

import serial


def bridge_once(ser: serial.Serial, host: str, tcp_port: int):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, tcp_port))
    srv.listen(1)
    print(f"[bridge:{ser.port}] listening tcp://{host}:{tcp_port}")

    try:
        while True:
            conn, addr = srv.accept()
            print(f"[bridge:{ser.port}] client connected: {addr}")
            conn.settimeout(0.2)
            stop = threading.Event()

            def s2n():
                while not stop.is_set():
                    try:
                        chunk = ser.read(ser.in_waiting or 1)
                    except Exception as e:
                        print(f"[bridge:{ser.port}] serial read error: {e}")
                        stop.set()
                        break
                    if chunk:
                        try:
                            conn.sendall(chunk)
                        except Exception:
                            stop.set()
                            break

            def n2s():
                while not stop.is_set():
                    try:
                        buf = conn.recv(4096)
                    except socket.timeout:
                        continue
                    except Exception:
                        stop.set()
                        break
                    if not buf:
                        stop.set()
                        break
                    try:
                        ser.write(buf)
                        ser.flush()
                    except Exception as e:
                        print(f"[bridge:{ser.port}] serial write error: {e}")
                        stop.set()
                        break

            t1 = threading.Thread(target=s2n, daemon=True)
            t2 = threading.Thread(target=n2s, daemon=True)
            t1.start()
            t2.start()

            while not stop.is_set():
                time.sleep(0.05)

            try:
                conn.close()
            except Exception:
                pass
            print(f"[bridge:{ser.port}] client disconnected")
    finally:
        srv.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--com", required=True, help="Windows COM, e.g. COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp-host", default="127.0.0.1")
    ap.add_argument("--tcp-port", type=int, required=True)
    args = ap.parse_args()

    while True:
        try:
            ser = serial.Serial(args.com, args.baud, timeout=0.05)
            print(f"[bridge:{args.com}] serial opened @ {args.baud}")
            bridge_once(ser, args.tcp_host, args.tcp_port)
        except Exception as e:
            print(f"[bridge:{args.com}] open/listen failed: {e}")
            time.sleep(1.0)
        finally:
            try:
                ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
