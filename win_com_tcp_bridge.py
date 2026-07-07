#!/usr/bin/env python3
"""
Windows-side raw COM-to-TCP bridge for WSL2 acquisition.

Default mapping:
  PRESSURE: COM10 -> tcp://0.0.0.0:17010
  IMU:      COM4  -> tcp://0.0.0.0:17004
"""

from __future__ import annotations

import argparse
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import serial


@dataclass
class BridgeSpec:
    name: str
    com: str
    tcp_port: int
    baud: int


class SerialTcpBridge(threading.Thread):
    def __init__(self, spec: BridgeSpec, tcp_host: str, status_interval: float, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.spec = spec
        self.tcp_host = tcp_host
        self.status_interval = status_interval
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            ser: Optional[serial.Serial] = None
            srv: Optional[socket.socket] = None
            try:
                ser = serial.Serial(self.spec.com, self.spec.baud, timeout=0.01, write_timeout=1.0)
                print(f"[{self.spec.name}] opened {self.spec.com} @ {self.spec.baud}", flush=True)

                srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((self.tcp_host, self.spec.tcp_port))
                srv.listen(1)
                srv.settimeout(0.5)
                print(
                    f"[{self.spec.name}] listening tcp://{self.tcp_host}:{self.spec.tcp_port} "
                    f"for {self.spec.com}",
                    flush=True,
                )

                while not self.stop_event.is_set():
                    try:
                        conn, addr = srv.accept()
                    except socket.timeout:
                        continue
                    self._serve_client(ser, conn, addr)
            except Exception as exc:
                print(f"[{self.spec.name}] bridge error: {exc}", flush=True)
                time.sleep(1.0)
            finally:
                if srv is not None:
                    try:
                        srv.close()
                    except OSError:
                        pass
                if ser is not None and ser.is_open:
                    ser.close()
                    print(f"[{self.spec.name}] closed {self.spec.com}", flush=True)

    def _serve_client(self, ser: serial.Serial, conn: socket.socket, addr) -> None:
        print(f"[{self.spec.name}] WSL client connected: {addr}", flush=True)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(0.05)
        client_stop = threading.Event()
        counters = {"serial_to_tcp": 0, "tcp_to_serial": 0}

        def serial_to_tcp() -> None:
            while not self.stop_event.is_set() and not client_stop.is_set():
                try:
                    chunk = ser.read(ser.in_waiting or 1)
                    if chunk:
                        conn.sendall(chunk)
                        counters["serial_to_tcp"] += len(chunk)
                except Exception as exc:
                    if not client_stop.is_set():
                        print(f"[{self.spec.name}] serial->tcp stopped: {exc}", flush=True)
                    client_stop.set()
                    break

        def tcp_to_serial() -> None:
            while not self.stop_event.is_set() and not client_stop.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except Exception as exc:
                    if not client_stop.is_set():
                        print(f"[{self.spec.name}] tcp->serial stopped: {exc}", flush=True)
                    client_stop.set()
                    break
                if not chunk:
                    client_stop.set()
                    break
                try:
                    ser.write(chunk)
                    ser.flush()
                    counters["tcp_to_serial"] += len(chunk)
                except Exception as exc:
                    if not client_stop.is_set():
                        print(f"[{self.spec.name}] serial write stopped: {exc}", flush=True)
                    client_stop.set()
                    break

        t_serial = threading.Thread(target=serial_to_tcp, daemon=True)
        t_tcp = threading.Thread(target=tcp_to_serial, daemon=True)
        t_serial.start()
        t_tcp.start()

        last_status = time.time()
        try:
            while not self.stop_event.is_set() and not client_stop.wait(0.1):
                now = time.time()
                if self.status_interval > 0 and now - last_status >= self.status_interval:
                    print(
                        f"[{self.spec.name}] bytes serial->tcp={counters['serial_to_tcp']} "
                        f"tcp->serial={counters['tcp_to_serial']}",
                        flush=True,
                    )
                    last_status = now
        finally:
            client_stop.set()
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass
            print(f"[{self.spec.name}] WSL client disconnected", flush=True)


def parse_map(text: str, default_baud: int) -> BridgeSpec:
    if "=" in text:
        name, rest = text.split("=", 1)
    else:
        name, rest = "COM", text
    parts = rest.split(":")
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("map must be NAME=COM:TCP_PORT[:BAUD] or COM:TCP_PORT[:BAUD]")
    com = parts[0]
    tcp_port = int(parts[1])
    baud = int(parts[2]) if len(parts) == 3 else default_baud
    return BridgeSpec(name=name.upper(), com=com, tcp_port=tcp_port, baud=baud)


def build_specs(args: argparse.Namespace) -> List[BridgeSpec]:
    if args.com or args.tcp_port:
        if not args.com or not args.tcp_port:
            raise SystemExit("--com and --tcp-port must be used together")
        return [BridgeSpec(name=args.name, com=args.com, tcp_port=args.tcp_port, baud=args.baud)]

    if args.map:
        return [parse_map(item, args.baud) for item in args.map]

    specs: List[BridgeSpec] = []
    if not args.no_pressure:
        specs.append(
            BridgeSpec(
                name="PRESSURE",
                com=args.pressure_com,
                tcp_port=args.pressure_tcp_port,
                baud=args.pressure_baud or args.baud,
            )
        )
    if not args.no_imu:
        specs.append(
            BridgeSpec(
                name="IMU",
                com=args.imu_com,
                tcp_port=args.imu_tcp_port,
                baud=args.imu_baud or args.baud,
            )
        )
    if not specs:
        raise SystemExit("No bridges selected")
    return specs


def main() -> None:
    parser = argparse.ArgumentParser(description="Windows COM-to-TCP bridge for WSL2.")
    parser.add_argument("--tcp-host", default="0.0.0.0", help="Bind address. Use 0.0.0.0 for WSL2 NAT access.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--status-interval", type=float, default=5.0)

    parser.add_argument("--pressure-com", default="COM10")
    parser.add_argument("--pressure-baud", type=int, default=None)
    parser.add_argument("--pressure-tcp-port", type=int, default=17010)
    parser.add_argument("--no-pressure", action="store_true")

    parser.add_argument("--imu-com", default="COM4")
    parser.add_argument("--imu-baud", type=int, default=None)
    parser.add_argument("--imu-tcp-port", type=int, default=17004)
    parser.add_argument("--no-imu", action="store_true")

    parser.add_argument("--map", action="append", help="Custom mapping: NAME=COM:TCP_PORT[:BAUD]")

    parser.add_argument("--com", default=None, help="Single-bridge compatibility mode COM port.")
    parser.add_argument("--tcp-port", type=int, default=None, help="Single-bridge compatibility mode TCP port.")
    parser.add_argument("--name", default="BRIDGE", help="Single-bridge compatibility mode name.")
    args = parser.parse_args()

    if os.name != "nt":
        print("[bridge] warning: this script is intended to run with Windows Python.", flush=True)

    specs = build_specs(args)
    print("[bridge] starting mappings:", flush=True)
    for spec in specs:
        print(f"[bridge]   {spec.name}: {spec.com} @ {spec.baud} -> tcp://{args.tcp_host}:{spec.tcp_port}", flush=True)
    print(
        "[bridge] WSL example: python run_dual_sensor_sage.py "
        "--pressure-port wintcp://17010 --imu-port wintcp://17004",
        flush=True,
    )

    stop_event = threading.Event()
    threads = [SerialTcpBridge(spec, args.tcp_host, args.status_interval, stop_event) for spec in specs]
    for thread in threads:
        thread.start()

    try:
        while any(thread.is_alive() for thread in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("[bridge] stopping", flush=True)
        stop_event.set()
    finally:
        stop_event.set()
        for thread in threads:
            thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
