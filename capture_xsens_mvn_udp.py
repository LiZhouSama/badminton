#!/usr/bin/env python3
"""
Capture and decode Xsens MVN Analyze/Animate real-time UDP packets to CSV.

Recommended MVN Network Streamer setting for full-body + Manus/Xsens
Metagloves capture:
  - Protocol: UDP
  - Datagram: Position + Orientation (Quaternion), packet type MXTP02
  - Port: 9763 unless you changed it in MVN

The MXTP02 pose CSV is written in long format: one row per streamed segment.
When finger tracking is enabled in MVN, the stream normally contains 23 body
segments plus 40 finger-tracking segments.

Recent MVN Analyze network-streamer documentation describes position values as
meters. Older protocol PDFs described centimeters, so this script keeps both
meter and centimeter columns but treats the received float as meters by default.
"""

from __future__ import annotations

import argparse
import csv
import signal
import socket
import struct
import sys
import tempfile
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


DEFAULT_PORT = 9763
HEADER_SIZE = 24
HEADER_STRUCT = struct.Struct(">6sIBBIBBBBHH")
POSITION_UNIT = "m"

BODY_SEGMENTS = [
    "Pelvis",
    "L5",
    "L3",
    "T12",
    "T8",
    "Neck",
    "Head",
    "Right Shoulder",
    "Right Upper Arm",
    "Right Forearm",
    "Right Hand",
    "Left Shoulder",
    "Left Upper Arm",
    "Left Forearm",
    "Left Hand",
    "Right Upper Leg",
    "Right Lower Leg",
    "Right Foot",
    "Right Toe",
    "Left Upper Leg",
    "Left Lower Leg",
    "Left Foot",
    "Left Toe",
]

FINGER_SEGMENTS = [
    "Carpus",
    "First Metacarpal",
    "First Proximal Phalange",
    "First Distal Phalange",
    "Second Metacarpal",
    "Second Proximal Phalange",
    "Second Middle Phalange",
    "Second Distal Phalange",
    "Third Metacarpal",
    "Third Proximal Phalange",
    "Third Middle Phalange",
    "Third Distal Phalange",
    "Fourth Metacarpal",
    "Fourth Proximal Phalange",
    "Fourth Middle Phalange",
    "Fourth Distal Phalange",
    "Fifth Metacarpal",
    "Fifth Proximal Phalange",
    "Fifth Middle Phalange",
    "Fifth Distal Phalange",
]

PACKET_COLUMNS = [
    "recv_time_s",
    "recv_time_iso",
    "source_ip",
    "source_port",
    "packet_seq",
    "raw_size_bytes",
    "packet_id",
    "message_type",
    "valid_mxtp",
    "sample_counter",
    "datagram_counter_raw",
    "datagram_index",
    "datagram_is_last",
    "number_of_items",
    "time_code_ms",
    "character_id",
    "body_segment_count",
    "prop_count",
    "finger_segment_count",
    "reserved",
    "payload_size",
    "actual_payload_size",
    "payload_size_ok",
    "parse_status",
]

BASE_DETAIL_COLUMNS = [
    "recv_time_s",
    "recv_time_iso",
    "source_ip",
    "source_port",
    "packet_seq",
    "packet_id",
    "message_type",
    "sample_counter",
    "time_code_ms",
    "character_id",
    "datagram_index",
    "datagram_is_last",
    "datagram_items",
    "body_segment_count",
    "prop_count",
    "finger_segment_count",
    "payload_size",
    "sample_complete",
    "sample_datagram_count",
    "missing_datagrams",
]

POSE_COLUMNS = BASE_DETAIL_COLUMNS + [
    "item_order_index",
    "item_kind",
    "item_name",
    "finger_side",
    "segment_id",
    "segment_index",
    "segment_name_from_id",
    "segment_id_base",
    "coordinate_system",
    "position_unit",
    "rotation_format",
    "x_cm",
    "y_cm",
    "z_cm",
    "x_m",
    "y_m",
    "z_m",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "rot_x_deg",
    "rot_y_deg",
    "rot_z_deg",
    "parse_warning",
]

POINT_COLUMNS = BASE_DETAIL_COLUMNS + [
    "item_order_index",
    "point_id",
    "x_cm",
    "y_cm",
    "z_cm",
    "x_m",
    "y_m",
    "z_m",
    "coordinate_system",
    "parse_warning",
]

JOINT_ANGLE_COLUMNS = BASE_DETAIL_COLUMNS + [
    "item_order_index",
    "parent_point_id",
    "child_point_id",
    "rot_x_deg",
    "rot_y_deg",
    "rot_z_deg",
    "coordinate_system",
    "parse_warning",
]

LINEAR_KINEMATICS_COLUMNS = BASE_DETAIL_COLUMNS + [
    "item_order_index",
    "item_kind",
    "item_name",
    "segment_id",
    "segment_index",
    "segment_name_from_id",
    "segment_id_base",
    "x_cm",
    "y_cm",
    "z_cm",
    "x_m",
    "y_m",
    "z_m",
    "vel_x_cm_s",
    "vel_y_cm_s",
    "vel_z_cm_s",
    "acc_x_cm_s2",
    "acc_y_cm_s2",
    "acc_z_cm_s2",
    "coordinate_system",
    "parse_warning",
]

ANGULAR_KINEMATICS_COLUMNS = BASE_DETAIL_COLUMNS + [
    "item_order_index",
    "item_kind",
    "item_name",
    "segment_id",
    "segment_index",
    "segment_name_from_id",
    "segment_id_base",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "ang_vel_x",
    "ang_vel_y",
    "ang_vel_z",
    "ang_acc_x",
    "ang_acc_y",
    "ang_acc_z",
    "coordinate_system",
    "parse_warning",
]

TRACKER_KINEMATICS_COLUMNS = BASE_DETAIL_COLUMNS + [
    "item_order_index",
    "attached_segment_id",
    "attached_segment_index",
    "attached_segment_name",
    "segment_id_base",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "free_acc_x",
    "free_acc_y",
    "free_acc_z",
    "local_acc_x",
    "local_acc_y",
    "local_acc_z",
    "angular_vel_x",
    "angular_vel_y",
    "angular_vel_z",
    "mag_x",
    "mag_y",
    "mag_z",
    "coordinate_system",
    "parse_warning",
]

COM_COLUMNS = BASE_DETAIL_COLUMNS + [
    "x_cm",
    "y_cm",
    "z_cm",
    "x_m",
    "y_m",
    "z_m",
    "coordinate_system",
    "parse_warning",
]

TIMECODE_COLUMNS = BASE_DETAIL_COLUMNS + [
    "timecode",
    "parse_warning",
]

METADATA_COLUMNS = BASE_DETAIL_COLUMNS + [
    "tag",
    "value",
    "raw_text",
    "parse_warning",
]

SCALE_SEGMENT_COLUMNS = BASE_DETAIL_COLUMNS + [
    "segment_order_index",
    "segment_name",
    "x_cm",
    "y_cm",
    "z_cm",
    "x_m",
    "y_m",
    "z_m",
    "coordinate_system",
    "parse_warning",
]

SCALE_POINT_COLUMNS = BASE_DETAIL_COLUMNS + [
    "point_order_index",
    "segment_id",
    "point_id",
    "point_name",
    "flags",
    "x_cm",
    "y_cm",
    "z_cm",
    "x_m",
    "y_m",
    "z_m",
    "coordinate_system",
    "parse_warning",
]

UNKNOWN_COLUMNS = BASE_DETAIL_COLUMNS + [
    "payload_hex_prefix",
    "payload_len",
    "parse_warning",
]


class MvnPacketError(ValueError):
    """Raised when a datagram cannot be read as an MVN packet."""


@dataclass(frozen=True)
class MvnHeader:
    packet_id: str
    message_type: str
    sample_counter: int
    datagram_counter_raw: int
    datagram_index: int
    datagram_is_last: bool
    number_of_items: int
    time_code_ms: int
    character_id: int
    body_segment_count: int
    prop_count: int
    finger_segment_count: int
    reserved: int
    payload_size: int
    actual_payload_size: int
    payload_size_ok: bool
    valid_mxtp: bool


@dataclass
class DatagramPart:
    header: MvnHeader
    payload: bytes
    recv_time_s: float
    source_ip: str
    source_port: int
    packet_seq: int


@dataclass
class PendingSample:
    first_seen_s: float
    parts: Dict[int, DatagramPart]
    last_index: Optional[int] = None


class CsvSink:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._files: Dict[str, object] = {}
        self._writers: Dict[str, csv.DictWriter] = {}
        self._columns: Dict[str, Sequence[str]] = {}

    def write(self, filename: str, columns: Sequence[str], row: Dict[str, object]) -> None:
        writer = self._writers.get(filename)
        if writer is None:
            path = self.output_dir / filename
            existed = path.exists() and path.stat().st_size > 0
            handle = path.open("a", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=list(columns), extrasaction="ignore")
            self._files[filename] = handle
            self._writers[filename] = writer
            self._columns[filename] = list(columns)
            if not existed:
                writer.writeheader()

        clean_row = {column: row.get(column, "") for column in self._columns[filename]}
        writer.writerow(clean_row)

    def flush(self) -> None:
        for handle in self._files.values():
            handle.flush()

    def close(self) -> None:
        for handle in self._files.values():
            handle.flush()
            handle.close()
        self._files.clear()
        self._writers.clear()
        self._columns.clear()


def format_time_s(ts: float) -> str:
    return f"{ts:.6f}"


def format_time_iso(ts: float) -> str:
    return datetime.fromtimestamp(ts).isoformat(timespec="milliseconds")


def parse_datagram(data: bytes) -> Tuple[MvnHeader, bytes]:
    if len(data) < HEADER_SIZE:
        raise MvnPacketError(f"datagram too short: {len(data)} bytes")

    (
        packet_id_bytes,
        sample_counter,
        datagram_counter_raw,
        number_of_items,
        time_code_ms,
        character_id,
        body_segment_count,
        prop_count,
        finger_segment_count,
        reserved,
        payload_size,
    ) = HEADER_STRUCT.unpack_from(data, 0)

    packet_id = packet_id_bytes.decode("ascii", errors="replace")
    message_type = packet_id[4:6] if len(packet_id) == 6 else ""
    valid_mxtp = packet_id.startswith("MXTP") and len(packet_id) == 6
    actual_payload_size = max(0, len(data) - HEADER_SIZE)
    payload_size_ok = actual_payload_size >= payload_size
    if not payload_size_ok:
        raise MvnPacketError(
            f"{packet_id}: payload shorter than header declares "
            f"({actual_payload_size} < {payload_size})"
        )

    payload = data[HEADER_SIZE : HEADER_SIZE + payload_size]
    header = MvnHeader(
        packet_id=packet_id,
        message_type=message_type,
        sample_counter=sample_counter,
        datagram_counter_raw=datagram_counter_raw,
        datagram_index=datagram_counter_raw & 0x7F,
        datagram_is_last=bool(datagram_counter_raw & 0x80),
        number_of_items=number_of_items,
        time_code_ms=time_code_ms,
        character_id=character_id,
        body_segment_count=body_segment_count,
        prop_count=prop_count,
        finger_segment_count=finger_segment_count,
        reserved=reserved,
        payload_size=payload_size,
        actual_payload_size=actual_payload_size,
        payload_size_ok=actual_payload_size == payload_size,
        valid_mxtp=valid_mxtp,
    )
    return header, payload


def packet_row(
    part: DatagramPart,
    raw_size_bytes: int,
    parse_status: str,
) -> Dict[str, object]:
    h = part.header
    return {
        "recv_time_s": format_time_s(part.recv_time_s),
        "recv_time_iso": format_time_iso(part.recv_time_s),
        "source_ip": part.source_ip,
        "source_port": part.source_port,
        "packet_seq": part.packet_seq,
        "raw_size_bytes": raw_size_bytes,
        "packet_id": h.packet_id,
        "message_type": h.message_type,
        "valid_mxtp": int(h.valid_mxtp),
        "sample_counter": h.sample_counter,
        "datagram_counter_raw": h.datagram_counter_raw,
        "datagram_index": h.datagram_index,
        "datagram_is_last": int(h.datagram_is_last),
        "number_of_items": h.number_of_items,
        "time_code_ms": h.time_code_ms,
        "character_id": h.character_id,
        "body_segment_count": h.body_segment_count,
        "prop_count": h.prop_count,
        "finger_segment_count": h.finger_segment_count,
        "reserved": h.reserved,
        "payload_size": h.payload_size,
        "actual_payload_size": h.actual_payload_size,
        "payload_size_ok": int(h.payload_size_ok),
        "parse_status": parse_status,
    }


def invalid_packet_row(
    data: bytes,
    recv_time_s: float,
    source_ip: str,
    source_port: int,
    packet_seq: int,
    parse_status: str,
) -> Dict[str, object]:
    packet_id = data[:6].decode("ascii", errors="replace") if data else ""
    return {
        "recv_time_s": format_time_s(recv_time_s),
        "recv_time_iso": format_time_iso(recv_time_s),
        "source_ip": source_ip,
        "source_port": source_port,
        "packet_seq": packet_seq,
        "raw_size_bytes": len(data),
        "packet_id": packet_id,
        "message_type": packet_id[4:6] if len(packet_id) == 6 else "",
        "valid_mxtp": 0,
        "actual_payload_size": max(0, len(data) - HEADER_SIZE),
        "parse_status": parse_status,
    }


def base_detail_row(
    part: DatagramPart,
    sample_complete: bool,
    sample_datagram_count: int,
    missing_datagrams: Sequence[int],
) -> Dict[str, object]:
    h = part.header
    return {
        "recv_time_s": format_time_s(part.recv_time_s),
        "recv_time_iso": format_time_iso(part.recv_time_s),
        "source_ip": part.source_ip,
        "source_port": part.source_port,
        "packet_seq": part.packet_seq,
        "packet_id": h.packet_id,
        "message_type": h.message_type,
        "sample_counter": h.sample_counter,
        "time_code_ms": h.time_code_ms,
        "character_id": h.character_id,
        "datagram_index": h.datagram_index,
        "datagram_is_last": int(h.datagram_is_last),
        "datagram_items": h.number_of_items,
        "body_segment_count": h.body_segment_count,
        "prop_count": h.prop_count,
        "finger_segment_count": h.finger_segment_count,
        "payload_size": h.payload_size,
        "sample_complete": int(sample_complete),
        "sample_datagram_count": sample_datagram_count,
        "missing_datagrams": " ".join(str(i) for i in missing_datagrams),
    }


def expected_record_bytes(message_type: str) -> Optional[int]:
    return {
        "01": 28,
        "02": 32,
        "03": 16,
        "05": 32,
        "20": 20,
        "21": 40,
        "22": 44,
        "23": 68,
        "24": 12,
        "25": 12,
    }.get(message_type)


def payload_warning(header: MvnHeader, expected_bytes_per_item: Optional[int]) -> str:
    warnings: List[str] = []
    if expected_bytes_per_item is not None:
        expected = expected_bytes_per_item * header.number_of_items
        if header.payload_size != expected:
            warnings.append(f"payload_size {header.payload_size} != expected {expected}")
    if not header.payload_size_ok:
        warnings.append(
            f"actual_payload_size {header.actual_payload_size} != declared {header.payload_size}"
        )
    return "; ".join(warnings)


def position_columns_from_m(x_m: float, y_m: float, z_m: float) -> Dict[str, str]:
    return {
        "x_cm": f"{x_m * 100.0:.9g}",
        "y_cm": f"{y_m * 100.0:.9g}",
        "z_cm": f"{z_m * 100.0:.9g}",
        "x_m": f"{x_m:.9g}",
        "y_m": f"{y_m:.9g}",
        "z_m": f"{z_m:.9g}",
    }


def infer_segment_id_base(
    segment_ids: Sequence[int],
    body_segment_count: int,
    requested: str,
) -> str:
    if requested != "auto":
        return requested
    if not segment_ids:
        return "one"
    first_body_ids = set(segment_ids[:body_segment_count])
    if 0 in first_body_ids:
        return "zero"
    if body_segment_count in first_body_ids:
        return "one"
    if min(first_body_ids) == 1 and max(first_body_ids) == body_segment_count:
        return "one"
    return "one"


def segment_index_from_id(segment_id: int, segment_id_base: str) -> int:
    if segment_id_base == "zero":
        return segment_id
    return segment_id - 1


def classify_by_order(order_index: int, header: MvnHeader) -> Tuple[str, str, str]:
    body_count = header.body_segment_count
    prop_count = header.prop_count
    finger_count = header.finger_segment_count
    left_count = finger_count // 2

    if order_index < body_count:
        name = BODY_SEGMENTS[order_index] if order_index < len(BODY_SEGMENTS) else f"Body {order_index}"
        return "body", name, ""

    prop_index = order_index - body_count
    if prop_index < prop_count:
        return "prop", f"Prop {prop_index + 1}", ""

    finger_index = order_index - body_count - prop_count
    if 0 <= finger_index < left_count:
        name = FINGER_SEGMENTS[finger_index] if finger_index < len(FINGER_SEGMENTS) else f"Finger {finger_index}"
        return "left_finger", f"Left {name}", "left"

    right_index = finger_index - left_count
    if 0 <= right_index < finger_count - left_count:
        name = FINGER_SEGMENTS[right_index] if right_index < len(FINGER_SEGMENTS) else f"Finger {right_index}"
        return "right_finger", f"Right {name}", "right"

    return "unknown", f"Item {order_index}", ""


def segment_name_from_index(segment_index: int, header: MvnHeader) -> str:
    if 0 <= segment_index < len(BODY_SEGMENTS):
        return BODY_SEGMENTS[segment_index]

    # Prop and finger ids in the public protocol are easiest to interpret from
    # the stream order.  This fallback labels obvious prop id ranges and then
    # tries the documented finger index offsets.
    if 24 <= segment_index <= 27:
        return f"Prop {segment_index - 23}"

    left_start = 23 + header.prop_count
    right_start = 43 + header.prop_count
    if left_start <= segment_index < left_start + len(FINGER_SEGMENTS):
        return "Left " + FINGER_SEGMENTS[segment_index - left_start]
    if right_start <= segment_index < right_start + len(FINGER_SEGMENTS):
        return "Right " + FINGER_SEGMENTS[segment_index - right_start]
    return ""


def pose_coordinate_system(message_type: str) -> str:
    if message_type == "01":
        return "Y-Up right-handed"
    if message_type == "05":
        return "Y-Up left-handed"
    return "Z-Up right-handed"


def parse_pose_records(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
) -> None:
    parsed: List[Tuple[DatagramPart, int, int, Tuple[object, ...]]] = []
    order_index = 0
    message_type = parts[0].header.message_type
    fmt = struct.Struct(">Iffffff") if message_type == "01" else struct.Struct(">Ifffffff")
    warning_by_part: Dict[int, str] = {}

    for part in parts:
        warning_by_part[part.packet_seq] = payload_warning(part.header, fmt.size)
        max_items = min(part.header.number_of_items, len(part.payload) // fmt.size)
        for local_index in range(max_items):
            record = fmt.unpack_from(part.payload, local_index * fmt.size)
            parsed.append((part, local_index, order_index, record))
            order_index += 1

    segment_ids = [int(record[0]) for _, _, _, record in parsed]
    segment_id_base = infer_segment_id_base(
        segment_ids,
        parts[0].header.body_segment_count,
        segment_id_base_arg,
    )

    for part, _local_index, global_index, record in parsed:
        h = part.header
        base = base_detail_row(part, sample_complete, len(parts), missing_datagrams)
        segment_id = int(record[0])
        segment_index = segment_index_from_id(segment_id, segment_id_base)
        item_kind, item_name, finger_side = classify_by_order(global_index, h)
        row = {
            **base,
            "item_order_index": global_index,
            "item_kind": item_kind,
            "item_name": item_name,
            "finger_side": finger_side,
            "segment_id": segment_id,
            "segment_index": segment_index,
            "segment_name_from_id": segment_name_from_index(segment_index, h),
            "segment_id_base": segment_id_base,
            "coordinate_system": pose_coordinate_system(h.message_type),
            "position_unit": POSITION_UNIT,
            "rotation_format": "euler_deg" if h.message_type == "01" else "quaternion_wxyz",
            **position_columns_from_m(float(record[1]), float(record[2]), float(record[3])),
            "parse_warning": warning_by_part.get(part.packet_seq, ""),
        }
        if h.message_type == "01":
            row.update(
                {
                    "rot_x_deg": f"{float(record[4]):.9g}",
                    "rot_y_deg": f"{float(record[5]):.9g}",
                    "rot_z_deg": f"{float(record[6]):.9g}",
                }
            )
        else:
            row.update(
                {
                    "q_w": f"{float(record[4]):.9g}",
                    "q_x": f"{float(record[5]):.9g}",
                    "q_y": f"{float(record[6]):.9g}",
                    "q_z": f"{float(record[7]):.9g}",
                }
            )
        sink.write("mvn_pose_segments.csv", POSE_COLUMNS, row)
        stats["pose_rows"] += 1


def parse_point_records(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
) -> None:
    fmt = struct.Struct(">Ifff")
    order_index = 0
    for part in parts:
        warning = payload_warning(part.header, fmt.size)
        max_items = min(part.header.number_of_items, len(part.payload) // fmt.size)
        for local_index in range(max_items):
            point_id, x, y, z = fmt.unpack_from(part.payload, local_index * fmt.size)
            row = {
                **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                "item_order_index": order_index,
                "point_id": point_id,
                **position_columns_from_m(x, y, z),
                "coordinate_system": "Y-Up right-handed",
                "parse_warning": warning,
            }
            sink.write("mvn_points.csv", POINT_COLUMNS, row)
            stats["point_rows"] += 1
            order_index += 1


def parse_joint_angle_records(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
) -> None:
    fmt = struct.Struct(">IIfff")
    order_index = 0
    for part in parts:
        warning = payload_warning(part.header, fmt.size)
        max_items = min(part.header.number_of_items, len(part.payload) // fmt.size)
        for local_index in range(max_items):
            parent_id, child_id, rx, ry, rz = fmt.unpack_from(part.payload, local_index * fmt.size)
            row = {
                **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                "item_order_index": order_index,
                "parent_point_id": parent_id,
                "child_point_id": child_id,
                "rot_x_deg": f"{rx:.9g}",
                "rot_y_deg": f"{ry:.9g}",
                "rot_z_deg": f"{rz:.9g}",
                "coordinate_system": "Z-Up right-handed",
                "parse_warning": warning,
            }
            sink.write("mvn_joint_angles.csv", JOINT_ANGLE_COLUMNS, row)
            stats["joint_angle_rows"] += 1
            order_index += 1


def parse_linear_kinematics_records(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
) -> None:
    fmt = struct.Struct(">Ifffffffff")
    parsed: List[Tuple[DatagramPart, int, Tuple[object, ...]]] = []
    for part in parts:
        max_items = min(part.header.number_of_items, len(part.payload) // fmt.size)
        for local_index in range(max_items):
            parsed.append((part, len(parsed), fmt.unpack_from(part.payload, local_index * fmt.size)))

    segment_id_base = infer_segment_id_base(
        [int(record[0]) for _, _, record in parsed],
        parts[0].header.body_segment_count,
        segment_id_base_arg,
    )
    for part, order_index, record in parsed:
        h = part.header
        segment_id = int(record[0])
        segment_index = segment_index_from_id(segment_id, segment_id_base)
        item_kind, item_name, _finger_side = classify_by_order(order_index, h)
        x, y, z = float(record[1]), float(record[2]), float(record[3])
        row = {
            **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
            "item_order_index": order_index,
            "item_kind": item_kind,
            "item_name": item_name,
            "segment_id": segment_id,
            "segment_index": segment_index,
            "segment_name_from_id": segment_name_from_index(segment_index, h),
            "segment_id_base": segment_id_base,
            **position_columns_from_m(x, y, z),
            "vel_x_cm_s": f"{float(record[4]) * 100.0:.9g}",
            "vel_y_cm_s": f"{float(record[5]) * 100.0:.9g}",
            "vel_z_cm_s": f"{float(record[6]) * 100.0:.9g}",
            "acc_x_cm_s2": f"{float(record[7]) * 100.0:.9g}",
            "acc_y_cm_s2": f"{float(record[8]) * 100.0:.9g}",
            "acc_z_cm_s2": f"{float(record[9]) * 100.0:.9g}",
            "coordinate_system": "Z-Up right-handed",
            "parse_warning": payload_warning(h, fmt.size),
        }
        sink.write("mvn_linear_kinematics.csv", LINEAR_KINEMATICS_COLUMNS, row)
        stats["linear_kinematics_rows"] += 1


def parse_angular_kinematics_records(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
    tracker: bool = False,
) -> None:
    fmt = struct.Struct(">Iffffffffffffffff") if tracker else struct.Struct(">Iffffffffff")
    parsed: List[Tuple[DatagramPart, int, Tuple[object, ...]]] = []
    for part in parts:
        max_items = min(part.header.number_of_items, len(part.payload) // fmt.size)
        for local_index in range(max_items):
            parsed.append((part, len(parsed), fmt.unpack_from(part.payload, local_index * fmt.size)))

    segment_id_base = infer_segment_id_base(
        [int(record[0]) for _, _, record in parsed],
        parts[0].header.body_segment_count,
        segment_id_base_arg,
    )

    for part, order_index, record in parsed:
        h = part.header
        segment_id = int(record[0])
        segment_index = segment_index_from_id(segment_id, segment_id_base)
        if tracker:
            row = {
                **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                "item_order_index": order_index,
                "attached_segment_id": segment_id,
                "attached_segment_index": segment_index,
                "attached_segment_name": segment_name_from_index(segment_index, h),
                "segment_id_base": segment_id_base,
                "q_w": f"{float(record[1]):.9g}",
                "q_x": f"{float(record[2]):.9g}",
                "q_y": f"{float(record[3]):.9g}",
                "q_z": f"{float(record[4]):.9g}",
                "free_acc_x": f"{float(record[5]):.9g}",
                "free_acc_y": f"{float(record[6]):.9g}",
                "free_acc_z": f"{float(record[7]):.9g}",
                "local_acc_x": f"{float(record[8]):.9g}",
                "local_acc_y": f"{float(record[9]):.9g}",
                "local_acc_z": f"{float(record[10]):.9g}",
                "angular_vel_x": f"{float(record[11]):.9g}",
                "angular_vel_y": f"{float(record[12]):.9g}",
                "angular_vel_z": f"{float(record[13]):.9g}",
                "mag_x": f"{float(record[14]):.9g}",
                "mag_y": f"{float(record[15]):.9g}",
                "mag_z": f"{float(record[16]):.9g}",
                "coordinate_system": "Z-Up right-handed",
                "parse_warning": payload_warning(h, fmt.size),
            }
            sink.write("mvn_tracker_kinematics.csv", TRACKER_KINEMATICS_COLUMNS, row)
            stats["tracker_kinematics_rows"] += 1
        else:
            item_kind, item_name, _finger_side = classify_by_order(order_index, h)
            row = {
                **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                "item_order_index": order_index,
                "item_kind": item_kind,
                "item_name": item_name,
                "segment_id": segment_id,
                "segment_index": segment_index,
                "segment_name_from_id": segment_name_from_index(segment_index, h),
                "segment_id_base": segment_id_base,
                "q_w": f"{float(record[1]):.9g}",
                "q_x": f"{float(record[2]):.9g}",
                "q_y": f"{float(record[3]):.9g}",
                "q_z": f"{float(record[4]):.9g}",
                "ang_vel_x": f"{float(record[5]):.9g}",
                "ang_vel_y": f"{float(record[6]):.9g}",
                "ang_vel_z": f"{float(record[7]):.9g}",
                "ang_acc_x": f"{float(record[8]):.9g}",
                "ang_acc_y": f"{float(record[9]):.9g}",
                "ang_acc_z": f"{float(record[10]):.9g}",
                "coordinate_system": "Z-Up right-handed",
                "parse_warning": payload_warning(h, fmt.size),
            }
            sink.write("mvn_angular_kinematics.csv", ANGULAR_KINEMATICS_COLUMNS, row)
            stats["angular_kinematics_rows"] += 1


def parse_center_of_mass(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
) -> None:
    fmt = struct.Struct(">fff")
    for part in parts:
        warning = payload_warning(part.header, fmt.size)
        if len(part.payload) < fmt.size:
            warning = (warning + "; " if warning else "") + "payload too short"
            x = y = z = 0.0
        else:
            x, y, z = fmt.unpack_from(part.payload, 0)
        row = {
            **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
            **position_columns_from_m(x, y, z),
            "coordinate_system": "Z-Up right-handed",
            "parse_warning": warning,
        }
        sink.write("mvn_center_of_mass.csv", COM_COLUMNS, row)
        stats["center_of_mass_rows"] += 1


def parse_timecode(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
) -> None:
    for part in parts:
        warning = ""
        try:
            timecode, offset = read_mvn_string(part.payload, 0)
            if offset != len(part.payload):
                warning = f"{len(part.payload) - offset} trailing bytes"
        except MvnPacketError as exc:
            warning = str(exc)
            timecode = part.payload.decode("ascii", errors="replace")
        row = {
            **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
            "timecode": timecode,
            "parse_warning": warning,
        }
        sink.write("mvn_timecode.csv", TIMECODE_COLUMNS, row)
        stats["timecode_rows"] += 1


def parse_metadata(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
) -> None:
    for part in parts:
        warning = ""
        try:
            text, offset = read_mvn_string(part.payload, 0)
            if offset != len(part.payload):
                warning = f"{len(part.payload) - offset} trailing bytes"
        except MvnPacketError as exc:
            warning = str(exc)
            text = part.payload.decode("utf-8", errors="replace")
        lines = [line for line in text.splitlines() if line]
        if not lines:
            lines = [""]
        for line in lines:
            tag, value = ("", line)
            if ":" in line:
                tag, value = line.split(":", 1)
            row = {
                **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                "tag": tag,
                "value": value,
                "raw_text": text,
                "parse_warning": warning,
            }
            sink.write("mvn_metadata.csv", METADATA_COLUMNS, row)
            stats["metadata_rows"] += 1


def read_mvn_string(payload: bytes, offset: int) -> Tuple[str, int]:
    if offset + 4 > len(payload):
        raise MvnPacketError("payload ended before string length")
    (length,) = struct.unpack_from(">i", payload, offset)
    offset += 4
    if length < 0:
        raise MvnPacketError(f"negative string length: {length}")
    if offset + length > len(payload):
        raise MvnPacketError("payload ended inside string")
    text = payload[offset : offset + length].decode("utf-8", errors="replace")
    return text, offset + length


def parse_scale_information(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
) -> None:
    for part in parts:
        warning = ""
        offset = 0
        try:
            if len(part.payload) < 4:
                raise MvnPacketError("payload too short for segment count")
            (segment_count,) = struct.unpack_from(">I", part.payload, offset)
            offset += 4
            for i in range(segment_count):
                name, offset = read_mvn_string(part.payload, offset)
                x, y, z = struct.unpack_from(">fff", part.payload, offset)
                offset += 12
                row = {
                    **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                    "segment_order_index": i,
                    "segment_name": name,
                    **position_columns_from_m(x, y, z),
                    "coordinate_system": "Z-Up right-handed",
                    "parse_warning": warning,
                }
                sink.write("mvn_scale_segments.csv", SCALE_SEGMENT_COLUMNS, row)
                stats["scale_segment_rows"] += 1

            if offset + 4 > len(part.payload):
                raise MvnPacketError("payload too short for point count")
            (point_count,) = struct.unpack_from(">I", part.payload, offset)
            offset += 4
            for i in range(point_count):
                segment_id, point_id = struct.unpack_from(">HH", part.payload, offset)
                offset += 4
                name, offset = read_mvn_string(part.payload, offset)
                flags, x, y, z = struct.unpack_from(">Ifff", part.payload, offset)
                offset += 16
                row = {
                    **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                    "point_order_index": i,
                    "segment_id": segment_id,
                    "point_id": point_id,
                    "point_name": name,
                    "flags": flags,
                    **position_columns_from_m(x, y, z),
                    "coordinate_system": "Z-Up right-handed",
                    "parse_warning": warning,
                }
                sink.write("mvn_scale_points.csv", SCALE_POINT_COLUMNS, row)
                stats["scale_point_rows"] += 1
        except (struct.error, MvnPacketError) as exc:
            row = {
                **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
                "payload_hex_prefix": part.payload[:64].hex(),
                "payload_len": len(part.payload),
                "parse_warning": str(exc),
            }
            sink.write("mvn_unknown_payloads.csv", UNKNOWN_COLUMNS, row)
            stats["unknown_rows"] += 1


def parse_unknown(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    missing_datagrams: Sequence[int],
    sink: CsvSink,
    stats: Counter,
    reason: str,
) -> None:
    for part in parts:
        row = {
            **base_detail_row(part, sample_complete, len(parts), missing_datagrams),
            "payload_hex_prefix": part.payload[:64].hex(),
            "payload_len": len(part.payload),
            "parse_warning": reason,
        }
        sink.write("mvn_unknown_payloads.csv", UNKNOWN_COLUMNS, row)
        stats["unknown_rows"] += 1


def decode_sample(
    parts: Sequence[DatagramPart],
    sample_complete: bool,
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
) -> None:
    if not parts:
        return
    ordered = sorted(parts, key=lambda p: p.header.datagram_index)
    present = {part.header.datagram_index for part in ordered}
    last_indices = [part.header.datagram_index for part in ordered if part.header.datagram_is_last]
    if sample_complete or last_indices:
        last = max(last_indices) if last_indices else ordered[-1].header.datagram_index
        missing = [i for i in range(last + 1) if i not in present]
    else:
        missing = []

    message_type = ordered[0].header.message_type
    try:
        if message_type in {"01", "02", "05"}:
            parse_pose_records(
                ordered,
                sample_complete,
                missing,
                segment_id_base_arg,
                sink,
                stats,
            )
            stats["pose_samples"] += 1
        elif message_type == "03":
            parse_point_records(ordered, sample_complete, missing, sink, stats)
        elif message_type == "12":
            parse_metadata(ordered, sample_complete, missing, sink, stats)
        elif message_type == "13":
            parse_scale_information(ordered, sample_complete, missing, sink, stats)
        elif message_type == "20":
            parse_joint_angle_records(ordered, sample_complete, missing, sink, stats)
        elif message_type == "21":
            parse_linear_kinematics_records(
                ordered,
                sample_complete,
                missing,
                segment_id_base_arg,
                sink,
                stats,
            )
        elif message_type == "22":
            parse_angular_kinematics_records(
                ordered,
                sample_complete,
                missing,
                segment_id_base_arg,
                sink,
                stats,
                tracker=False,
            )
        elif message_type == "23":
            parse_angular_kinematics_records(
                ordered,
                sample_complete,
                missing,
                segment_id_base_arg,
                sink,
                stats,
                tracker=True,
            )
        elif message_type == "24":
            parse_center_of_mass(ordered, sample_complete, missing, sink, stats)
        elif message_type == "25":
            parse_timecode(ordered, sample_complete, missing, sink, stats)
        else:
            parse_unknown(ordered, sample_complete, missing, sink, stats, "unsupported message type")
    except (struct.error, MvnPacketError) as exc:
        parse_unknown(ordered, sample_complete, missing, sink, stats, str(exc))

    stats["samples_decoded"] += 1


def sample_key(header: MvnHeader) -> Tuple[str, int, int]:
    return (header.packet_id, header.character_id, header.sample_counter)


def handle_part(
    part: DatagramPart,
    pending: Dict[Tuple[str, int, int], PendingSample],
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
) -> None:
    h = part.header
    if h.datagram_is_last and h.datagram_index == 0:
        decode_sample([part], True, segment_id_base_arg, sink, stats)
        return

    key = sample_key(h)
    state = pending.get(key)
    if state is None:
        state = PendingSample(first_seen_s=part.recv_time_s, parts={})
        pending[key] = state
    if h.datagram_index in state.parts:
        stats["duplicate_datagrams"] += 1
    state.parts[h.datagram_index] = part
    if h.datagram_is_last:
        state.last_index = h.datagram_index

    if state.last_index is not None:
        expected = set(range(state.last_index + 1))
        if expected.issubset(state.parts):
            parts = [state.parts[i] for i in sorted(expected)]
            decode_sample(parts, True, segment_id_base_arg, sink, stats)
            del pending[key]


def flush_stale_pending(
    pending: Dict[Tuple[str, int, int], PendingSample],
    now_s: float,
    timeout_s: float,
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
) -> None:
    stale_keys = [
        key for key, state in pending.items() if now_s - state.first_seen_s >= timeout_s
    ]
    for key in stale_keys:
        state = pending.pop(key)
        decode_sample(
            list(state.parts.values()),
            False,
            segment_id_base_arg,
            sink,
            stats,
        )
        stats["incomplete_samples_flushed"] += 1


def flush_all_pending(
    pending: Dict[Tuple[str, int, int], PendingSample],
    segment_id_base_arg: str,
    sink: CsvSink,
    stats: Counter,
) -> None:
    for key in list(pending.keys()):
        state = pending.pop(key)
        decode_sample(list(state.parts.values()), False, segment_id_base_arg, sink, stats)
        stats["incomplete_samples_flushed"] += 1


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    default_output = Path("output/xsens_mvn_udp") / datetime.now().strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="0.0.0.0", help="Local interface to bind.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port from MVN.")
    parser.add_argument("--output-dir", type=Path, default=default_output)
    parser.add_argument("--accept-from", default=None, help="Optional sender IP filter.")
    parser.add_argument("--duration-s", type=float, default=None, help="Stop after this many seconds.")
    parser.add_argument("--max-packets", type=int, default=None, help="Stop after this many UDP packets.")
    parser.add_argument("--max-samples", type=int, default=None, help="Stop after this many decoded samples.")
    parser.add_argument("--socket-buffer-bytes", type=int, default=4 * 1024 * 1024)
    parser.add_argument("--recv-bytes", type=int, default=65535)
    parser.add_argument("--pending-timeout-s", type=float, default=1.0)
    parser.add_argument("--flush-every", type=int, default=100)
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument(
        "--segment-id-base",
        choices=["auto", "zero", "one"],
        default="auto",
        help="Override segment id interpretation if your MVN version differs.",
    )
    parser.add_argument("--self-test", action="store_true", help="Run a synthetic parser smoke test.")
    return parser.parse_args(argv)


def open_udp_socket(args: argparse.Namespace) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if args.socket_buffer_bytes:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.socket_buffer_bytes)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(0.2)
    return sock


def should_stop(args: argparse.Namespace, stats: Counter, start_s: float) -> bool:
    if args.duration_s is not None and time.monotonic() - start_s >= args.duration_s:
        return True
    if args.max_packets is not None and stats["packets"] >= args.max_packets:
        return True
    if args.max_samples is not None and stats["samples_decoded"] >= args.max_samples:
        return True
    return False


def print_status(stats: Counter, output_dir: Path) -> None:
    print(
        "[MVN] packets={packets} samples={samples} pose_rows={pose_rows} "
        "incomplete={incomplete} out={out}".format(
            packets=stats["packets"],
            samples=stats["samples_decoded"],
            pose_rows=stats["pose_rows"],
            incomplete=stats["incomplete_samples_flushed"],
            out=output_dir,
        ),
        flush=True,
    )


def capture(args: argparse.Namespace) -> Counter:
    stop_requested = False

    def _request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    sink = CsvSink(args.output_dir)
    pending: Dict[Tuple[str, int, int], PendingSample] = {}
    stats: Counter = Counter()
    start_s = time.monotonic()

    print(f"[MVN] Listening on {args.bind_ip}:{args.port}")
    print(f"[MVN] Writing CSV files under {args.output_dir}")

    sock = open_udp_socket(args)
    try:
        while not stop_requested and not should_stop(args, stats, start_s):
            try:
                data, (source_ip, source_port) = sock.recvfrom(args.recv_bytes)
            except socket.timeout:
                flush_stale_pending(
                    pending,
                    time.time(),
                    args.pending_timeout_s,
                    args.segment_id_base,
                    sink,
                    stats,
                )
                continue

            if args.accept_from and source_ip != args.accept_from:
                stats["packets_ignored_by_source"] += 1
                continue

            recv_time_s = time.time()
            stats["packets"] += 1
            packet_seq = stats["packets"]

            try:
                header, payload = parse_datagram(data)
                part = DatagramPart(
                    header=header,
                    payload=payload,
                    recv_time_s=recv_time_s,
                    source_ip=source_ip,
                    source_port=source_port,
                    packet_seq=packet_seq,
                )
                sink.write("mvn_packets.csv", PACKET_COLUMNS, packet_row(part, len(data), "ok"))
                if not header.valid_mxtp:
                    stats["invalid_mxtp"] += 1
                    continue
                handle_part(part, pending, args.segment_id_base, sink, stats)
            except MvnPacketError as exc:
                sink.write(
                    "mvn_packets.csv",
                    PACKET_COLUMNS,
                    invalid_packet_row(
                        data,
                        recv_time_s,
                        source_ip,
                        source_port,
                        packet_seq,
                        str(exc),
                    ),
                )
                stats["packet_parse_errors"] += 1

            flush_stale_pending(
                pending,
                recv_time_s,
                args.pending_timeout_s,
                args.segment_id_base,
                sink,
                stats,
            )
            if args.flush_every > 0 and packet_seq % args.flush_every == 0:
                sink.flush()
            if args.print_every > 0 and packet_seq % args.print_every == 0:
                print_status(stats, args.output_dir)
    finally:
        flush_all_pending(pending, args.segment_id_base, sink, stats)
        sink.flush()
        sink.close()
        sock.close()

    print_status(stats, args.output_dir)
    return stats


def make_synthetic_packet() -> bytes:
    records = [
        struct.pack(">Ifffffff", 1, 10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0),
        struct.pack(">Ifffffff", 2, 11.0, 21.0, 31.0, 0.0, 1.0, 0.0, 0.0),
    ]
    payload = b"".join(records)
    header = HEADER_STRUCT.pack(
        b"MXTP02",
        7,
        0x80,
        2,
        123,
        0,
        2,
        0,
        0,
        0,
        len(payload),
    )
    return header + payload


def make_synthetic_split_packets() -> Tuple[bytes, bytes]:
    records = [
        struct.pack(">Ifffffff", 1, 10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0),
        struct.pack(">Ifffffff", 2, 11.0, 21.0, 31.0, 0.0, 1.0, 0.0, 0.0),
        struct.pack(">Ifffffff", 24, 12.0, 22.0, 32.0, 0.0, 0.0, 1.0, 0.0),
    ]
    payload0 = b"".join(records[:2])
    payload1 = records[2]
    header0 = HEADER_STRUCT.pack(
        b"MXTP02",
        8,
        0x00,
        2,
        456,
        0,
        2,
        0,
        2,
        0,
        len(payload0),
    )
    header1 = HEADER_STRUCT.pack(
        b"MXTP02",
        8,
        0x81,
        1,
        456,
        0,
        2,
        0,
        2,
        0,
        len(payload1),
    )
    return header0 + payload0, header1 + payload1


def run_self_test() -> None:
    with tempfile.TemporaryDirectory(prefix="mvn_udp_test_") as tmp:
        output_dir = Path(tmp)
        sink = CsvSink(output_dir)
        stats: Counter = Counter()
        header, payload = parse_datagram(make_synthetic_packet())
        part = DatagramPart(
            header=header,
            payload=payload,
            recv_time_s=time.time(),
            source_ip="127.0.0.1",
            source_port=9763,
            packet_seq=1,
        )
        decode_sample([part], True, "auto", sink, stats)
        sink.close()
        pose_path = output_dir / "mvn_pose_segments.csv"
        rows = list(csv.DictReader(pose_path.open("r", newline="", encoding="utf-8")))
        if len(rows) != 2:
            raise AssertionError(f"expected 2 pose rows, got {len(rows)}")
        if rows[0]["item_name"] != "Pelvis":
            raise AssertionError(f"expected Pelvis, got {rows[0]['item_name']}")
        if rows[0]["q_w"] != "1":
            raise AssertionError(f"expected q_w=1, got {rows[0]['q_w']}")

        output_dir_2 = output_dir / "split"
        sink = CsvSink(output_dir_2)
        stats = Counter()
        pending: Dict[Tuple[str, int, int], PendingSample] = {}
        for packet_seq, packet in enumerate(make_synthetic_split_packets(), start=1):
            header, payload = parse_datagram(packet)
            handle_part(
                DatagramPart(
                    header=header,
                    payload=payload,
                    recv_time_s=time.time(),
                    source_ip="127.0.0.1",
                    source_port=9763,
                    packet_seq=packet_seq,
                ),
                pending,
                "auto",
                sink,
                stats,
            )
        sink.close()
        rows = list(csv.DictReader((output_dir_2 / "mvn_pose_segments.csv").open("r", newline="", encoding="utf-8")))
        if len(rows) != 3:
            raise AssertionError(f"expected 3 split pose rows, got {len(rows)}")
        if rows[-1]["item_kind"] != "left_finger":
            raise AssertionError(f"expected split item left_finger, got {rows[-1]['item_kind']}")
    print("[MVN] self-test passed")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    capture(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
