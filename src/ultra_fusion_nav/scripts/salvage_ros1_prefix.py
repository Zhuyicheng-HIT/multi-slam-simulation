#!/usr/bin/env python3
"""Recover complete ROS1 bag chunks from a truncated download prefix.

The source file is never modified.  Complete chunks are re-indexed into a
new ROS1 bag; an incomplete final chunk is ignored.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from io import BytesIO
from pathlib import Path

from rosbags.rosbag1 import Writer
from rosbags.rosbag1.reader import Header, RecordType, decompressors, normalize_msgtype, read_uint32


def read_connection(outer: Header, data: BytesIO) -> tuple[int, dict[str, str | int | None]]:
    conn_id = outer.get_uint32("conn")
    inner = Header.read(data)
    latching_raw = inner.get_string("latching") if "latching" in inner else ""
    info: dict[str, str | int | None] = {
        "topic": inner.get_string("topic"),
        "msgtype": inner.get_string("type"),
        "md5sum": inner.get_string("md5sum"),
        "msgdef": inner.get_string("message_definition"),
        "callerid": inner.get_string("callerid") if "callerid" in inner else None,
        "latching": int(latching_raw) if latching_raw else None,
    }
    return conn_id, info


def parse_chunk(raw: bytes) -> tuple[list[tuple[int, dict[str, str | int | None]]], list[tuple[int, int, bytes]]]:
    data = BytesIO(raw)
    connections: list[tuple[int, dict[str, str | int | None]]] = []
    messages: list[tuple[int, int, bytes]] = []
    while data.tell() < len(raw):
        try:
            header = Header.read(data)
            op = header.get_uint8("op")
            if op == RecordType.CONNECTION:
                connections.append(read_connection(header, data))
            elif op == RecordType.MSGDATA:
                conn_id = header.get_uint32("conn")
                stamp = header.get_time("time")
                size = read_uint32(data)
                payload = data.read(size)
                if len(payload) != size:
                    break
                messages.append((conn_id, stamp, payload))
            else:
                break
        except Exception:
            break
    return connections, messages


def recover(source: Path, destination: Path) -> dict[str, object]:
    if destination.exists():
        raise FileExistsError(destination)

    chunk_count = 0
    message_count = 0
    topic_counts: defaultdict[str, int] = defaultdict(int)
    connection_info: dict[int, dict[str, str | int | None]] = {}
    writer_connections = {}
    first_stamp: int | None = None
    last_stamp: int | None = None

    with source.open("rb") as src:
        magic = src.readline()
        if magic != b"#ROSBAG V2.0\n":
            raise ValueError("not a ROS1 bag v2.0")
        Header.read(src, RecordType.BAGHEADER)
        pad_size = read_uint32(src)
        src.seek(pad_size, 1)

        with Writer(destination) as writer:
            writer.chunk_threshold = 4 * (1 << 20)
            while True:
                chunk_start = src.tell()
                try:
                    header = Header.read(src)
                    op = header.get_uint8("op")
                    if op == RecordType.IDXDATA:
                        index_size = read_uint32(src)
                        src.seek(index_size, 1)
                        continue
                    if op != RecordType.CHUNK:
                        break
                    compression = header.get_string("compression")
                    compressed_size = read_uint32(src)
                    compressed = src.read(compressed_size)
                    if len(compressed) != compressed_size:
                        break
                    raw = decompressors[compression](compressed)
                except Exception:
                    break

                connections, messages = parse_chunk(raw)
                if not connections and not messages:
                    break
                chunk_count += 1
                for conn_id, info in connections:
                    connection_info[conn_id] = info
                    if conn_id not in writer_connections:
                        writer_connections[conn_id] = writer.add_connection(
                            str(info["topic"]),
                            normalize_msgtype(str(info["msgtype"])),
                            msgdef=str(info["msgdef"]),
                            md5sum=str(info["md5sum"]),
                            callerid=info["callerid"] if isinstance(info["callerid"], str) else None,
                            latching=info["latching"] if isinstance(info["latching"], int) else None,
                        )
                for conn_id, stamp, payload in messages:
                    if conn_id not in writer_connections:
                        continue
                    writer.write(writer_connections[conn_id], stamp, payload)
                    message_count += 1
                    topic_counts[str(connection_info[conn_id]["topic"])] += 1
                    first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
                    last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)

                if chunk_count % 25 == 0:
                    print(f"complete_chunks={chunk_count} messages={message_count} source_offset={chunk_start}")

    if message_count == 0:
        destination.unlink(missing_ok=True)
        raise RuntimeError("no complete ROS1 chunk could be recovered")
    return {
        "source": str(source),
        "source_bytes": source.stat().st_size,
        "destination": str(destination),
        "destination_bytes": destination.stat().st_size,
        "complete_chunks": chunk_count,
        "messages": message_count,
        "topics": dict(sorted(topic_counts.items())),
        "start_ns": first_stamp,
        "end_ns": last_stamp,
        "duration_s": (last_stamp - first_stamp) / 1e9 if first_stamp is not None and last_stamp is not None else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    print(recover(args.source, args.destination))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
