import copy
import math

import numpy as np


def shift_stamp(stamp, offset_s):
    total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    total_ns = max(0, total_ns + int(round(offset_s * 1.0e9)))
    stamp.sec, stamp.nanosec = divmod(total_ns, 1_000_000_000)


def ensure_monotonic_stamp(stamp, last_stamp_ns):
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    repaired = stamp_ns <= last_stamp_ns and stamp_ns != 0
    if repaired:
        stamp_ns = last_stamp_ns + 1
        stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    return max(last_stamp_ns, stamp_ns), repaired


def drop_pointcloud(msg, fraction, rng):
    output = copy.deepcopy(msg)
    count = min(int(msg.width) * int(msg.height), len(msg.data) // max(1, int(msg.point_step)))
    if count <= 0 or fraction <= 0.0:
        return output
    keep = rng.random(count) >= min(1.0, fraction)
    chunks = [
        msg.data[index * int(msg.point_step):(index + 1) * int(msg.point_step)]
        for index in range(count) if keep[index]
    ]
    output.height = 1
    output.width = len(chunks)
    output.row_step = int(output.point_step) * int(output.width)
    output.data = b"".join(chunks)
    output.is_dense = False
    return output


def add_gnss_jump(msg, north_m, east_m=0.0):
    output = copy.deepcopy(msg)
    earth_radius_m = 6_378_137.0
    output.latitude += math.degrees(north_m / earth_radius_m)
    longitude_scale = max(1.0e-6, math.cos(math.radians(output.latitude)))
    output.longitude += math.degrees(east_m / (earth_radius_m * longitude_scale))
    return output


def add_depth_holes(msg, fraction, rng):
    output = copy.deepcopy(msg)
    if fraction <= 0.0 or not msg.data:
        return output
    if msg.encoding in ("16UC1", "mono16"):
        values = np.frombuffer(msg.data, dtype=np.uint16).copy()
    elif msg.encoding == "32FC1":
        values = np.frombuffer(msg.data, dtype=np.float32).copy()
    else:
        return output
    values[rng.random(values.size) < min(1.0, fraction)] = 0
    output.data = values.tobytes()
    return output


def flatten_image(msg, level=127):
    output = copy.deepcopy(msg)
    if msg.encoding in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
        output.data = bytes([max(0, min(255, int(level)))]) * len(msg.data)
    return output
