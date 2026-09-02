"""Deterministic streaming PCD I/O for large offline map products."""

from pathlib import Path
import shutil

import numpy as np


def _header(point_count):
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {point_count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {point_count}\n"
        "DATA binary\n"
    ).encode("ascii")


class StreamingBinaryPcdWriter:
    def __init__(self, path):
        self.path = Path(path)
        if self.path.exists():
            raise FileExistsError(self.path)
        self.body_path = self.path.with_suffix(self.path.suffix + ".body.tmp")
        if self.body_path.exists():
            raise FileExistsError(self.body_path)
        self.body = self.body_path.open("wb")
        self.point_count = 0
        self.closed = False

    def append(self, points):
        if self.closed:
            raise RuntimeError("PCD writer is closed")
        values = np.asarray(points)
        if values.ndim != 2 or values.shape[1] not in (3, 4):
            raise ValueError("PCD points must be Nx3 or Nx4")
        output = np.zeros((len(values), 4), dtype="<f4")
        output[:, :3] = values[:, :3]
        if values.shape[1] == 4:
            output[:, 3] = values[:, 3]
        if not np.all(np.isfinite(output[:, :3])):
            raise ValueError("PCD contains non-finite XYZ")
        self.body.write(output.tobytes(order="C"))
        self.point_count += len(output)

    def close(self, commit=True):
        if self.closed:
            return
        self.closed = True
        self.body.close()
        if not commit:
            self.body_path.unlink(missing_ok=True)
            return
        with self.path.open("xb") as output:
            output.write(_header(self.point_count))
            with self.body_path.open("rb") as body:
                shutil.copyfileobj(body, output, length=1024 * 1024)
        self.body_path.unlink()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, _exception, _traceback):
        self.close(commit=exception_type is None)
        return False


def read_binary_pcd(path):
    payload = Path(path).read_bytes()
    marker = b"DATA binary\n"
    offset = payload.find(marker)
    if offset < 0:
        raise ValueError("only binary PCD is supported")
    header = payload[:offset + len(marker)].decode("ascii")
    fields = {
        line.split(maxsplit=1)[0]: line.split(maxsplit=1)[1]
        for line in header.splitlines()
        if " " in line
    }
    count = int(fields["POINTS"])
    body = payload[offset + len(marker):]
    expected = count * 4 * 4
    if len(body) != expected:
        raise ValueError("binary PCD payload size mismatch")
    return np.frombuffer(body, dtype="<f4").reshape((-1, 4)).copy()
