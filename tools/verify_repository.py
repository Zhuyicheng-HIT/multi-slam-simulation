#!/usr/bin/env python3
from pathlib import Path
import os
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_DIRS = {
    "build", "install", "log", "logs", "external", "__pycache__",
    ".codex_backups", ".venv", "venv",
}
MAX_FILE_BYTES = 25 * 1024 * 1024
PERSONAL_PATHS = [
    re.compile(rb"/home/[A-Za-z0-9_.-]+/(?:multi-slam|LiDAR)(?:/|\b)"),
    re.compile(rb"[A-Za-z]:\\Users\\[A-Za-z0-9_.-]+\\"),
]


def tracked_files():
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    )
    for raw_path in result.stdout.split(b"\0"):
        if raw_path:
            yield ROOT / raw_path.decode("utf-8")


def main():
    errors = []
    for path in tracked_files():
        relative = path.relative_to(ROOT)
        if path.is_symlink():
            target = os.readlink(path)
            if Path(target).is_absolute():
                errors.append(f"absolute symbolic link: {relative} -> {target}")
            continue
        if not path.is_file():
            errors.append(f"unsupported repository entry: {relative}")
            continue
        if any(part in FORBIDDEN_DIRS for part in relative.parts):
            errors.append(f"forbidden directory: {relative}")
            continue
        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            errors.append(f"file exceeds 25 MiB: {relative} ({size} bytes)")
        data = path.read_bytes()
        for pattern in PERSONAL_PATHS:
            if pattern.search(data):
                errors.append(f"personal absolute path: {relative}")
                break
    if errors:
        print("Repository verification failed:")
        for error in errors:
            print(f"  - {error}")
        return 1
    print("Repository verification passed: no forbidden directories, personal paths, or files over 25 MiB.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
