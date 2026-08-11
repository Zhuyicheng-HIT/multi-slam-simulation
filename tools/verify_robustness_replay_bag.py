#!/usr/bin/env python3
"""Verify two rosbag2 inputs have identical topic-ordered CDR payloads."""

import argparse
from collections import Counter
import hashlib
import json

import rosbag2_py


def digest(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    counts = Counter()
    hashes = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        counts[topic] += 1
        hashes.setdefault(topic, hashlib.sha256()).update(data)
    return dict(counts), {topic: value.hexdigest() for topic, value in hashes.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--derived", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    source_counts, source_hashes = digest(args.source)
    derived_counts, derived_hashes = digest(args.derived)
    report = {
        "source": args.source,
        "derived": args.derived,
        "counts_equal": source_counts == derived_counts,
        "payload_hashes_equal": source_hashes == derived_hashes,
        "topic_counts": source_counts,
        "topic_payload_sha256": source_hashes,
    }
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps({
        "counts_equal": report["counts_equal"],
        "payload_hashes_equal": report["payload_hashes_equal"],
        "topics": len(source_counts),
    }, sort_keys=True))
    raise SystemExit(0 if report["counts_equal"] and report["payload_hashes_equal"] else 1)


if __name__ == "__main__":
    main()
