MID360_LINE_COUNT = 4
MID360_DEFAULT_TAG = 0


def line_for_output_index(index, line_count=MID360_LINE_COUNT):
    return int(index) % max(1, int(line_count))


def relative_time_seconds(source_index, source_count, scan_period_s):
    denominator = max(1, int(source_count) - 1)
    ratio = min(max(int(source_index), 0), denominator) / denominator
    return ratio * max(0.0, float(scan_period_s))
