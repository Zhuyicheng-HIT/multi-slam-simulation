
"""Exact-stamp RGB-D pair health accounting helpers."""

from collections import deque


class ExactStampPairHealth:
    """Track complete RGB-D pairs without depending on callback interleaving."""

    def __init__(self, max_pending=120, max_intervals=120,
                 source_window=30):
        self.max_pending = max(2, int(max_pending))
        self.color_arrivals = {}
        self.depth_arrivals = {}
        self.intervals = deque(maxlen=max(2, int(max_intervals)))
        self.paired_stamps = deque()
        self.paired_stamp_set = set()
        self.pair_sequence = 0
        self.observed_pair_count = 0
        self.pair_sequence_gaps = 0
        self.last_external_sequence = None
        self.source_sequence_gaps = 0
        self.last_source_sequence = None
        self.source_deltas = deque(maxlen=max(2, int(source_window)))
        self.last_pair_stamp_ns = None
        self.last_pair_arrival_s = None
        self.last_stamp_delta_ms = None

    def observe(self, stream, stamp_ns, arrival_s):
        """Record one callback and return True only when it completes a new pair."""
        stamp_ns = int(stamp_ns)
        arrival_s = float(arrival_s)
        if stream == "color":
            own = self.color_arrivals
            other = self.depth_arrivals
        elif stream == "depth":
            own = self.depth_arrivals
            other = self.color_arrivals
        else:
            raise ValueError("stream must be 'color' or 'depth'")

        if stamp_ns in self.paired_stamp_set:
            return False
        own[stamp_ns] = arrival_s
        if stamp_ns not in other:
            self._trim_pending()
            return False

        pair_arrival_s = max(own.pop(stamp_ns), other.pop(stamp_ns))
        return self.observe_pair(stamp_ns, pair_arrival_s)

    def observe_pair(self, stamp_ns, arrival_s, sequence=None,
                     source_sequence=None):
        """Record a lightweight transport record for one published RGB-D pair."""
        stamp_ns = int(stamp_ns)
        arrival_s = float(arrival_s)
        if stamp_ns in self.paired_stamp_set:
            return False
        if self.last_pair_arrival_s is not None:
            interval = arrival_s - self.last_pair_arrival_s
            if 0.0 < interval < 5.0:
                self.intervals.append(interval)
        self.last_pair_arrival_s = arrival_s
        self.last_pair_stamp_ns = stamp_ns
        self.last_stamp_delta_ms = 0.0
        self.observed_pair_count += 1
        if sequence is None:
            self.pair_sequence += 1
        else:
            sequence = int(sequence)
            if (self.last_external_sequence is not None and
                    sequence > self.last_external_sequence + 1):
                self.pair_sequence_gaps += (
                    sequence - self.last_external_sequence - 1)
            self.last_external_sequence = sequence
            self.pair_sequence = sequence
        if source_sequence is not None:
            source_sequence = int(source_sequence)
            if (self.last_source_sequence is not None and
                    source_sequence > self.last_source_sequence):
                delta = source_sequence - self.last_source_sequence
                self.source_deltas.append(delta)
                self.source_sequence_gaps += max(0, delta - 1)
            self.last_source_sequence = source_sequence
        self.paired_stamps.append(stamp_ns)
        self.paired_stamp_set.add(stamp_ns)
        while len(self.paired_stamps) > self.max_pending:
            self.paired_stamp_set.discard(self.paired_stamps.popleft())
        self._trim_pending()
        return True

    @property
    def source_drop_ratio(self):
        if not self.source_deltas:
            return 0.0
        total = sum(self.source_deltas)
        lost = sum(max(0, delta - 1) for delta in self.source_deltas)
        return float(lost / total) if total > 0 else 0.0

    def _trim_pending(self):
        for pending in (self.color_arrivals, self.depth_arrivals):
            if len(pending) > self.max_pending:
                for key in sorted(pending)[:-self.max_pending]:
                    pending.pop(key, None)

