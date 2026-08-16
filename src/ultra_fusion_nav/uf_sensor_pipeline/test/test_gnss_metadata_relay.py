from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import NavSatFix

from uf_sensor_pipeline.gnss_metadata_relay import LatestGnssPairBuffer


def stamped(message_type, stamp_s):
    message = message_type()
    message.header.stamp.sec = int(stamp_s)
    message.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1.0e9))
    return message


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def test_latest_complete_pair_is_selected_and_source_stamps_are_preserved():
    buffer = LatestGnssPairBuffer(tolerance_s=0.05)
    for stamp_s in (10.0, 10.1, 10.2):
        buffer.add_fix(stamped(NavSatFix, stamp_s))
        buffer.add_raw(stamped(GPSRAW, stamp_s + 0.002))

    fix, raw = buffer.take_latest()

    assert stamp_ns(fix) == 10_200_000_000
    assert stamp_ns(raw) == 10_202_000_000
    assert buffer.take_latest() is None


def test_unmatched_fix_publishes_without_pairing_wrong_metadata():
    buffer = LatestGnssPairBuffer(tolerance_s=0.02)
    buffer.add_fix(stamped(NavSatFix, 20.0))
    buffer.add_raw(stamped(GPSRAW, 20.2))
    fix, raw = buffer.take_latest()
    assert stamp_ns(fix) == 20_000_000_000
    assert raw is None

    buffer.add_fix(stamped(NavSatFix, 20.3))
    buffer.add_raw(stamped(GPSRAW, 20.01))
    next_fix, next_raw = buffer.take_latest()
    assert stamp_ns(next_fix) == 20_300_000_000
    assert next_raw is None


def test_fix_is_not_blocked_when_raw_metadata_is_missing():
    buffer = LatestGnssPairBuffer(tolerance_s=0.05)
    buffer.add_fix(stamped(NavSatFix, 25.0))

    fix, raw = buffer.take_latest()

    assert stamp_ns(fix) == 25_000_000_000
    assert raw is None


def test_new_samples_replace_pending_samples_without_fifo_replay():
    buffer = LatestGnssPairBuffer(tolerance_s=0.01)
    for stamp_s in (30.0, 30.1, 30.2, 30.3):
        buffer.add_fix(stamped(NavSatFix, stamp_s))
        buffer.add_raw(stamped(GPSRAW, stamp_s))

    first_fix, _ = buffer.take_latest()
    assert stamp_ns(first_fix) == 30_300_000_000

    buffer.add_fix(stamped(NavSatFix, 30.4))
    buffer.add_raw(stamped(GPSRAW, 30.4))
    second_fix, _ = buffer.take_latest()
    assert stamp_ns(second_fix) == 30_400_000_000
