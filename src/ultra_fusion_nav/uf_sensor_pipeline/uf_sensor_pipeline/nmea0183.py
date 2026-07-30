"""Strict, dependency-free parsing for the GNSS NMEA0183 subset we consume."""

from dataclasses import dataclass
from datetime import date, datetime, time, timezone
import math


KNOT_TO_MPS = 0.5144444444444445


@dataclass(frozen=True)
class RmcObservation:
    utc_time: time
    valid: bool
    latitude_deg: float
    longitude_deg: float
    speed_mps: float
    course_deg: float
    utc_date: date


@dataclass(frozen=True)
class GgaObservation:
    utc_time: time
    latitude_deg: float
    longitude_deg: float
    fix_quality: int
    satellite_count: int
    hdop: float
    altitude_msl_m: float
    geoid_separation_m: float

    @property
    def altitude_ellipsoid_m(self):
        if not math.isfinite(self.altitude_msl_m):
            return math.nan
        if not math.isfinite(self.geoid_separation_m):
            return self.altitude_msl_m
        return self.altitude_msl_m + self.geoid_separation_m


def nmea_checksum(payload):
    checksum = 0
    for byte in payload.encode("ascii"):
        checksum ^= byte
    return checksum


def _split_sentence(sentence, strict_checksum=True):
    text = str(sentence).strip()
    if not text.startswith("$") or "*" not in text:
        raise ValueError("NMEA sentence must start with '$' and contain '*hh'")
    payload, checksum_text = text[1:].rsplit("*", 1)
    if len(checksum_text) < 2:
        raise ValueError("NMEA checksum is incomplete")
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError as exc:
        raise ValueError("NMEA checksum is not hexadecimal") from exc
    actual = nmea_checksum(payload)
    if strict_checksum and actual != expected:
        raise ValueError(
            f"NMEA checksum mismatch: received {expected:02X}, calculated {actual:02X}"
        )
    return payload.split(","), actual == expected


def _parse_utc_time(value):
    if len(value) < 6:
        raise ValueError("NMEA UTC time is incomplete")
    hour = int(value[0:2])
    minute = int(value[2:4])
    second_value = float(value[4:])
    second = int(second_value)
    microsecond = int(round((second_value - second) * 1_000_000.0))
    if microsecond == 1_000_000:
        second += 1
        microsecond = 0
    return time(hour, minute, second, microsecond, tzinfo=timezone.utc)


def _parse_date(value):
    if len(value) != 6:
        raise ValueError("NMEA UTC date must be ddmmyy")
    day = int(value[0:2])
    month = int(value[2:4])
    year_two_digits = int(value[4:6])
    year = 2000 + year_two_digits if year_two_digits < 80 else 1900 + year_two_digits
    return date(year, month, day)


def _coordinate(value, hemisphere, degree_digits):
    if not value:
        return math.nan
    degrees = int(value[:degree_digits])
    minutes = float(value[degree_digits:])
    coordinate = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        coordinate = -coordinate
    elif hemisphere not in ("N", "E"):
        raise ValueError(f"unsupported NMEA hemisphere: {hemisphere}")
    return coordinate


def _float_or_nan(value):
    return float(value) if value else math.nan


def parse_sentence(sentence, strict_checksum=True):
    fields, checksum_valid = _split_sentence(sentence, strict_checksum)
    if not fields or len(fields[0]) != 5:
        raise ValueError("unsupported NMEA talker/sentence identifier")
    sentence_type = fields[0][2:]
    if sentence_type == "RMC":
        if len(fields) < 10:
            raise ValueError("RMC sentence has too few fields")
        result = RmcObservation(
            utc_time=_parse_utc_time(fields[1]),
            valid=fields[2] == "A",
            latitude_deg=_coordinate(fields[3], fields[4], 2),
            longitude_deg=_coordinate(fields[5], fields[6], 3),
            speed_mps=_float_or_nan(fields[7]) * KNOT_TO_MPS,
            course_deg=_float_or_nan(fields[8]),
            utc_date=_parse_date(fields[9]),
        )
    elif sentence_type == "GGA":
        if len(fields) < 12:
            raise ValueError("GGA sentence has too few fields")
        result = GgaObservation(
            utc_time=_parse_utc_time(fields[1]),
            latitude_deg=_coordinate(fields[2], fields[3], 2),
            longitude_deg=_coordinate(fields[4], fields[5], 3),
            fix_quality=int(fields[6] or 0),
            satellite_count=int(fields[7] or 0),
            hdop=_float_or_nan(fields[8]),
            altitude_msl_m=_float_or_nan(fields[9]),
            geoid_separation_m=_float_or_nan(fields[11]),
        )
    else:
        return None, checksum_valid
    return result, checksum_valid


def utc_datetime(rmc):
    return datetime.combine(rmc.utc_date, rmc.utc_time).astimezone(timezone.utc)


def seconds_of_day(value):
    return (
        value.hour * 3600.0
        + value.minute * 60.0
        + value.second
        + value.microsecond * 1.0e-6
    )


def circular_time_difference_s(first, second):
    difference = abs(seconds_of_day(first) - seconds_of_day(second))
    return min(difference, 86400.0 - difference)
