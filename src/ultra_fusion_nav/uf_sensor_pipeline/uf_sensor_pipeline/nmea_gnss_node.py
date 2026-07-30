"""ROS 2 adapter for a direct 115200 8N1 NMEA0183 GNSS receiver."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import GPSRAW
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference

from .nmea0183 import (
    GgaObservation,
    RmcObservation,
    circular_time_difference_s,
    parse_sentence,
    utc_datetime,
)


UINT16_MAX = 0xFFFF
UINT32_MAX = 0xFFFFFFFF


def fix_type(rmc, gga):
    if rmc is None or not rmc.valid:
        return GPSRAW.GPS_FIX_TYPE_NO_FIX
    if gga is None:
        return GPSRAW.GPS_FIX_TYPE_2D_FIX
    return {
        0: GPSRAW.GPS_FIX_TYPE_NO_FIX,
        2: GPSRAW.GPS_FIX_TYPE_DGPS,
        4: GPSRAW.GPS_FIX_TYPE_RTK_FIXED,
        5: GPSRAW.GPS_FIX_TYPE_RTK_FLOAT,
    }.get(gga.fix_quality, GPSRAW.GPS_FIX_TYPE_3D_FIX)


def navsat_status(rmc, gga):
    if rmc is None or not rmc.valid or (gga is not None and gga.fix_quality == 0):
        return NavSatStatus.STATUS_NO_FIX
    if gga is not None and gga.fix_quality == 2:
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_FIX


class NmeaGnssNode(Node):
    def __init__(self):
        super().__init__("nmea_gnss")
        defaults = {
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "frame_id": "gnss_antenna_link",
            "fix_topic": "/gnss/direct/fix",
            "raw_topic": "/sensors/gnss/raw",
            "time_reference_topic": "/gnss/direct/time_reference",
            "diagnostics_topic": "/gnss/direct/diagnostics",
            "strict_checksum": True,
            "serial_timeout_s": 0.02,
            "maximum_pair_age_s": 1.5,
            "horizontal_uere_m": 3.0,
            "vertical_uere_m": 5.0,
            "stale_after_s": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required; install the ROS dependency python3-serial"
            ) from exc

        self.strict_checksum = bool(self.get_parameter("strict_checksum").value)
        self.maximum_pair_age_s = float(
            self.get_parameter("maximum_pair_age_s").value
        )
        self.horizontal_uere_m = float(
            self.get_parameter("horizontal_uere_m").value
        )
        self.vertical_uere_m = float(self.get_parameter("vertical_uere_m").value)
        self.stale_after_s = float(self.get_parameter("stale_after_s").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.latest_rmc = None
        self.latest_gga = None
        self.last_valid_arrival = None
        self.counts = {
            "lines": 0,
            "rmc": 0,
            "gga": 0,
            "unsupported": 0,
            "checksum_errors": 0,
            "parse_errors": 0,
            "published": 0,
            "invalid_fix": 0,
        }

        self.fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter("fix_topic").value),
            qos_profile_sensor_data,
        )
        self.raw_pub = self.create_publisher(
            GPSRAW, str(self.get_parameter("raw_topic").value),
            qos_profile_sensor_data,
        )
        self.time_pub = self.create_publisher(
            TimeReference, str(self.get_parameter("time_reference_topic").value),
            qos_profile_sensor_data,
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.serial = serial.Serial(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=float(self.get_parameter("serial_timeout_s").value),
        )
        self.create_timer(0.005, self._poll)
        self.create_timer(1.0, self._diagnostics)
        self.get_logger().info(
            f"Direct GNSS active on {self.serial.port} at {self.serial.baudrate} 8N1; "
            f"strict_checksum={self.strict_checksum}"
        )

    def destroy_node(self):
        if getattr(self, "serial", None) is not None and self.serial.is_open:
            self.serial.close()
        return super().destroy_node()

    def _poll(self):
        for _ in range(64):
            if self.serial.in_waiting <= 0:
                break
            raw = self.serial.readline()
            if not raw:
                break
            self.counts["lines"] += 1
            try:
                sentence = raw.decode("ascii", errors="strict")
                observation, checksum_valid = parse_sentence(
                    sentence, strict_checksum=self.strict_checksum
                )
            except (UnicodeDecodeError, ValueError) as exc:
                if "checksum" in str(exc).lower():
                    self.counts["checksum_errors"] += 1
                else:
                    self.counts["parse_errors"] += 1
                continue
            if not checksum_valid:
                self.counts["checksum_errors"] += 1
            if observation is None:
                self.counts["unsupported"] += 1
            elif isinstance(observation, RmcObservation):
                self.latest_rmc = observation
                self.counts["rmc"] += 1
                self._publish_fix()
            elif isinstance(observation, GgaObservation):
                self.latest_gga = observation
                self.counts["gga"] += 1

    def _paired_gga(self):
        if self.latest_rmc is None or self.latest_gga is None:
            return None
        difference = circular_time_difference_s(
            self.latest_rmc.utc_time, self.latest_gga.utc_time
        )
        return self.latest_gga if difference <= self.maximum_pair_age_s else None

    def _publish_fix(self):
        rmc = self.latest_rmc
        if rmc is None:
            return
        gga = self._paired_gga()
        header_stamp = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = header_stamp
        fix.header.frame_id = self.frame_id
        fix.status.status = navsat_status(rmc, gga)
        fix.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        fix.latitude = float(rmc.latitude_deg)
        fix.longitude = float(rmc.longitude_deg)
        fix.altitude = math.nan if gga is None else float(gga.altitude_ellipsoid_m)
        if gga is not None and math.isfinite(gga.hdop):
            horizontal_sigma = max(0.5, gga.hdop * self.horizontal_uere_m)
            vertical_sigma = max(1.0, gga.hdop * self.vertical_uere_m)
            fix.position_covariance = [
                horizontal_sigma ** 2, 0.0, 0.0,
                0.0, horizontal_sigma ** 2, 0.0,
                0.0, 0.0, vertical_sigma ** 2,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

        raw = GPSRAW()
        raw.header = fix.header
        raw.fix_type = fix_type(rmc, gga)
        raw.lat = int(round(rmc.latitude_deg * 1.0e7))
        raw.lon = int(round(rmc.longitude_deg * 1.0e7))
        raw.alt = 0 if gga is None else int(round(gga.altitude_msl_m * 1000.0))
        raw.eph = (
            UINT16_MAX if gga is None or not math.isfinite(gga.hdop)
            else min(UINT16_MAX - 1, max(0, int(round(gga.hdop * 100.0))))
        )
        raw.epv = UINT16_MAX
        raw.vel = (
            UINT16_MAX if not math.isfinite(rmc.speed_mps)
            else min(UINT16_MAX - 1, max(0, int(round(rmc.speed_mps * 100.0))))
        )
        raw.cog = (
            UINT16_MAX if not math.isfinite(rmc.course_deg)
            else min(35999, max(0, int(round((rmc.course_deg % 360.0) * 100.0))))
        )
        raw.satellites_visible = 255 if gga is None else min(255, gga.satellite_count)
        raw.alt_ellipsoid = (
            0 if gga is None else int(round(gga.altitude_ellipsoid_m * 1000.0))
        )
        raw.h_acc = UINT32_MAX
        raw.v_acc = UINT32_MAX
        raw.vel_acc = UINT32_MAX
        raw.hdg_acc = -1
        self.raw_pub.publish(raw)

        reference = TimeReference()
        reference.header = fix.header
        timestamp = utc_datetime(rmc).timestamp()
        reference.time_ref.sec = int(timestamp)
        reference.time_ref.nanosec = int(round((timestamp - int(timestamp)) * 1.0e9))
        reference.source = "GNSS_NMEA0183_UTC"
        self.time_pub.publish(reference)

        self.counts["published"] += 1
        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            self.counts["invalid_fix"] += 1
        else:
            self.last_valid_arrival = time.monotonic()

    @staticmethod
    def _value(key, value):
        item = KeyValue()
        item.key = str(key)
        item.value = str(value)
        return item

    def _diagnostics(self):
        age_s = (
            math.inf if self.last_valid_arrival is None
            else time.monotonic() - self.last_valid_arrival
        )
        status = DiagnosticStatus()
        status.name = "direct_gnss/nmea0183"
        status.hardware_id = self.serial.port
        status.level = (
            DiagnosticStatus.OK if age_s <= self.stale_after_s
            else DiagnosticStatus.ERROR
        )
        status.message = "ok" if status.level == DiagnosticStatus.OK else "stale_or_no_fix"
        status.values = [
            self._value("transport", f"{self.serial.baudrate}_8N1"),
            self._value("strict_checksum", self.strict_checksum),
            self._value("age_s", f"{age_s:.3f}" if math.isfinite(age_s) else "missing"),
        ]
        status.values.extend(self._value(name, value) for name, value in self.counts.items())
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self.diagnostic_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = NmeaGnssNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
