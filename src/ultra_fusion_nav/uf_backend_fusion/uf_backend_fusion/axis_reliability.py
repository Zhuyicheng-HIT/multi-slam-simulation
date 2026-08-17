"""Per-axis reliability composition for heterogeneous navigation sources."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class AxisReliabilityProfile:
    """Separate source health, factor consistency, and axis observability."""

    source: str
    health: float
    consistency_xyz: object
    observability_xyz: object
    global_reference_xyz: object

    def __post_init__(self):
        health = float(self.health)
        consistency = np.asarray(self.consistency_xyz, dtype=float)
        observability = np.asarray(self.observability_xyz, dtype=float)
        global_reference = np.asarray(self.global_reference_xyz, dtype=bool)
        if not str(self.source):
            raise ValueError("axis reliability source must be named")
        if not math.isfinite(health) or not 0.0 <= health <= 1.0:
            raise ValueError("axis reliability health must be in [0, 1]")
        if consistency.shape != (3,) or observability.shape != (3,):
            raise ValueError("axis reliability evidence must be 3-vectors")
        if global_reference.shape != (3,):
            raise ValueError("axis global-reference mask must be a 3-vector")
        if (
            np.any(~np.isfinite(consistency))
            or np.any(~np.isfinite(observability))
            or np.any(consistency < 0.0)
            or np.any(consistency > 1.0)
            or np.any(observability < 0.0)
            or np.any(observability > 1.0)
        ):
            raise ValueError("axis reliability evidence must be in [0, 1]")
        object.__setattr__(self, "health", health)
        object.__setattr__(self, "consistency_xyz", consistency.copy())
        object.__setattr__(self, "observability_xyz", observability.copy())
        object.__setattr__(self, "global_reference_xyz", global_reference.copy())

    @property
    def reliability_xyz(self):
        return (
            self.health
            * self.consistency_xyz
            * self.observability_xyz
        )

    @property
    def degradation_xyz(self):
        return 1.0 - self.reliability_xyz


@dataclass(frozen=True)
class AxisReliabilitySummary:
    reliability_xyz: object
    degradation_xyz: object
    global_reliability_xyz: object
    supporting_sources_xyz: object


def barometer_activation_required(
    *,
    lidar_z_weak,
    alternative_z_information,
    stamp_s,
    gnss_prefit_stamp_s,
    gnss_max_age_s,
    gnss_z_admitted,
    gnss_z_nis,
    gnss_z_nis_gate,
    enabled=True,
):
    """Decide whether a local pressure segment should protect the Z axis.

    A healthy fresh GNSS Z or RGB-D Z source suppresses the local segment. A
    fresh GNSS Z innovation that is rejected is different: it is evidence that
    the current Z estimate is inconsistent, so pressure may protect subsequent
    local height changes even when the LiDAR Hessian still looks strong.
    """
    if not bool(enabled):
        return False
    alternative = float(alternative_z_information)
    if not math.isfinite(alternative) or alternative < 0.0:
        alternative = 0.0
    stamp_s = float(stamp_s)
    gnss_stamp_s = float(gnss_prefit_stamp_s)
    gnss_age_s = stamp_s - gnss_stamp_s
    gnss_fresh = (
        math.isfinite(stamp_s)
        and math.isfinite(gnss_stamp_s)
        and math.isfinite(float(gnss_max_age_s))
        and gnss_stamp_s > 0.0
        and 0.0 <= gnss_age_s <= float(gnss_max_age_s)
    )
    gnss_conflict = gnss_fresh and (
        not bool(gnss_z_admitted)
        or (
            math.isfinite(float(gnss_z_nis))
            and float(gnss_z_nis) >= float(gnss_z_nis_gate)
        )
    )
    return bool(alternative <= 0.0 and (bool(lidar_z_weak) or gnss_conflict))


def combine_axis_reliability(profiles, minimum_source_reliability=0.05):
    """Combine sources with OR semantics independently on each axis.

    Independent reliabilities combine as ``1 - product(1-r)``. One healthy
    source therefore keeps its axis usable even when every other modality is
    degraded. The global mask is tracked separately so local sources such as
    RGB-D, LiDAR, optical flow, and a local barometer segment cannot be mistaken
    for an absolute global datum.
    """
    minimum = float(minimum_source_reliability)
    if not math.isfinite(minimum) or not 0.0 <= minimum <= 1.0:
        raise ValueError("minimum source reliability must be in [0, 1]")
    profiles = tuple(profiles)
    total_complement = np.ones(3, dtype=float)
    global_complement = np.ones(3, dtype=float)
    sources = [[], [], []]
    for profile in profiles:
        if not isinstance(profile, AxisReliabilityProfile):
            raise TypeError("profiles must contain AxisReliabilityProfile")
        reliability = profile.reliability_xyz
        total_complement *= 1.0 - reliability
        global_reliability = np.where(
            profile.global_reference_xyz, reliability, 0.0
        )
        global_complement *= 1.0 - global_reliability
        for axis in range(3):
            if reliability[axis] >= minimum:
                sources[axis].append(profile.source)
    reliability = 1.0 - total_complement
    return AxisReliabilitySummary(
        reliability_xyz=reliability,
        degradation_xyz=1.0 - reliability,
        global_reliability_xyz=1.0 - global_complement,
        supporting_sources_xyz=tuple(tuple(values) for values in sources),
    )
