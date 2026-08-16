import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


REPO_ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "record_reliability_timeline",
    REPO_ROOT
    / "src"
    / "ultra_fusion_nav"
    / "scripts"
    / "record_reliability_timeline.py",
)
TIMELINE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TIMELINE)


def backend_events(samples):
    recorder = SimpleNamespace(events=[])
    recorder._relative_event = lambda kind, message: {"kind": kind}
    for sample in samples:
        message = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "unified_backend_fusion"
        status.values = [
            KeyValue(key=name, value=str(value)) for name, value in sample.items()
        ]
        message.status = [status]
        TIMELINE.ReliabilityTimelineRecorder._backend(recorder, message)
    return recorder.events


def visual_frontend_events(samples):
    recorder = SimpleNamespace(events=[])
    recorder._relative_event = lambda kind, message: {"kind": kind}
    recorder._finite_float = TIMELINE.ReliabilityTimelineRecorder._finite_float
    for sample in samples:
        message = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "uf_rgbd_feature_frontend"
        status.values = [
            KeyValue(key=name, value=str(value)) for name, value in sample.items()
        ]
        message.status = [status]
        TIMELINE.ReliabilityTimelineRecorder._visual_frontend(recorder, message)
    return recorder.events


def test_source_age_p95_uses_finite_nonnegative_samples():
    samples = [
        {
            "native_lidar_callback_source_age_s": callback,
            "native_lidar_worker_source_age_s": worker,
            "output_source_age_s": output,
        }
        for callback, worker, output in (
            (0.01, 0.02, 0.03),
            (0.02, 0.04, 0.06),
            (0.03, 0.06, 0.09),
            (0.04, 0.08, 0.12),
            (0.05, 0.10, 0.15),
            (-1.0, -1.0, -1.0),
            ("nan", "inf", "-inf"),
        )
    ]

    summary = TIMELINE.summarize(backend_events(samples))

    assert math.isclose(
        summary["backend_native_lidar_callback_source_age_s_p95"], 0.048
    )
    assert math.isclose(
        summary["backend_native_lidar_worker_source_age_s_p95"], 0.096
    )
    assert math.isclose(summary["backend_output_source_age_s_p95"], 0.144)


def test_source_age_p95_is_none_when_diagnostics_do_not_publish_age():
    summary = TIMELINE.summarize(backend_events([{}]))

    assert summary["backend_native_lidar_callback_source_age_s_p95"] is None
    assert summary["backend_native_lidar_worker_source_age_s_p95"] is None
    assert summary["backend_output_source_age_s_p95"] is None


def test_visual_frontend_summary_preserves_rejection_reason_totals():
    samples = [
        {
            "quality_rejected_candidates": 2,
            "quality_rejected_pnp_invalid": 2,
            "last_geometric_tracks": 0,
            "last_valid_depth_tracks": 0,
        },
        {
            "quality_rejected_candidates": 5,
            "quality_rejected_pnp_invalid": 3,
            "quality_rejected_insufficient_spatial_coverage": 2,
            "last_geometric_tracks": 18,
            "last_valid_depth_tracks": 15,
        },
    ]

    events = visual_frontend_events(samples)
    summary = TIMELINE.summarize(events)

    assert events[-1]["last_geometric_tracks"] == 18
    assert events[-1]["last_valid_depth_tracks"] == 15
    assert summary["visual_quality_rejected_candidates_max"] == 5
    assert summary["visual_quality_rejection_counts"] == {
        "pnp_invalid": 3,
        "insufficient_spatial_coverage": 2,
    }
