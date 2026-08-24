import math

from uf_reliability.relocalization_request_arbiter import (
    RelocalizationRequestArbiterCore,
)


def update(core, source, sequence, active, now, **kwargs):
    return core.update(
        source_id=source,
        instance_id=kwargs.get("instance_id", source + "-instance"),
        sequence=sequence,
        episode_id=kwargs.get("episode_id", 1),
        active=active,
        lease_duration_s=kwargs.get("lease_duration_s", 1.0),
        source_stamp_s=kwargs.get("source_stamp_s", now),
        steady_now_s=now,
        ros_now_s=kwargs.get("ros_now_s", now),
        reason=kwargs.get("reason", "test"),
    )


def test_legacy_last_writer_wins_reproduces_lost_request():
    production_request = False
    production_request = True  # reliability owns a still-valid request
    production_request = False  # unrelated safety source releases
    assert production_request is False


def test_one_source_release_cannot_clear_other_source():
    core = RelocalizationRequestArbiterCore()
    assert update(core, "reliability_scheduler", 1, True, 1.0).output_active
    decision = update(core, "localization_safety", 1, False, 1.1)
    assert decision.output_active
    assert decision.active_sources == ("reliability_scheduler",)


def test_both_sources_and_interleaved_release_are_or_owned():
    core = RelocalizationRequestArbiterCore()
    transitions = []
    for source, sequence, active, now in (
        ("reliability_scheduler", 1, True, 1.0),
        ("localization_safety", 1, True, 1.1),
        ("reliability_scheduler", 2, False, 1.2),
        ("reliability_scheduler", 3, True, 1.3),
        ("localization_safety", 2, False, 1.4),
        ("reliability_scheduler", 4, False, 1.5),
    ):
        decision = update(core, source, sequence, active, now)
        if decision.output_changed:
            transitions.append(decision.output_active)
    assert transitions == [True, False]
    assert core.output_transitions == 2


def test_source_crash_expires_without_permanent_request():
    core = RelocalizationRequestArbiterCore()
    update(core, "reliability_scheduler", 1, True, 1.0, lease_duration_s=0.5)
    assert core.tick(1.49).output_active
    decision = core.tick(1.50)
    assert not decision.output_active
    assert decision.expired_sources == ("reliability_scheduler",)
    assert core.expired_leases == 1


def test_other_active_source_survives_peer_expiry():
    core = RelocalizationRequestArbiterCore()
    update(core, "reliability_scheduler", 1, True, 1.0, lease_duration_s=0.5)
    update(core, "localization_safety", 1, True, 1.1, lease_duration_s=1.0)
    decision = core.tick(1.51)
    assert decision.output_active
    assert decision.active_sources == ("localization_safety",)


def test_heartbeats_do_not_create_duplicate_output_edges():
    core = RelocalizationRequestArbiterCore()
    assert update(core, "reliability_scheduler", 1, True, 1.0).output_changed
    for sequence, now in enumerate((1.2, 1.4, 1.6), start=2):
        decision = update(core, "reliability_scheduler", sequence, True, now)
        assert decision.output_active
        assert not decision.output_changed
    assert core.output_transitions == 1


def test_duplicate_and_reordered_packets_do_not_extend_lease():
    core = RelocalizationRequestArbiterCore()
    update(core, "reliability_scheduler", 2, True, 1.0, lease_duration_s=0.5)
    decision = update(
        core,
        "reliability_scheduler",
        2,
        True,
        1.4,
        lease_duration_s=0.5,
        source_stamp_s=1.0,
    )
    assert not decision.accepted
    assert decision.reason == "duplicate_or_reordered"
    assert not core.tick(1.51).output_active


def test_stale_future_nonfinite_and_timestamp_regression_are_rejected():
    core = RelocalizationRequestArbiterCore()
    assert not update(
        core, "reliability_scheduler", 1, True, 5.0, source_stamp_s=2.0
    ).accepted
    assert not update(
        core, "reliability_scheduler", 1, True, 5.0, source_stamp_s=6.0
    ).accepted
    assert not update(
        core,
        "reliability_scheduler",
        1,
        True,
        5.0,
        lease_duration_s=math.nan,
    ).accepted
    assert update(core, "reliability_scheduler", 1, True, 5.0).accepted
    decision = update(
        core, "reliability_scheduler", 2, True, 5.1, source_stamp_s=4.9
    )
    assert not decision.accepted
    assert decision.reason == "timestamp_regression"


def test_restart_replaces_stale_instance_and_rejects_late_old_instance():
    core = RelocalizationRequestArbiterCore()
    update(
        core,
        "reliability_scheduler",
        8,
        True,
        1.0,
        instance_id="old",
    )
    decision = update(
        core,
        "reliability_scheduler",
        1,
        False,
        1.1,
        instance_id="new",
    )
    assert decision.accepted
    assert not decision.output_active
    late = update(
        core,
        "reliability_scheduler",
        9,
        True,
        1.2,
        instance_id="old",
    )
    assert not late.accepted
    assert late.reason == "retired_instance"


def test_recovery_then_new_episode_produces_exactly_one_new_edge():
    core = RelocalizationRequestArbiterCore()
    first = update(core, "localization_safety", 1, True, 1.0, episode_id=1)
    release = update(core, "localization_safety", 2, False, 1.1, episode_id=1)
    second = update(core, "localization_safety", 3, True, 6.2, episode_id=2)
    repeat = update(core, "localization_safety", 4, True, 6.3, episode_id=2)
    assert first.output_changed and first.output_active
    assert release.output_changed and not release.output_active
    assert second.output_changed and second.output_active
    assert not repeat.output_changed
    assert core.output_transitions == 3


def test_epoch_or_failure_release_only_releases_own_source():
    core = RelocalizationRequestArbiterCore()
    update(core, "reliability_scheduler", 1, True, 1.0)
    update(core, "localization_safety", 1, True, 1.1)
    decision = update(
        core,
        "reliability_scheduler",
        2,
        False,
        1.2,
        reason="epoch_committed",
    )
    assert decision.output_active
    decision = update(
        core,
        "localization_safety",
        2,
        False,
        1.3,
        reason="relocalization_failed",
    )
    assert not decision.output_active


def test_unknown_source_and_invalid_lease_fail_closed():
    core = RelocalizationRequestArbiterCore()
    assert not update(core, "unknown", 1, True, 1.0).accepted
    assert not update(
        core,
        "reliability_scheduler",
        1,
        True,
        1.0,
        lease_duration_s=9.0,
    ).accepted
    assert not core.output_active


def test_clock_reset_clears_all_leases():
    core = RelocalizationRequestArbiterCore()
    update(core, "reliability_scheduler", 1, True, 1.0)
    decision = core.reset()
    assert decision.output_changed
    assert not decision.output_active
    assert not core.leases
