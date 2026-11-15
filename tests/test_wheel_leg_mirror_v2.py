"""Vehicle adapter checks for the generic StateMirror v2 transport."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

import pytest

from wheeled_algo.wheel_leg_mirror import MirrorModel, ROOT, load_telemetry_reader


@pytest.fixture
def protocol():
    return load_telemetry_reader(Path(os.environ.get('RMCS_ROOT', ROOT.parent / 'RMCS')), v2=True)


@pytest.fixture
def model():
    return MirrorModel()


def frame(protocol, model, *, estimator=True):
    names = protocol.MOTOR_NAMES
    by_source = {e['source']: e for e in model.entries}
    q = model.spec['nominal_joint_pos']
    joints = {name: ((q[by_source[name]['target']] - by_source[name]['offset_rad']) / by_source[name]['sign'],
                     .01, .002) for name in names}
    values = {'coordinate': 'rmcs_hardware_output_shaft_v1', 'schema': 'rmcs_state_mirror_v2',
              'control/enabled': 'false', 'control/ready': 'true', 'control/reset_count': '2'}
    values.update({'model_offset/' + n: str(by_source[n]['offset_rad']) for n in names[:4]})
    if estimator:
        values.update({'state/position/' + str(i): str(q[by_source[n]['target']]) for i, n in enumerate(names)})
    vectors = {'rl_projected_gravity': (0., 0., -1.)} if estimator else {}
    return protocol.Frame(1_000_000_001, joints,
        dict(quaternion_wxyz=(1., 0., 0., 0.), gyro=(.1, .2, .3), age_s=.002), vectors, values, 'chassis_body')


def test_v2_missing_passive_references_remain_displayable(protocol, model):
    snapshot = protocol.from_frame(frame(protocol, model))
    result = model.reconstruct(snapshot)
    assert result['reference_valid'] is False
    assert result['knee_error_rad'] is None and result['leg_length_error_m'] is None
    assert result['closure_error_m'] < 2e-6
    assert max(abs(v) for v in result['coordinate_error_rad']) < 1e-12
    assert snapshot.control_enabled is False and snapshot.dm_control_ready is True
    assert snapshot.reset_count == 2


def test_v2_gimbal_only_omits_estimator_comparisons(protocol, model):
    snapshot = protocol.from_frame(frame(protocol, model, estimator=False))
    result = model.reconstruct(snapshot)
    assert result['reference_valid'] is False
    assert result['coordinate_error_rad'] is None and result['gravity_error'] is None
    assert result['closure_error_m'] < 2e-6


def test_vehicle_adapter_requires_measured_joints_and_coordinate(protocol, model):
    sample = frame(protocol, model)
    del sample.joints['left_knee_joint']
    with pytest.raises(ValueError, match='requires six'):
        protocol.from_frame(sample)
    sample = frame(protocol, model)
    sample.values['coordinate'] = 'other'
    with pytest.raises(ValueError, match='coordinate'):
        protocol.from_frame(sample)


def test_stale_motor_and_imu_age_is_a_vehicle_check(protocol, model):
    sample = frame(protocol, model)
    sample.joints['left_knee_joint'] = (*sample.joints['left_knee_joint'][:2], .1)
    with pytest.raises(ValueError, match='older than 40 ms'):
        model.reconstruct(protocol.from_frame(sample))
    sample = frame(protocol, model)
    sample.imu['age_s'] = -1.
    with pytest.raises(ValueError, match='older than 40 ms'):
        model.reconstruct(protocol.from_frame(sample))


def test_v2_record_replay_uses_existing_mapping_identity(protocol, model, tmp_path):
    path = tmp_path / 'v2.jsonl'
    latest = protocol.LatestSnapshot(path, metadata=model.metadata)
    sample = frame(protocol, model)
    latest.put(sample)
    assert latest.get()[0].positions == protocol.from_frame(sample).positions
    latest.close()
    snapshot = next(protocol.read_record(path, model.metadata))[1]
    assert snapshot.positions == protocol.from_frame(sample).positions
    assert model.reconstruct(snapshot)['reference_valid'] is False
    with pytest.raises(ValueError, match='mapping_sha256'):
        list(protocol.read_record(path, {'mapping_sha256': 'wrong'}))


def test_v1_record_retains_joint_imu_values_and_old_mapping(protocol, model, tmp_path):
    sample = protocol.from_frame(frame(protocol, model))
    path = tmp_path / 'v1.jsonl'
    path.write_text(json.dumps(dict(kind='metadata', schema='wheel_leg_mirror_record_v1', metadata=model.metadata))
                    + '\n' + json.dumps(asdict(sample) | {'record_time_s': 0.}) + '\n')
    replay = next(protocol.read_record(path, model.metadata))[1]
    assert replay.positions == sample.positions and replay.velocities == sample.velocities
    assert replay.quaternion_wxyz == sample.quaternion_wxyz and replay.gyro_body == sample.gyro_body


def test_v1_conversion_has_only_canonical_model_keys(protocol, model):
    generic = sys.modules[protocol.Frame.__module__]
    snapshot = protocol.from_frame(frame(protocol, model))
    converted = generic.convert_v1(asdict(snapshot))
    assert 'state/position/0' in converted.values
    assert 'model_offset/left_hip_joint' in converted.values
    assert not any(k.startswith(('rmcs_model_positions/', 'model_offsets/')) for k in converted.values)


def test_public_replay_event_keeps_source_data(protocol):
    generic = sys.modules[protocol.Frame.__module__]
    latest = generic.LatestFrame()
    events = []
    latest.listeners.append(lambda kind, row: events.append((kind, row)))
    latest.replay_event(dict(kind='event', record_time_s=1.25, name='source_epoch', epoch=2))
    assert events == [('event', dict(name='source_epoch', epoch=2))]
