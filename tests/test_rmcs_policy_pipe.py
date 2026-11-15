import math

import pytest

from wheeled_algo.rmcs_policy_pipe import MOTOR_NAMES, feedback_to_interfaces, interfaces_to_result


def feedback():
    return dict(positions=[0.] * 6, velocities=[1.] * 6, quaternion_wxyz=[1., 0., 0., 0.],
                gyro_body=[0.] * 3, command=[.1, .2, .3], height=.305, enabled=True,
                reset_count=2, jump_request_count=1, mode=0, ticks=5)


def test_feedback_field_mapping():
    encoded = feedback_to_interfaces(feedback())
    assert encoded["/chassis/jump/request_count"] == 1
    assert encoded["/chassis/reset_count"] == 2
    assert encoded["/wheel_leg/imu/quaternion"] == [1., 0., 0., 0.]
    assert encoded["/chassis/control_velocity"] == [.1, .2, .3]
    for name in MOTOR_NAMES:
        assert encoded[f"/wheel_leg/{name}/angle"] == 0.
        assert encoded[f"/wheel_leg/{name}/velocity"] == 1.


@pytest.mark.parametrize("field,value", [("positions", [0.] * 5), ("gyro_body", [math.nan] * 3),
                                        ("ticks", 0), ("ticks", 1.5), ("enabled", 1),
                                        ("reset_count", -1)])
def test_bad_feedback_is_rejected(field, value):
    value_map = feedback()
    value_map[field] = value
    with pytest.raises(ValueError):
        feedback_to_interfaces(value_map)


def test_result_mapping_and_disarmed_nan():
    values = {f"/wheel_leg/{name}/control_torque": .2 for name in MOTOR_NAMES}
    values.update({"/wheel_leg/rl/state": 3, "/wheel_leg/rl/action_seq": 17,
                   "/wheel_leg/rl/input/enable": True, "/wheel_leg/rl/valid": True,
                   "/wheel_leg/rl/healthy": True})
    assert interfaces_to_result(values)["effort_api"] == [.2] * 6
    values["/wheel_leg/left_hip_joint/control_torque"] = None
    with pytest.raises((TypeError, ValueError)):
        interfaces_to_result(values)
    values["/wheel_leg/rl/state"] = 1
    values["/wheel_leg/rl/input/enable"] = False
    assert interfaces_to_result(values)["effort_api"][0] == 0.


def test_legacy_tick_default():
    values = feedback()
    del values["ticks"]
    assert feedback_to_interfaces(values)["ticks"] == 5
