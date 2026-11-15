"""Wheel-leg field adapter for the generic RMCS PipeFeedback plugin."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import selectors
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MOTOR_NAMES = ("left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint", "left_wheel", "right_wheel")
PROFILE = "rmcs_ws/src/rmcs_rl/config/simulation/wheel-leg-pipe.yaml"
COMPONENTS = {
    "rmcs_rl::PipeFeedback", "rmcs_rl::RlBridge",
    "rmcs_core::controller::chassis::WheelLegStateEstimator",
    "rmcs_core::controller::chassis::WheelLegObservation",
    "rmcs_core::controller::chassis::WheelLegController",
}


def _vector(value, size, name):
    if len(value) != size:
        raise ValueError(f"{name} must contain {size} values")
    values = [float(v) for v in value]
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} must be finite")
    return values


def feedback_to_interfaces(feedback):
    """Translate the historical PhysX feedback fields into interface path keys."""
    result = {}
    for field, suffix in (("positions", "angle"), ("velocities", "velocity")):
        values = _vector(feedback[field], 6, field)
        result.update({f"/wheel_leg/{name}/{suffix}": value for name, value in zip(MOTOR_NAMES, values)})
    result["/wheel_leg/imu/quaternion"] = _vector(feedback["quaternion_wxyz"], 4, "quaternion_wxyz")
    result["/wheel_leg/imu/angular_velocity"] = _vector(feedback["gyro_body"], 3, "gyro_body")
    result["/chassis/control_velocity"] = _vector(feedback["command"], 3, "command")
    result["/chassis/control_height"] = _vector([feedback["height"]], 1, "height")[0]
    if type(feedback["enabled"]) is not bool:
        raise ValueError("enabled must be a bool")
    result["/chassis/control_enable"] = feedback["enabled"]
    for field, path in (("reset_count", "/chassis/reset_count"),
                        ("jump_request_count", "/chassis/jump/request_count"),
                        ("mode", "/chassis/control_mode")):
        value = feedback.get(field, 0)
        if type(value) is not int or value < 0:
            raise ValueError(f"{field} must be a nonnegative integer")
        result[path] = value
    ticks = feedback.get("ticks", 5)
    if type(ticks) is not int or ticks <= 0:
        raise ValueError("ticks must be a positive integer")
    result["ticks"] = ticks
    return result


def interfaces_to_result(values):
    """Keep the PhysX driver's result shape while the transport stays generic."""
    state = values["/wheel_leg/rl/state"]
    enabled = values["/wheel_leg/rl/input/enable"]
    effort = [values[f"/wheel_leg/{name}/control_torque"] for name in MOTOR_NAMES]
    # JSON encodes the controller's disarmed NaN torque as null. An owned simulated
    # actuator receives zero in that state; non-finite running efforts are errors.
    if state == 1 and not enabled:
        effort = [0. if v is None else v for v in effort]
    return dict(effort_api=_vector(effort, 6, "effort_api"), state=state,
                action_seq=values["/wheel_leg/rl/action_seq"], enable=enabled,
                policy_valid=values["/wheel_leg/rl/valid"], healthy=values["/wheel_leg/rl/healthy"])


def profile_identity(rmcs_root, config=None):
    import yaml
    rmcs_root = Path(rmcs_root).resolve()
    path = Path(config).resolve() if config is not None else rmcs_root / PROFILE
    relative = path.relative_to(rmcs_root)
    document = yaml.safe_load(path.read_text())
    executor = document["rmcs_executor"]["ros__parameters"]
    classes = [item.split(" -> ")[0] for item in executor["components"]]
    if executor.get("clock") != "stepped" or len(classes) != 5 or set(classes) != COMPONENTS:
        raise ValueError("Pipe simulation requires the five configured simulation/control plugins and stepped clock")
    params = document["rl_bridge"]["ros__parameters"]
    policy = Path(params["rl_model_path"])
    if not policy.is_absolute():
        policy = rmcs_root / "rmcs_ws/src/rmcs_rl" / policy
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    return dict(profile="RMCS:" + relative.as_posix(), profile_sha256=digest(path),
                policy_sha256=digest(policy), policy_hz=params["policy_rate"],
                physics_hz=executor["update_rate"])


class PolicyHost:
    """One owned executor subprocess stepped by pipe feedback."""
    def __init__(self, rmcs_root, *, config=None):
        rmcs_root = Path(rmcs_root).resolve()
        self.identity = profile_identity(rmcs_root, config)
        command = [str(ROOT / "scripts/wheel_leg_policy_pipe.sh"), "--rmcs-root", str(rmcs_root)]
        if config is not None:
            command.extend(["--config", str(Path(config).resolve())])
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.poller = selectors.DefaultSelector()
        self.poller.register(self.process.stdout, selectors.EVENT_READ)
        try:
            if self.receive(20.) != {"ready": True}:
                raise ValueError("PipeFeedback did not confirm readiness")
        except BaseException:
            self.close()
            raise

    def receive(self, timeout=3.):
        if not self.poller.select(timeout):
            raise TimeoutError("local simulation executor did not respond")
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError(f"local simulation executor exited ({self.process.poll()}); see stderr")
        return json.loads(line)

    def step(self, feedback):
        self.process.stdin.write(json.dumps(feedback_to_interfaces(feedback), allow_nan=False) + "\n")
        self.process.stdin.flush()
        return interfaces_to_result(self.receive())

    def close(self):
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=5.)
        self.process.stdout.close()
        self.poller.close()
