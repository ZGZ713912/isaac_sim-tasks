"""Independent native PhysX vehicle driven by the normal RMCS pipe executor.

PhysicsVehicle is retained from repository commit 498bb2c1; transport uses PipeFeedback.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
from wheeled_algo.rmcs_policy_pipe import MOTOR_NAMES, PROFILE, PolicyHost, profile_identity


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def display_alignment(real_pose, simulated_pose):
    """Rigid yaw/translation display transform; preserves roll/pitch differences."""
    yaw = lambda p: np.arctan2(p[1, 0], p[0, 0])
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("z", yaw(real_pose) - yaw(simulated_pose)).as_matrix()
    transform[:3, 3] = real_pose[:3, 3] - transform[:3, :3] @ simulated_pose[:3, 3]
    return transform


def torch_value(value):
    return value.torch if hasattr(value, "torch") else value


class PhysicsVehicle:
    def __init__(self, model, rmcs_root, *, device="cpu", position=(0., 0., .65)):
        # Imports must follow SimulationApp/AppLauncher initialization.
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation, ArticulationCfg
        from isaaclab.actuators import IdealPDActuatorCfg
        from pxr import UsdGeom
        from wheeled_tasks.chassis.gas_spring_compensation import GasSpringCompensation
        from wheeled_tasks.chassis.leg_motor_friction import LegMotorFriction, declaration
        from wheeled_tasks.chassis.identified_wheel_response import IdentifiedWheelResponse
        self.torch, self.device, self.model = torch, device, model
        self.dt = .001
        self.steps = self.reset_count = 0
        self.remote_reset = None
        self.gimbal_target = np.zeros(2, dtype=float)
        self.result = dict(state=1, phase=0, fault=0, action_seq=0, enable=False)
        self.host = None
        self.version3 = hasattr(Articulation, "write_root_link_pose_to_sim_index")
        if self.version3:
            from isaaclab_physx.physics import PhysxCfg
            sim_cfg = sim_utils.SimulationCfg(device=device, dt=self.dt, render_interval=1,
                physics=PhysxCfg(solver_type=1, enable_external_forces_every_iteration=True))
        else:
            sim_cfg = sim_utils.SimulationCfg(device=device, dt=self.dt, render_interval=1,
                use_fabric=False, physx=sim_utils.PhysxCfg(solver_type=1,
                    enable_external_forces_every_iteration=True))
        self.sim = sim_utils.SimulationContext(sim_cfg)
        self.stage = self.sim.stage
        floor = sim_utils.CuboidCfg(size=(200., 200., .1),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=.002, rest_offset=0.),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=.7, dynamic_friction=.7, restitution=0.))
        floor.func("/World/Ground", floor, translation=(0., 0., -.05))
        light = sim_utils.DomeLightCfg(intensity=2500.)
        light.func("/World/Light", light)
        self.active = list(model.manifest["control_joint_names"])
        self.legs = ["L_joint1", "LL_joint1", "R_joint1", "RR_joint1"]
        self.springs = model.manifest["spring_joint_names"]
        gimbals = ["M_joint1", "M_joint2"]
        passive = [n for n in model.manifest["tree_joint_names"] if n not in self.active + self.springs + gimbals]
        self.friction = LegMotorFriction(1, device, declaration(ROOT), root=ROOT)
        cfg = json.loads((model.bundle.parent / "isaac/runtime.json").read_text())
        gas_cfg = cfg["v5_control"]["gas_spring_compensation"] | dict(
            model_file=str(model.bundle / "gas_spring_model.json"), model_sha256=sha(model.bundle / "gas_spring_model.json"))
        self.gas = GasSpringCompensation(gas_cfg, root=ROOT, device=device)
        self.wheels = IdentifiedWheelResponse(1, device, self.dt, dict(enabled=True,
            model_path=str(model.bundle / "wheel_model.json"), model_sha256=sha(model.bundle / "wheel_model.json"),
            apply_aggregate_delay=True), root=ROOT)
        native = self.wheels.native_parameters()
        def actuator(names, cap, damping=0., **extra):
            return IdealPDActuatorCfg(joint_names_expr=names, stiffness=0., damping=damping,
                effort_limit=cap, effort_limit_sim=cap, velocity_limit_sim=1.e9, **extra)
        self.robot = Articulation(ArticulationCfg(prim_path="/World/PhysicsCar",
            spawn=sim_utils.UsdFileCfg(usd_path=str(model.bundle / model.manifest["usd"]),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False,
                    solver_position_iteration_count=64, solver_velocity_iteration_count=32)),
            init_state=ArticulationCfg.InitialStateCfg(pos=tuple(position),
                joint_pos=model.manifest["nominal_joint_pos"], joint_vel={".*": 0.}),
            actuators=dict(legs=actuator(self.legs, self.friction.native_effort_limit,
                    friction=0., dynamic_friction=0., viscous_friction=0.),
                wheels=actuator(["L_joint3", "R_joint3"], self.wheels.native_effort_limit,
                    armature=native["armature"], friction=native["frictionloss"],
                    dynamic_friction=native["dynamic_friction"], viscous_friction=native["damping"]),
                yaw=actuator([gimbals[0]], 2.), pitch=actuator([gimbals[1]], 10.),
                passive=actuator(passive, 100., .002), gas=actuator(self.springs, 1000., .002)),
            soft_joint_pos_limit_factor=1.))
        # Rendering uses a second visual-only copy, so display alignment never
        # changes the dynamics root or its collision geometry.
        UsdGeom.Imageable(self.stage.GetPrimAtPath("/World/PhysicsCar")).MakeInvisible()
    def initialize(self, rmcs_root):
        self.sim.reset()
        self.robot.update(self.dt)
        self.ids = {n: self.robot.joint_names.index(n) for n in self.robot.joint_names}
        self.active_ids = [self.ids[n] for n in self.active]
        self.leg_ids = [self.ids[n] for n in self.legs]
        self.gas_ids = [self.ids[n] for n in self.springs]
        self.host = PolicyHost(rmcs_root)

    def pose(self):
        if self.version3:
            p = torch_value(self.robot.data.root_link_pose_w)[0].cpu().numpy()
            quat = p[3:7]
        else:
            p = self.robot.data.root_link_state_w[0, :7].cpu().numpy()
            quat = p[[4, 5, 6, 3]]
        result = np.eye(4)
        result[:3, 3] = p[:3]
        result[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return result

    def joints(self):
        return dict(zip(self.robot.joint_names, torch_value(self.robot.data.joint_pos)[0].cpu().tolist()))

    def reset_pose(self, pose):
        torch = self.torch
        q = torch_value(self.robot.data.joint_pos).clone()
        rotation = Rotation.from_matrix(pose[:3, :3]).as_quat()
        if not self.version3:
            rotation = rotation[[3, 0, 1, 2]]
        root = torch.tensor([[*pose[:3, 3], *rotation]], dtype=torch.float32, device=self.device)
        zero = torch.zeros_like(q)
        if self.version3:
            self.robot.write_root_link_pose_to_sim_index(root_pose=root)
            self.robot.write_root_com_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=self.device))
            self.robot.write_joint_position_to_sim_index(position=q)
            self.robot.write_joint_velocity_to_sim_index(velocity=zero)
        else:
            self.robot.write_root_pose_to_sim(root)
            self.robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=self.device))
            self.robot.write_joint_state_to_sim(q, zero)
        self.robot.reset()
        self.wheels.reset()
        self.reset_count += 1
        self.robot.update(self.dt)

    def step(self, snapshot, transport_fresh):
        torch = self.torch
        q, dq = (torch_value(v) for v in (self.robot.data.joint_pos, self.robot.data.joint_vel))
        pose = self.pose()
        rotation = Rotation.from_matrix(pose[:3, :3])
        if self.version3:
            world_gyro = torch_value(self.robot.data.root_com_vel_w)[0, 3:].cpu().numpy()
            gyro = rotation.inv().apply(world_gyro)
        else:
            gyro = self.robot.data.root_ang_vel_b[0].cpu().numpy()
        quaternion = rotation.as_quat()[[3, 0, 1, 2]].tolist()
        op = snapshot.operator if snapshot else None
        if snapshot is not None and snapshot.remote:
            remote = snapshot.remote
            left_switch = int(round(remote.get("switch_left", 0.)))
            right_switch = int(round(remote.get("switch_right", 0.)))
            if left_switch in (0, 2) or right_switch in (0, 2):
                self.gimbal_target[:] = 0.
            else:
                # Same SimpleGimbalController joystick sensitivity and signs;
                # the physical gimbal then follows this reference through its
                # own inertia/effort, never by copying measured q.
                self.gimbal_target[0] += .006 * remote.get("left_y", 0.)
                self.gimbal_target[1] -= .006 * remote.get("left_x", 0.)
                self.gimbal_target[0] = float(np.clip(self.gimbal_target[0], -np.pi, np.pi))
                self.gimbal_target[1] = float(np.clip(self.gimbal_target[1], -.6109, .5236))
        if snapshot is not None and snapshot.reset_count != self.remote_reset:
            self.remote_reset = snapshot.reset_count
            self.reset_count += 1
        enabled = bool(op and op["enabled"] and op["fresh"] and transport_fresh)
        # Freshness gates only the owned simulated actuator. The real measured
        # mirror never depends on remote/disarm state.
        by_source = {e["source"]: e for e in self.model.entries}
        entries = [by_source[n] for n in MOTOR_NAMES]
        feedback = dict(
            positions=[(float(q[0, self.ids[e["target"]]]) - e["offset_rad"]) / e["sign"] for e in entries],
            velocities=[float(dq[0, self.ids[e["target"]]]) / e["sign"] for e in entries],
            quaternion_wxyz=quaternion, gyro_body=gyro.tolist(), enabled=enabled,
            reset_count=self.reset_count, mode=op["mode"] if op else 0,
            command=op["command"] if op else [0., 0., 0.], height=op["height"] if op else .305,
            ticks=5)
        self.result = self.host.step(feedback)
        effort = torch.zeros_like(q)
        for entry, value in zip(entries, self.result["effort_api"]):
            effort[:, self.ids[entry["target"]]] = entry["sign"] * value
        active_qdot = dq[:, self.active_ids]
        active_effort = self.wheels.apply_torque(effort[:, self.active_ids], active_qdot)
        # Double-down clears the owned wheel command-delay history too.
        if not enabled:
            self.wheels.reset()
            active_effort.zero_()
        effort[:, self.active_ids] = active_effort
        effort[:, self.ids["L_joint3"]] += self.wheels.passive_effort[:, 0]
        effort[:, self.ids["R_joint3"]] += self.wheels.passive_effort[:, 1]
        effort[:, self.leg_ids] += self.friction.effort(dq[:, self.leg_ids])
        compression = q.new_tensor([self.model.spec["spring_binding"][n]["compression_at_q_zero_m"] for n in self.springs]) - q[:, self.gas_ids]
        effort[:, self.gas_ids] = self.gas.physical_force(compression)
        # Gimbal remains a physical servo. Its reference is derived from the
        # same RMCS joystick/switch semantics, not copied from real measured q.
        if snapshot is not None and snapshot.remote:
            for index, desired in zip((self.ids["M_joint1"], self.ids["M_joint2"]), self.gimbal_target):
                error = (desired - float(q[0, index]) + np.pi) % (2*np.pi) - np.pi
                cap = 2. if index == self.ids["M_joint1"] else 10.
                effort[:, index] = np.clip(12.*error - .3*float(dq[0, index]), -cap, cap)
        if self.version3:
            self.robot.set_joint_effort_target_index(target=effort)
        else:
            self.robot.set_joint_effort_target(effort)
        self.robot.write_data_to_sim()
        self.sim.step(render=False)
        self.robot.update(self.dt)
        self.steps += 1

    def close(self):
        if self.host:
            self.host.close()
            self.host = None
