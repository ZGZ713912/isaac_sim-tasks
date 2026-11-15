# RMCS jump policy data

The refactored RMCS controller uses the generic `jump_policy.hpp` data namespace. Select a
behavior-qualified jump export (v7, v10 or a later compatible policy) before changing a real
robot deployment. The 16001 foundation-v2 policy remains with the old RMCS controller and its
`wheel-leg-16001-stable` tag. Do not select it for this jump controller.

`scripts/generate_jump_policy_header.py` generates a header from an export's deployment contract
and the exact gas table pinned by that contract. It does not export an actor or qualify behavior.
Use a new output path, format with the target RMCS `.clang-format`, review the generated diff,
and install the header together with the matching ONNX and exact expected-metadata pins.

```sh
python scripts/generate_jump_policy_header.py \
  --contract .codex/artifacts/selected-jump/deployment_contract.json \
  --gas-model model/wheel_leg_v2.0.2/friction_v1/isaac/gas_spring_model.json \
  --output .codex/artifacts/selected-jump/jump_policy.hpp
```

The deployment contract must include `training_source_commit`, `gas_model_sha256`,
`winding_bounds_rad`, `real_state_soft_limits_rad`, and the following blocks. The accepted
export must obtain these values from its frozen training contract, with no local defaults:

- `command`: `forward_limit_m_s`, `yaw_limit_rad_s`, `height_min_m`, `height_max_m`, `height_rate_m_s`.
- `control`: `leg_kp_nm_per_rad`, `leg_kd_nm_s_per_rad`, `virtual_pair_overtravel_rad`,
  `virtual_pair_overtravel_scope = jump-request`, and `tuck_spring_compensation` with
  `mode = model`, `gain`, `cap_nm`, `error_on_rad`, `error_full_rad`, `inward_pd_full_nm`.
- `request`: `standing_height_m`, `crouch_height_m`, `release_height_m`, `crouch_seconds`,
  `outcome_seconds`, `nominal_request_delay_s`, `apex_range_m`, `moving_vx_max_m_s`.

The generated apex range is checked by the RMCS jump command. Include the full range required
by all exported request modes, including the stair slot and platform jumps. The generator
refuses a gas hash mismatch, inconsistent winding bounds and invalid interpolation tables.
It preserves both side tables and uses full double precision. It makes no assumption about a
particular jump version. A changed observation or request ABI still requires an explicit RMCS
controller update; generating this header alone does not implement a stair request protocol.

Before formal export exists, the compiled RMCS data are an offline reference. The production
YAML retains a pending-contract sentinel and a missing `jump_policy.onnx` path. The pilot-derived
fixture belongs only under the local AI directory and its `policy_hz` pin is the exact string
`"50"`. Header generation, build success and offline parity do not constitute bench or robot
acceptance.

The actor exporter accepts `--rmcs-contract`, `--gas-model` and repeatable `--path-root NAME=DIR`.
It writes exact deployment/observation/model pins and the checkpoint SHA to ONNX metadata,
generates `jump_policy.hpp` in the same new output directory, and verifies 1024 identical
inference outputs before and after metadata changes. Its contract receipt uses repository names
instead of host paths. `--allow-pilot` always writes `pilot=true`; a pilot remains unqualified for
hardware deployment. A receipt with an unmapped absolute path is rejected. The generated
`kDeploymentContractSha256` and ONNX `wheelbipe_deployment_contract_sha256` use the same field.
