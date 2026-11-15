"""Deployment header identity and portable artifact paths, without Isaac or a learner."""
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def fixture():
    gas = {'force_extension': {'low_compression_rate_per_m': 5., 'high_compression_slope_n_per_m': 10.},
           'compression_knots_m': [.01, .02], 'force_knots_n': [10., 20.], 'sides': {}}
    for side in ['left', 'right']:
        gas['sides'][side] = dict(delta_rad=[-.1, 1.4], compression_m=[.02, .01],
                                 length_jacobian_m_per_rad=[[-.1, .1], [-.2, .2]])
    raw = json.dumps(gas).encode()
    contract = dict(gas_model_sha256=hashlib.sha256(raw).hexdigest(), training_source_commit='a' * 40,
        deployment_contract_sha256='b' * 64,
        command=dict(forward_limit_m_s=2., yaw_limit_rad_s=10., height_min_m=.2, height_max_m=.5, height_rate_m_s=.6),
        control=dict(leg_kp_nm_per_rad=80., leg_kd_nm_s_per_rad=1.5, virtual_pair_overtravel_rad=.2,
            virtual_pair_overtravel_scope='jump-request', tuck_spring_compensation=dict(mode='model', gain=.75,
                cap_nm=16., error_on_rad=.02, error_full_rad=.1, inward_pd_full_nm=2.)),
        request=dict(standing_height_m=.305, crouch_height_m=.25, release_height_m=.39, crouch_seconds=.35,
            outcome_seconds=4.5, nominal_request_delay_s=.8, apex_range_m=[.42, .75], moving_vx_max_m_s=.5),
        real_state_soft_limits_rad=[-.1, 1.4], winding_bounds_rad=[[-.1, 1.4], [-.1, 1.4]])
    return contract, raw


def test_header_uses_selected_deployment_identity_and_apex():
    contract, raw = fixture()
    header = module('generate_jump_policy_header').generate(contract, raw)
    assert 'kDeploymentContractSha256 = "' + contract['deployment_contract_sha256'] + '"' in header
    assert 'kMaximumApex = 0.75;' in header
    assert 'kLegKp = 80;' in header
    assert 'jump_v4' not in header


@pytest.mark.parametrize('case', ['gas_hash', 'identity', 'apex', 'bounds', 'nonfinite'])
def test_inconsistent_contract_rejected(case):
    contract, raw = fixture()
    if case == 'gas_hash':
        contract['gas_model_sha256'] = '0' * 64
    elif case == 'identity':
        contract['deployment_contract_sha256'] = 'invalid'
    elif case == 'apex':
        contract['request']['apex_range_m'] = [.2, .3]
    elif case == 'bounds':
        contract['winding_bounds_rad'][0][0] = -.2
    else:
        contract['control']['leg_kp_nm_per_rad'] = float('nan')
    with pytest.raises(ValueError):
        module('generate_jump_policy_header').generate(contract, raw)


def test_generic_paths_require_an_explicit_external_root(tmp_path):
    exporter = module('export_wheelbipe_port_onnx')
    assert exporter.generic_path(ROOT / 'contracts/policy.json') == 'isaac_wheeled_rl_train:contracts/policy.json'
    assert exporter.generic_path(tmp_path / 'checkpoint.pt', [('training', tmp_path)]) == 'training:checkpoint.pt'
    with pytest.raises(ValueError, match='path-root'):
        exporter.generic_path(tmp_path / 'checkpoint.pt')
