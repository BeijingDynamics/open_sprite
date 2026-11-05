"""Load named serial-training controls; no training, simulator or hardware access."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = Path('control_profiles/sprite0825_serial_training_20261105.json')
SOURCE_FIELDS = ('kp', 'kd', 'effort_nm', 'velocity_rad_s', 'default_pos_rad',
                 'archived_command_min_rad', 'archived_command_max_rad')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


def _constant(value):
    raise ValueError(f'Nonfinite JSON constant: {value}')


def _json(path):
    return json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=_pairs, parse_constant=_constant)


def _number(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be >= {minimum}')
    return float(value)


def _bound_path(root: Path, relative: str) -> Path:
    path = PurePosixPath(relative)
    if not relative or '\\' in relative or path.is_absolute() or any(x in ('.', '..') for x in path.parts):
        raise ValueError(f'Invalid repository-relative path: {relative}')
    resolved = (root / relative).resolve()
    if not resolved.is_relative_to(root.resolve()):
        raise ValueError(f'Path leaves repository: {relative}')
    return resolved


@dataclass(frozen=True)
class JointControl:
    kp: float
    kd: float
    command_effort_nm: float
    velocity_limit_rad_s: float
    default_pos_rad: float

    @property
    def urdf_velocity_rad_s(self) -> float:
        """Tracker compatibility name; this value is the PROFILE threshold, not URDF metadata."""
        return self.velocity_limit_rad_s


@dataclass(frozen=True)
class ControlProfile:
    data: dict
    joints: Mapping[str, JointControl]
    sha256: str

    def for_joint_names(self, names: Sequence[str]) -> dict[str, JointControl]:
        names = tuple(names)
        if len(names) != len(set(names)):
            raise ValueError('Duplicate requested joint names')
        missing = sorted(set(self.joints) - set(names))
        unknown = sorted(set(names) - set(self.joints))
        if missing or unknown:
            raise ValueError(f'Joint set mismatch: missing={missing}; unknown={unknown}')
        return {name: self.joints[name] for name in names}


def validate_profile_data(data: dict, repository_root: Path = ROOT) -> dict[str, JointControl]:
    """Validate decoded content and its asset binding; callers normally use load_control_profile."""
    if data.get('schema') != 'open_sprite_training_control_profile_v1':
        raise ValueError('Unsupported control profile schema')
    if data.get('hardware_qualified') is not False or data.get('status') != 'frozen_training_snapshot_not_hardware_qualified':
        raise ValueError('Training snapshot must not grant hardware qualification')
    expected_units = dict(kp='N*m/rad', kd='N*m*s/rad', effort_nm='N*m', velocity_rad_s='rad/s',
                          default_pos_rad='rad', position_limits='rad')
    if data.get('units') != expected_units:
        raise ValueError('Unexpected control units')
    root = Path(repository_root).resolve()
    binding = data['model_binding']
    asset = _bound_path(root, binding['asset_directory'])
    if not binding['files_sha256']:
        raise ValueError('Missing model hash binding')
    required = {f"{binding['asset_directory']}/{name}" for name in (
        'manifest.json', 'sprite0825_float.urdf', 'mujoco/sprite0825_v5_external_pd.xml',
        'mujoco/sprite0825_v5_visual_locked.xml')}
    if set(binding['files_sha256']) != required:
        raise ValueError('Incomplete model hash binding')
    for relative, expected in binding['files_sha256'].items():
        if sha256(_bound_path(root, relative)) != expected:
            raise ValueError(f'Model hash mismatch: {relative}')
    manifest = _json(asset / 'manifest.json')
    if manifest['asset_revision'] != binding['asset_revision']:
        raise ValueError('Asset revision mismatch')
    for relative, expected in manifest['files_sha256'].items():
        path = _bound_path(asset, relative)
        if sha256(path) != expected:
            raise ValueError(f'Asset member hash mismatch: {relative}')
    urdf = ET.parse(asset / 'sprite0825_float.urdf').getroot()
    mechanical = {j.get('name'): j for j in urdf.findall('joint') if j.get('type') == 'revolute'}
    values = data['joints']
    if len(values) != 31 or set(values) != set(mechanical):
        raise ValueError('Profile must name exactly the31 bound URDF revolute joints')
    result = {}
    for name, v in values.items():
        missing = set(SOURCE_FIELDS) - set(v)
        if missing:
            raise ValueError(f'Missing control fields for {name}: {sorted(missing)}')
        for key in SOURCE_FIELDS:
            _number(v[key], f'{name}.{key}', minimum=0 if key in ('kp', 'kd') else None)
        if v['effort_nm'] <= 0 or v['velocity_rad_s'] <= 0:
            raise ValueError(f'{name} requires positive torque/velocity thresholds')
        limits = mechanical[name].find('limit')
        lo, hi = (float(limits.get(k)) for k in ('lower', 'upper'))
        if v['mechanical_position_limit_rad'] != [lo, hi]:
            raise ValueError(f'Mechanical limit mismatch: {name}')
        if not lo <= v['default_pos_rad'] <= hi:
            raise ValueError(f'Default position outside mechanical limits: {name}')
        if not v['archived_command_min_rad'] < v['archived_command_max_rad']:
            raise ValueError(f'Archived bounds invalid: {name}')
        for key, attribute in (('urdf_effort_metadata_nm', 'effort'), ('urdf_velocity_metadata_rad_s', 'velocity')):
            if _number(v[key], f'{name}.{key}') != float(limits.get(attribute)):
                raise ValueError(f'URDF metadata mismatch: {name}.{key}')
        result[name] = JointControl(v['kp'], v['kd'], v['effort_nm'], v['velocity_rad_s'], v['default_pos_rad'])
    if set(data['provenance']['source_joint_fields']) != set(SOURCE_FIELDS):
        raise ValueError('Unexpected source joint fields')
    source = {name: {key: v[key] for key in SOURCE_FIELDS} for name, v in values.items()}
    encoded = json.dumps(source, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    if hashlib.sha256(encoded).hexdigest() != data['provenance']['source_joint_values_sha256']:
        raise ValueError('Published controls differ from frozen source values')
    timing = data['timing']
    for key in ('policy_hz', 'policy_dt_s', 'simulation_pd_hz', 'physics_dt_s', 'decimation'):
        if _number(timing[key], f'timing.{key}', minimum=0) == 0:
            raise ValueError('Timing values must be positive')
    if type(timing['decimation']) is not int:
        raise ValueError('Decimation must be an integer')
    for a, b in ((timing['policy_hz'] * timing['policy_dt_s'], 1),
                 (timing['simulation_pd_hz'] * timing['physics_dt_s'], 1),
                 (timing['decimation'] * timing['physics_dt_s'], timing['policy_dt_s'])):
        if not math.isclose(a, b, rel_tol=0, abs_tol=1e-12):
            raise ValueError('Inconsistent control timing')
    if not math.isclose(_number(data['action']['scale_rad'], 'action.scale_rad'), .25, rel_tol=0, abs_tol=1e-12):
        raise ValueError('Unexpected snapshot action scale')
    if data['action']['raw_clip'] != [-20, 20] or data['action']['joint_target_projection_enabled'] is not False:
        raise ValueError('Unexpected snapshot action clipping/projection')
    controller = data['controller']
    if controller['space'] != 'serial_joint' or controller['torque_lag_enabled'] is not False:
        raise ValueError('Snapshot must use serial PD without torque lag')
    for key in ('velocity_target_rad_s', 'feedforward_torque_nm'):
        if _number(controller[key], 'controller.' + key) != 0:
            raise ValueError('Snapshot velocity target/feedforward must be zero')
    for key in ('gain_randomization_enabled', 'effort_randomization_enabled', 'motor_mapping',
                'mit_quantization', 'runtime_watchdogs', 'parallel_linkage_conversion'):
        if controller[key] is not False:
            raise ValueError('Unexpected runtime emulation or PD/effort randomization')
    expected_timing = dict(policy_hz=50, policy_dt_s=.02, simulation_pd_hz=200, physics_dt_s=.005, decimation=4)
    if any(timing[key] != value for key, value in expected_timing.items()):
        raise ValueError('Timing differs from frozen training snapshot')
    delay = timing['command_delay']
    expected_delay = dict(enabled=True, min_physics_ticks=0, max_physics_ticks=1,
                          min_delay_s=0.0, max_delay_s=.005, hold_probability=.9,
                          refresh_physics_ticks=4, refresh_period_s=.02)
    if any(delay[key] != value or type(delay[key]) is not type(value) for key, value in expected_delay.items()):
        raise ValueError('Command delay differs from frozen training snapshot')
    if controller['torque_time_constant_s'] != [0, 0]:
        raise ValueError('Unexpected snapshot torque time constant')
    if binding['same_complete_model_as_source_training'] is not False:
        raise ValueError('Do not claim public/source training model identity')
    return result


def load_control_profile(path: Path | str = DEFAULT_PROFILE, *, repository_root: Path = ROOT) -> ControlProfile:
    root = Path(repository_root).resolve()
    path = Path(path)
    path = path.resolve() if path.is_absolute() else _bound_path(root, path.as_posix())
    if not path.is_relative_to(root / 'control_profiles'):
        raise ValueError('Control profile must be inside repository control_profiles/')
    entries = {}
    for line in (root / 'control_profiles/SHA256SUMS').read_text(encoding='ascii').splitlines():
        expected, relative = line.split('  ', 1)
        if relative in entries:
            raise ValueError('Duplicate control checksum entry')
        entries[relative] = expected
    relative = path.relative_to(root / 'control_profiles').as_posix()
    actual = sha256(path)
    if entries.get(relative) != actual:
        raise ValueError('Control profile checksum mismatch or profile not listed')
    data = _json(path)
    controls = validate_profile_data(data, root)
    return ControlProfile(data, MappingProxyType(controls), actual)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', nargs='?', default=str(DEFAULT_PROFILE))
    args = parser.parse_args()
    profile = load_control_profile(args.profile)
    print(json.dumps(dict(profile_id=profile.data['profile_id'], profile_sha256=profile.sha256,
        asset_revision=profile.data['model_binding']['asset_revision'], joints=len(profile.joints),
        policy_hz=profile.data['timing']['policy_hz'], simulation_pd_hz=profile.data['timing']['simulation_pd_hz'],
        hardware_qualified=False, passed=True, warnings=profile.data['warnings']), indent=2))


if __name__ == '__main__':
    main()
