"""Check the unified model/control entry point, including compiled joint dynamics."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from build_run002_bundle import REVISION, PROFILE, MODEL_FILE
from control_profile import ROOT, DEFAULT_PROFILE, _bound_path, load_control_profile, sha256


def validate_training_bundle(root: Path = ROOT):
    root = Path(root).resolve()
    bundle = json.loads((root / 'TRAINING_BUNDLE.json').read_text())
    if (bundle['schema'] != 'open_sprite_recommended_training_bundle_v1'
            or bundle['asset_revision'] != REVISION
            or bundle['model'] != f'models/{REVISION}/{MODEL_FILE}'
            or bundle['control_profile'] != DEFAULT_PROFILE.as_posix()
            or bundle['dynamics'] != f'models/{REVISION}/dynamics.json'
            or bundle['policy_hz'] != 50 or bundle['simulation_pd_hz'] != 500
            or bundle['hardware_qualified'] is not False):
        raise ValueError('Recommended model/control pointer mismatch')
    model_path = _bound_path(root, bundle['model'])
    profile = load_control_profile(bundle['control_profile'], repository_root=root)
    if sha256(model_path) != bundle['model_sha256'] or profile.sha256 != bundle['profile_sha256']:
        raise ValueError('Recommended bundle hash mismatch')
    if profile.data['profile_id'] != PROFILE:
        raise ValueError('Wrong recommended control profile')
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if (model.nq, model.nv, model.nu, model.njnt) != (38, 37, 0, 32):
        raise ValueError('Expected floating31-joint external-PD model')
    if model.opt.timestep != .002:
        raise ValueError('Compiled model must integrate at500Hz')
    dynamics = json.loads(_bound_path(root, bundle['dynamics']).read_text())
    hinges = {model.joint(j).name: j for j in range(model.njnt)
              if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE}
    profile.for_joint_names(tuple(hinges))
    for name, jid in hinges.items():
        dof = model.jnt_dofadr[jid]
        expected = dynamics['joints'][name]
        for attr, key in (('dof_armature', 'armature'), ('dof_damping', 'damping'),
                          ('dof_frictionloss', 'frictionloss')):
            if not np.isclose(getattr(model, attr)[dof], float(expected[key]), rtol=0, atol=1e-12):
                raise ValueError(f'Compiled joint {key} mismatch: {name}')
        if not np.allclose(model.jnt_range[jid], profile.data['joints'][name]['mechanical_position_limit_rad'], rtol=0, atol=1e-12):
            raise ValueError(f'Compiled mechanical range mismatch: {name}')
        if ('shoulder_pitch' in name or 'shoulder_roll' in name) and model.dof_armature[dof] != .032:
            raise ValueError(f'Wrong four-J4340P shoulder armature: {name}')
    physical = [g for g in range(model.ngeom) if (model.geom(g).name or '').endswith('_collision')]
    if len(physical) != 32 or any(model.geom_contype[g] != 1 or model.geom_conaffinity[g] != 1 for g in physical):
        raise ValueError('Physical collision masks must be explicitly enabled')
    if model.body_mass[model.body('waist_yaw_link').id] != 3.7391:
        raise ValueError('Wrong waist mass')
    print(f'OK unified training bundle {REVISION} joints=31 shoulder_armature=0.032 PD=500Hz policy=50Hz')
    return model, profile


if __name__ == '__main__':
    validate_training_bundle()
