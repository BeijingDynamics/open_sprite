"""Package the verified run002 simulation model, not a new dynamics revision."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
REVISION = 'sprite0825_v5_supermimic_run002_20261106'
PROFILE = 'sprite0825_run002_serial500_20261106'
BASE = 'sprite0825_v5_4340_shoulders_waist20261105'
SOURCE_SHA256 = '2cdfb00df9867076271d49dd699c95e2a8ec2f4c6d88be25d5c9ccc6d73fd9e1'
SEMANTIC_SHA256 = 'c711ca53909441deb15528a4c5413d63e002059e23d546ec8ba9b64b190658c3'
CHECKPOINT_SHA256 = '094b52267159d26f4d5bfb41edc5823a9becc32f706c87df05f8aa36b4016282'
MODEL_FILE = 'mujoco/sprite0825_v5_external_pd.xml'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8', newline='\n')


def xml_bytes(root):
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='utf-8', xml_declaration=True) + b'\n'


def semantic_sha(root):
    root = copy.deepcopy(root)
    root.find('compiler').set('meshdir', 'PORTABLE_MESH_ROOT')
    return hashlib.sha256(xml_bytes(root)).hexdigest()


def dynamics(root):
    joints = {j.get('name'): {k: j.get(k, v) for k, v in (
        ('axis', '0 0 1'), ('range', '0 0'), ('armature', '0'),
        ('damping', '0'), ('frictionloss', '0'))}
        for j in root.findall('.//joint') if j.get('type') != 'free'}
    return dict(schema='open_sprite_serial_model_dynamics_v1',
        units=dict(armature='kg*m^2, joint-side reflected inertia', damping='N*m*s/rad'),
        source_model_sha256=SOURCE_SHA256,
        model_semantic_sha256=semantic_sha(root), joints=joints,
        interpretation='Nominal simulation parameters, not measured motor identification or hardware commands',
        contact='Exact run002 collision primitives, masks, exclusions and local friction are in the bound MJCF',
        urdf_scope='URDF is the CAD/kinematic description; it cannot encode MuJoCo armature/contact exclusions. Use the bound MJCF for run002 replay.')


def build(source, root=ROOT):
    if hashlib.sha256(source).hexdigest() != SOURCE_SHA256:
        raise ValueError('Not the verified run002 training model')
    asset = root / 'models' / REVISION
    if asset.exists():
        raise FileExistsError('Refuse to overwrite a published model revision')
    shutil.copytree(root / 'models' / BASE, asset)
    tree = ET.fromstring(source)
    original_semantic = semantic_sha(tree)
    if original_semantic != SEMANTIC_SHA256:
        raise ValueError('Run002 model semantics differ from the verified source')
    tree.find('compiler').set('meshdir', '../')
    (asset / MODEL_FILE).write_bytes(xml_bytes(tree))
    assert semantic_sha(tree) == original_semantic
    locked = copy.deepcopy(tree)
    for body in locked.findall('.//body'):
        for joint in list(body.findall('joint')):
            body.remove(joint)
    (asset / 'mujoco/sprite0825_v5_visual_locked.xml').write_bytes(xml_bytes(locked))
    # The CAD URDF and mesh bytes are deliberately retained, not rounded to MJCF.
    write_json(asset / 'dynamics.json', dynamics(tree))
    files = {p.relative_to(asset).as_posix(): sha(p) for p in sorted(asset.rglob('*'))
             if p.is_file() and p.name != 'manifest.json'}
    manifest = dict(schema='open_sprite_public_robot_asset_v3', asset_revision=REVISION,
        base_asset=BASE, source='Completed SuperMimic run002 archived training source',
        source_model_sha256=SOURCE_SHA256, model_semantic_sha256=original_semantic,
        source_checkpoint_sha256=CHECKPOINT_SHA256,
        urdf='sprite0825_float.urdf', urdf_sha256=sha(asset / 'sprite0825_float.urdf'),
        canonical_training_mjcf=MODEL_FILE, dynamics='dynamics.json',
        mesh_count=33, revolute_joint_count=31,
        packaging_changes=['Only compiler.meshdir is relocated in the training MJCF; XML formatting is normalized',
            'Inspection model removes joints; scenes and CAD URDF are retained from the waist revision'],
        urdf_scope='Same CAD/kinematics; source MJCF contains rounded transforms/inertials and generated primitives. URDF import alone does not reproduce run002 dynamics.',
        shoulder_armature_kg_m2=0.032, waist_mass_kg=3.7391,
        hardware_qualified=False, files_sha256=files)
    write_json(asset / 'manifest.json', manifest)
    old = root / 'control_profiles/sprite0825_serial_training_20261105.json'
    profile = json.loads(old.read_text())
    profile.update(schema='open_sprite_training_control_profile_v2', profile_id=PROFILE)
    profile['model_binding'] = dict(asset_revision=REVISION, asset_directory=f'models/{REVISION}',
        files_sha256={f'models/{REVISION}/{p}': sha(asset / p) for p in (
            'manifest.json', 'sprite0825_float.urdf', MODEL_FILE,
            'mujoco/sprite0825_v5_visual_locked.xml', 'dynamics.json')},
        same_complete_model_as_source_training=True,
        scope='Training MJCF is semantically identical to the verified run002 source after mesh-path relocation; URDF is a CAD interchange artifact, not a training model replacement.',
        canonical_training_mjcf=MODEL_FILE,
        source_training_model_sha256=SOURCE_SHA256,
        model_semantic_sha256=original_semantic)
    parent_provenance = profile['provenance']
    profile['provenance'] = dict(parent_profile_sha256=sha(old),
        control_values_origin='Unchanged31-joint values from the parent profile; its adapter ancestry is historical, not the run002 training implementation',
        source_joint_fields=parent_provenance['source_joint_fields'],
        source_joint_values_sha256=parent_provenance['source_joint_values_sha256'],
        canonicalization=parent_provenance['canonicalization'],
        source_training_model_sha256=SOURCE_SHA256,
        run002_checkpoint_sha256=CHECKPOINT_SHA256,
        verification='Run002 copied the parent named joint values unchanged; this revision binds its actual model and explicit500Hz timing.')
    profile['timing'].update(simulation_pd_hz=500, physics_dt_s=.002, decimation=10)
    profile['timing']['command_delay'].update(max_delay_s=.002, refresh_physics_ticks=10)
    profile['warnings'][0] = 'Exact run002 simulation asset/control pair; simulation success is not hardware qualification.'
    write_json(root / f'control_profiles/{PROFILE}.json', profile)
    assets = [f'{sha(p)}  {p.relative_to(root).as_posix()}'
              for p in sorted((root / 'models').rglob('*')) if p.is_file()]
    (root / 'ASSET_SHA256SUMS').write_text('\n'.join(assets) + '\n', encoding='ascii', newline='\n')
    profiles = [f'{sha(p)}  {p.name}' for p in sorted((root / 'control_profiles').glob('*.json'))]
    (root / 'control_profiles/SHA256SUMS').write_text('\n'.join(profiles) + '\n', encoding='ascii', newline='\n')
    write_json(root / 'TRAINING_BUNDLE.json', dict(schema='open_sprite_recommended_training_bundle_v1',
        asset_revision=REVISION, model=f'models/{REVISION}/{MODEL_FILE}',
        control_profile=f'control_profiles/{PROFILE}.json',
        model_sha256=sha(asset / MODEL_FILE), profile_sha256=sha(root / f'control_profiles/{PROFILE}.json'),
        dynamics=f'models/{REVISION}/dynamics.json', policy_hz=50, simulation_pd_hz=500,
        source_checkpoint_sha256=CHECKPOINT_SHA256, hardware_qualified=False,
        note='Unified default for new simulation work; existing checkpoints must retain their original model/profile hashes. No weights are included.'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', type=Path, required=True)
    args = parser.parse_args()
    with tarfile.open(args.source_archive) as archive:
        source = archive.extractfile('assets/opensprite/opensprite.xml').read()
    build(source)
    print('BUILT verified run002 model + named500Hz controls; no training or hardware access')


if __name__ == '__main__':
    main()
