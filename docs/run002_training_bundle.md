# Unified run002 simulation model and control bundle

## Why this publication exists

The November5 publication updated the waist and exported named controls, but
did not unify the complete training MJCF and effective timing. It explicitly
bound reusable controls to a different public model and retained a200Hz snapshot.
That was insufficient as a single source for cooperating training projects.

The public v5 four-J4340P shoulder MJCF already used0.032kg m^2 armature.
Completed supermimic_run/run002 used that value. Our separate serial-training
derivative retained0.0018kg m^2 on four shoulder pitch/roll joints. Therefore
the difference is not evidence that run002 used an obsolete public shoulder
model, nor a reason by itself to discard its trained policy. Armature is joint-
side reflected inertia, not Kp, Kd, a link inertia tensor or a torque limit.
These simulation values are not newly measured hardware identification.

## Recommended entry point

Use the root `TRAINING_BUNDLE.json` for new simulation work. It binds:

- `models/sprite0825_v5_supermimic_run002_20261106/mujoco/sprite0825_v5_external_pd.xml`
- `control_profiles/sprite0825_run002_serial500_20261106.json`
- the named `dynamics.json`, mesh set and manifest in the same model directory.

The portable training MJCF preserves ALL run002 source model semantics; only
compiler meshdir and XML formatting differ. Body transforms/inertials, joints,
armatures, primitive collisions, masks, exclusions and solver options are not
retuned. The original archived source SHA256 is:

`2cdfb00df9867076271d49dd699c95e2a8ec2f4c6d88be25d5c9ccc6d73fd9e1`

The source checkpoint is run002 model15398, SHA256:

`094b52267159d26f4d5bfb41edc5823a9becc32f706c87df05f8aa36b4016282`

Weights and motion datasets are NOT included. This model/control identity
does not establish hardware readiness or replace runtime safety limits.

## Load the pair, not independent defaults

```python
from pathlib import Path
import sys
import mujoco

repo = Path('/path/to/open_sprite').resolve()
sys.path.insert(0, str(repo / 'scripts'))
from control_profile import load_control_profile

profile = load_control_profile(repository_root=repo)
binding = profile.data['model_binding']
model_path = repo / binding['asset_directory'] / binding['canonical_training_mjcf']
model = mujoco.MjModel.from_xml_path(str(model_path))
joint_names = [model.joint(j).name for j in range(model.njnt)
               if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
controls = profile.for_joint_names(joint_names)
```

The loader rejects incomplete/mixed model hashes, missing or repeated joint
names, changed control values, stale200Hz timing, and changed model semantics.
It reads the bound dynamics table by joint name. Do not take gains from the
new profile and a robot XML from another directory.

At deployment and training, explicitly preserve the31-joint ordering, action
offset/scale/clipping and controller torque saturation. The model has no
actuators (`nu=0`): a qfrc_applied external PD loop must clamp torque itself.
URDF effort metadata and MJCF actuatorfrcrange are not substitutes for the
profile's actual applied caps. A framework-created actuator must override
inherited lower caps with the profile, rather than silently clip at CAD limits.

## Timing and dynamics

| Item | Recommended run002 bundle |
| --- | --- |
| Policy | 50Hz,20ms |
| Physics / serial-joint PD | 500Hz,2ms |
| Substeps | 10 |
| Command delay | Original tracker stochastic buffer,0/1tick =0/2ms |
| Delay refresh | Every10physics ticks,20ms; hold probability0.9 |
| First-order torque lag | Disabled |
| Four shoulder pitch/roll armatures | 0.032kg m^2 |
| Shoulder yaw armatures | 0.0018kg m^2 |
| Waist yaw link mass | 3.7391kg |
| Kp/Kd, torque caps, action offsets | All31 unchanged from the published named table |

`dynamics.json` records every joint's nominal armature, passive damping,
friction and range. Passive model damping is separate from controller Kd.
The URDF is retained byte-for-byte from the published CAD waist revision;
URDF does not contain MuJoCo armature, generated contact primitives or
exclusions. run002's MJCF has rounded source transforms/inertials. Do not
replace it with a fresh URDF import while claiming exact reproduction.

The bundle defines nominal robot/control identity, not the entire training
experiment. Scene floor friction, contact priority, domain randomization,
reward, reference data and observation layout must also match the selected
training recipe. run002's full-body scene sets physical collision masks1/1,
condim3, priority1 and friction0.8. Keep randomized parameters and realized
actuator configuration in each experiment's own archived contract.

## Validate before use

```bash
python scripts/control_profile.py
python scripts/validate_models.py
python scripts/validate_training_bundle.py
python -m unittest discover -s scripts -v
```

All validation is offline. It does not connect to Jetson, IMU or CAN, load a
policy, enable a motor or reset a motor zero. Model checks include the compiled
31-joint armatures/damping/friction/ranges and active collision masks, not just
URDF mass or Kp/Kd tables. Source-semantic checks also cover all body inertials
and transforms, including off-diagonal inertia terms and contact exclusions.

## Existing runs and runtime

Keep completed run002 and its original model/profile/archive hashes unchanged.
Its source MJCF already matches this newly packaged training model. This
publication alone does not require retraining it. Do not relabel the original
200Hz profile as500Hz inside old result archives; run002's explicit override
remains valid provenance.

The older `sprite0825_serial_training_20261105.json` remains byte-identical and
loadable by explicit path. Historical CAD, v4/v5 and waist-only assets are not
rewritten. Our separate0.0018shoulder experiment is distinct evidence; it has
not been silently migrated by this publication.

No current training process, checkpoint, runtime gain, motor mapping,
parallel-ankle/head controller, IMU reader, hardware safeguard or calibration
is modified. New experiments should record `TRAINING_BUNDLE.json` plus model,
profile and trainer hashes before starting. Existing runtime integration still
requires policy-interface parity and platform-specific shadow validation.
