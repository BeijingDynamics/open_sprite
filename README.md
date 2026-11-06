# Open Sprite

Open robot-description assets and reproducible simulation entry points for the
Sprite humanoid robot. The current public model is **Sprite0825**, a roughly
0.95 m, 31-actuated-joint humanoid with a V-shaped pelvis.

The recommended simulation bundle is **the completed SuperMimic run002 model
and its paired 500 Hz serial-PD control profile (2026-11-06)**. Start from
[`TRAINING_BUNDLE.json`](TRAINING_BUNDLE.json), not an independently chosen
URDF and controller table. Its four proximal shoulder DM-J4340P joints use
**armature 0.032 kg m^2**, not 0.0018. Its
`waist_yaw_link` mass is **3.7391 kg**, not the historical 2.070928 kg.
Left/right shoulder pitch and roll use J4340P; shoulder yaw is not part of
that upgrade. This is a versioned robot-description release, not a policy bundle.

This repository contains:

- the 2026-08-25 meshes;
- the portable sanitized v5 URDF and its complete referenced mesh set;
- a stable one-command MuJoCo visual model;
- the floating-base external-PD MuJoCo model used for controller development;
- versioned source provenance and SHA-256 checksums;
- a named-joint control profile exported from current serial training, with a strict loader;
- the original v4 and historical v5 models, retained unchanged for reproducibility.

The historical v5 assets were extracted from the public G74 `model5999` candidate in
[`open_sprite_rl`](https://github.com/BeijingDynamics/open_sprite_rl/tree/07a513188de64572e891ab21cacb6c79e083b044/candidates/sprite0825_stage2_g74_model5999_4340_shoulders).
That provenance does **not** make this the external G74 `model3000` deployment
package referenced by a runtime launcher. The historical waist-only revision
remains unchanged. The new recommended training MJCF additionally packages
run002's actual generated collision primitives, exclusions and inertial values;
only mesh-path relocation and XML formatting differ from its archived source.
This does not qualify an old policy for a new model or authorize hardware. See
[`docs/asset_versions.md`](docs/asset_versions.md) for the exact distinction.

## Quick start: view Sprite0825 in MuJoCo

Python 3.10 or newer is recommended.

### Linux/macOS

```bash
git clone https://github.com/BeijingDynamics/open_sprite.git
cd open_sprite
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m mujoco.viewer --mjcf=models/sprite0825_v5_supermimic_run002_20261106/mujoco/scene.xml
```

### Windows PowerShell

```powershell
git clone https://github.com/BeijingDynamics/open_sprite.git
cd open_sprite
py -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python -m mujoco.viewer --mjcf=models/sprite0825_v5_supermimic_run002_20261106/mujoco/scene.xml
```

The default scene is intentionally joint-locked so the first viewer command
shows the designed standing geometry instead of an uncontrolled floating-base
robot falling under gravity. It is a geometry inspection scene at zero joint
coordinates, not active balancing or a demonstration of policy performance.

On macOS, launch the viewer with the environment's `mjpython` executable
instead of `python` (MuJoCo's macOS GUI requirement).

## Validate the models

```bash
python scripts/validate_models.py
```

The validator checks the recommended training bundle, historical v5 and legacy v4 scenes,
URDF mesh paths and asset hashes. It also checks the 31 hinges, four upgraded
shoulders, URDF/MJCF axes and position limits, and the latest waist mass,
COM and full inertia tensor in both the URDF and compiled MuJoCo models.
No robot connection is needed. It also checks all 31 compiled joint armatures,
passive damping, friction and ranges against the bound dynamics table, explicit
collision masks, the 500 Hz timestep, and the unified model/profile pointer.

## Training controls

The current training Kp/Kd, torque caps, default joint positions, velocity-cost
thresholds, action scale and timing are published separately from the URDF:

- [`TRAINING_BUNDLE.json`](TRAINING_BUNDLE.json): the model/profile pair for new training
- [`control_profiles/sprite0825_run002_serial500_20261106.json`](control_profiles/sprite0825_run002_serial500_20261106.json)
- [`docs/training_control_profile.md`](docs/training_control_profile.md), including named-joint loading and `supermimic_run` integration

```bash
python scripts/control_profile.py
```

The default profile binds the run002 training model by SHA256, rejects unknown or
missing joint names, and preserves the exact current training values. It is a
**training snapshot, not hardware-qualified gains or a walking-policy release**.
This profile uses **50 Hz policy updates, 500 Hz simulation PD, 2 ms physics,
10 substeps**, with the original named gains/caps unchanged. The separate
runtime's500Hz ankle controller is not changed. Loading a JSON does not
implement timing or reconcile inherited actuator limits; follow the integration
notes before starting a new experiment. The old200Hz profile remains available
by its explicit filename for historical checkpoints; it is no longer the default.

## Which model should I use?

| Purpose | Entry point |
| --- | --- |
| Inspect the recommended model in MuJoCo | `models/sprite0825_v5_supermimic_run002_20261106/mujoco/scene.xml` |
| Reproduce run002 robot dynamics | `TRAINING_BUNDLE.json` and its exact training MJCF/control profile |
| Develop an external-PD controller | `models/sprite0825_v5_supermimic_run002_20261106/mujoco/scene_external_pd.xml` |
| Import the latest CAD URDF, e.g. into Isaac Lab | `models/sprite0825_v5_supermimic_run002_20261106/sprite0825_float.urdf` |
| Reproduce historical v5, waist mass 2.070928 kg | `models/sprite0825_v5_4340_shoulders/sprite0825_float.urdf` |
| Reproduce the legacy v4 release | `models/sprite0825/sprite0825_sanitized_v4.urdf` |

The external-PD model intentionally contains no MuJoCo actuators (`nu=0`). A
controller must compute torque from target position, measured position and
velocity, then write it through `qfrc_applied` at the physics rate. Opening
that scene in the viewer without a controller is not a walking demo. In
particular, `actuatorfrcrange` is not a clamp on externally written
`qfrc_applied`: external controllers must implement their own torque limits.

Ankles and head pitch/roll are represented as **serial joint coordinates** in
the simulation asset. The physical parallel-linkage transformations belong
to the runtime, not to this model's joint tree. Head yaw is independent.

## Related repositories

| Repository | Responsibility |
| --- | --- |
| [open_sprite](https://github.com/BeijingDynamics/open_sprite) (this repository) | Versioned URDF, meshes, MuJoCo scenes, named training controls and validation |
| [open_sprite_rl](https://github.com/BeijingDynamics/open_sprite_rl) | Training code and explicitly identified policy candidates |
| [open_sprite_runtime](https://github.com/BeijingDynamics/open_sprite_runtime) | Hardware runtime, motor mapping, parallel-joint conversion, IMU and deployment entry points |

Cloning this repository provides the v5 robot assets without a runtime or RL
checkout. It does not include ONNX/PT checkpoints, deployment contracts,
Isaac USD caches, or motor calibration. For a policy, use its own exact
asset revision, joint order, observation/action contract and validation record;
matching the robot name alone is not sufficient.

The CAD URDF cannot express MuJoCo armature and contact exclusions. Importing it
alone is **not** a reproduction of run002. Use the bound MJCF for exact robot
dynamics and the profile for gains, torque saturation, action scaling and timing.
Training-scene domain randomization/rewards/observations remain training-code
responsibilities. The bundle does not include the trainer or policy weights.
Historical assets remain at their old paths and still intentionally contain
their original parameters. See the [migration notes](docs/run002_training_bundle.md).

## Locomotion status

The historical V38 whole-body-tracking milestone is described in
[`docs/locomotion_milestones.md`](docs/locomotion_milestones.md). Research
checkpoints are not described as sim-to-real-ready until they pass both Isaac
Lab and MuJoCo gates and their motion-data redistribution terms are clear.
Current training and deployment work is tracked in the related repositories;
the archived v4 milestone page is not a claim about current training assets.

## Safety

These files and the training control snapshot are research artifacts. They are
not a complete robot controller and do not provide hardware current limits, CAN communication, state estimation,
emergency stopping, or hardware interlocks. Verify joint directions, zero
positions, limits, inertias, coupled-ankle mapping, PD gains, motor ratings and
torque-speed envelopes against the physical robot before applying power.

## License

The repository is licensed under the GNU Affero General Public License v3.0;
see [`LICENSE`](LICENSE). Third-party motion datasets are not redistributed in
this release.
