# Open Sprite

Open robot-description assets and reproducible simulation entry points for the
Sprite humanoid robot. The current public model is **Sprite0825**, a roughly
0.95 m, 31-actuated-joint humanoid with a V-shaped pelvis.

The default public asset is **v5 with four proximal shoulder DM-J4340P motors
and the updated waist inertial parameters (2026-11-05)**. Its
`waist_yaw_link` mass is **3.7391 kg**, not the historical 2.070928 kg.
Left/right shoulder pitch and roll use J4340P; shoulder yaw is not part of
that upgrade. This is a versioned robot-description release, not a policy bundle.

This repository contains:

- the 2026-08-25 meshes;
- the portable sanitized v5 URDF and its complete referenced mesh set;
- a stable one-command MuJoCo visual model;
- the floating-base external-PD MuJoCo model used for controller development;
- versioned source provenance and SHA-256 checksums;
- the original v4 and historical v5 models, retained unchanged for reproducibility.

The historical v5 assets were extracted from the public G74 `model5999` candidate in
[`open_sprite_rl`](https://github.com/BeijingDynamics/open_sprite_rl/tree/07a513188de64572e891ab21cacb6c79e083b044/candidates/sprite0825_stage2_g74_model5999_4340_shoulders).
That provenance does **not** make this the external G74 `model3000` deployment
package referenced by a runtime launcher. The default revision adds only the
owner-supplied waist mass, COM and inertia; it does not relabel old policies
as qualified for the heavier model. See
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
python -m mujoco.viewer --mjcf=models/sprite0825_v5_4340_shoulders_waist20261105/mujoco/scene.xml
```

### Windows PowerShell

```powershell
git clone https://github.com/BeijingDynamics/open_sprite.git
cd open_sprite
py -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python -m mujoco.viewer --mjcf=models/sprite0825_v5_4340_shoulders_waist20261105/mujoco/scene.xml
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

The validator checks the latest revision, historical v5 and legacy v4 scenes,
URDF mesh paths and asset hashes. It also checks the 31 hinges, four upgraded
shoulders, URDF/MJCF axes and position limits, and the latest waist mass,
COM and full inertia tensor in both the URDF and compiled MuJoCo models.
No robot connection is needed.

## Which model should I use?

| Purpose | Entry point |
| --- | --- |
| Inspect the latest model in MuJoCo | `models/sprite0825_v5_4340_shoulders_waist20261105/mujoco/scene.xml` |
| Develop an external-PD controller with the latest model | `models/sprite0825_v5_4340_shoulders_waist20261105/mujoco/scene_external_pd.xml` |
| Import the latest URDF, e.g. into Isaac Lab | `models/sprite0825_v5_4340_shoulders_waist20261105/sprite0825_float.urdf` |
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
| [open_sprite](https://github.com/BeijingDynamics/open_sprite) (this repository) | Versioned URDF, meshes, MuJoCo scenes and validation |
| [open_sprite_rl](https://github.com/BeijingDynamics/open_sprite_rl) | Training code and explicitly identified policy candidates |
| [open_sprite_runtime](https://github.com/BeijingDynamics/open_sprite_runtime) | Hardware runtime, motor mapping, parallel-joint conversion, IMU and deployment entry points |

Cloning this repository provides the v5 robot assets without a runtime or RL
checkout. It does not include ONNX/PT checkpoints, deployment contracts,
Isaac USD caches, or motor calibration. For a policy, use its own exact
asset revision, joint order, observation/action contract and validation record;
matching the robot name alone is not sufficient.

The waist update does not change joint axes, mechanical limits, shoulder
armature, collision geometry, meshes or PD gains. It matches the owner-supplied
waist inertial block, not every setting in a training experiment's model adapter.
Historical assets remain at their old paths and still intentionally contain
the old waist mass. Do not use those paths when requesting the updated model.

## Locomotion status

The historical V38 whole-body-tracking milestone is described in
[`docs/locomotion_milestones.md`](docs/locomotion_milestones.md). Research
checkpoints are not described as sim-to-real-ready until they pass both Isaac
Lab and MuJoCo gates and their motion-data redistribution terms are clear.
Current training and deployment work is tracked in the related repositories;
the archived v4 milestone page is not a claim about current training assets.

## Safety

These files are research artifacts. They are not a complete robot controller
and do not provide current limits, CAN communication, state estimation,
emergency stopping, or hardware interlocks. Verify joint directions, zero
positions, limits, inertias, coupled-ankle mapping, PD gains, motor ratings and
torque-speed envelopes against the physical robot before applying power.

## License

The repository is licensed under the GNU Affero General Public License v3.0;
see [`LICENSE`](LICENSE). Third-party motion datasets are not redistributed in
this release.
