# Sprite Humanoid

Open robot-description assets and reproducible simulation entry points for the
Sprite humanoid robot. The current public model is **Sprite0825**, a roughly
0.95 m, 31-actuated-joint humanoid with a V-shaped pelvis.

This repository currently contains:

- the 2026-08-25 meshes;
- the sanitized v4 URDF used by current Isaac Lab training;
- a stable one-command MuJoCo visual model;
- the floating-base external-PD MuJoCo model used for controller development;
- documented locomotion milestones and artifact hashes.

## Quick start: view Sprite0825 in MuJoCo

Python 3.10 or newer is recommended.

### Linux/macOS

```bash
git clone https://github.com/BeijingDynamics/sprite_humanoid.git
cd sprite_humanoid
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m mujoco.viewer --mjcf=models/sprite0825/mujoco/scene.xml
```

### Windows PowerShell

```powershell
git clone https://github.com/BeijingDynamics/sprite_humanoid.git
cd sprite_humanoid
py -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python -m mujoco.viewer --mjcf=models/sprite0825/mujoco/scene.xml
```

The default scene is intentionally joint-locked so the first viewer command
shows the designed standing geometry instead of an uncontrolled floating-base
robot falling under gravity.

## Validate the models

```bash
python scripts/validate_models.py
```

The validator loads both the visual scene and the floating-base external-PD
scene, then prints body, joint, DoF, geometry, mesh, actuator, and timestep
counts. A successful load also verifies that all relative mesh paths resolve.

## Which model should I use?

| Purpose | Entry point |
| --- | --- |
| Inspect the robot in MuJoCo | `models/sprite0825/mujoco/scene.xml` |
| Develop an external-PD controller | `models/sprite0825/mujoco/scene_external_pd.xml` |
| Reproduce the current training URDF | `models/sprite0825/sprite0825_sanitized_v4.urdf` |

The external-PD model intentionally contains no MuJoCo actuators (`nu=0`). A
controller must compute torque from target position, measured position and
velocity, then write it through `qfrc_applied` at the physics rate. Opening
that scene in the viewer without a controller is not a walking demo.

## Locomotion status

The frozen V38 whole-body-tracking milestone and the active Stage 2
command-locomotion effort are described in
[`docs/locomotion_milestones.md`](docs/locomotion_milestones.md). Research
checkpoints are not described as sim-to-real-ready until they pass both Isaac
Lab and MuJoCo gates and their motion-data redistribution terms are clear.

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
