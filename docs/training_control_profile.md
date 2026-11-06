# Serial training control profiles

## Recommended current bundle: run002,500Hz

The default loader now reads
[`sprite0825_run002_serial500_20261106.json`](../control_profiles/sprite0825_run002_serial500_20261106.json).
Use it together with the exact MJCF in [`TRAINING_BUNDLE.json`](../TRAINING_BUNDLE.json).
This is the verified run002 robot/control pair:500Hz serial PD,50Hz policy,
10substeps, four shoulder armatures0.032kg m^2 and waist mass3.7391kg.
All31named Kp/Kd, effort caps, velocity thresholds and action offsets are
unchanged from the historical table below. The new profile binds the actual
training MJCF, not just CAD URDF metadata.

Read [run002 integration and migration](run002_training_bundle.md) before
training. The loader examples below now select this new bundle by default.
The200Hz snapshot described in the remainder of this document is immutable
historical provenance, NOT the recommended timing for new runs. To load it:

```python
profile = load_control_profile('control_profiles/sprite0825_serial_training_20261105.json',
                               repository_root=ASSET_ROOT)
```

## Historical200Hz snapshot (2026-11-05)

Use [`sprite0825_serial_training_20261105.json`](../control_profiles/sprite0825_serial_training_20261105.json)
to load the31 named joint controls used by the November5 serial SuperMimic
standing experiment before its500Hz timing revision. This is a control-parameter snapshot, **not a
successful walking policy, hardware rating table or hardware deployment
configuration**. It does not change the active training run or runtime.

The snapshot is bound by revision and SHA256 to the public model
`models/sprite0825_v5_4340_shoulders_waist20261105/`, published in commit
`f42a266c860db70d73ce92f34e26d253a2659636`. The loader checks the URDF, both
robot MJCFs, manifest and all manifest-listed assets before returning controls.
Keep the JSON and its `control_profiles/SHA256SUMS` inventory together.

## Load by joint name

From the repository root, check the installed asset/profile pair:

```bash
python scripts/control_profile.py
python scripts/validate_models.py
python -m unittest discover -s scripts -v
```

The loader uses only the Python standard library. It does not import torch,
start threads, load a policy, access CAN/IMU or transmit hardware commands.
MuJoCo is needed only by the separate model validator and its tests.

In another training project:

```python
from pathlib import Path
import sys

ASSET_ROOT = Path("/path/to/open_sprite").resolve()
sys.path.insert(0, str(ASSET_ROOT / "scripts"))
from control_profile import load_control_profile

profile = load_control_profile(repository_root=ASSET_ROOT)
# Supply the simulator's complete31 serial joint names in its actual order.
controls = profile.for_joint_names(simulator_joint_names)
kp = [controls[name].kp for name in simulator_joint_names]
kd = [controls[name].kd for name in simulator_joint_names]
effort = [controls[name].command_effort_nm for name in simulator_joint_names]
default_q = [controls[name].default_pos_rad for name in simulator_joint_names]
velocity_threshold = [controls[name].velocity_limit_rad_s for name in simulator_joint_names]
```

JSON key order is not actuator order. `for_joint_names()` rejects missing,
unknown and repeated names; it does not fall back to generic gains. For
framework names such as `robot/left_knee_joint`, explicitly resolve the entity
prefix in the framework before calling it. Do not pass physical ankle motor
names in place of the serial joint names.

For `supermimic_run`'s existing `JointControl` dataclass, construct its
`JOINT_CONTROL` dictionary from this mapping instead of calling `_gains()`:

```python
named = profile.for_joint_names(BODY_JOINT_NAMES)
JOINT_CONTROL = {
    name: JointControl(
        kp=c.kp, kd=c.kd, command_effort_nm=c.command_effort_nm,
        urdf_velocity_rad_s=c.velocity_limit_rad_s,
        default_pos_rad=c.default_pos_rad,
    )
    for name, c in named.items()
}
```

Its field name `urdf_velocity_rad_s` is historical: the assigned value here
is the **profile's training velocity threshold**, not a value re-read from
URDF. Set the training project's robot/URDF paths to the bound revision too.
Build this dictionary before dependent task/actuator modules import it;
reassigning it after they initialized does not update existing actuators.

## Parameter meanings

The JSON contains every joint's Kp, Kd, applied torque cap, velocity-cost
threshold, action offset, historical command bounds and bound-URDF metadata.
Numbers are preserved without rounding from the source snapshot. The table
below is only a readable summary; the JSON is authoritative.

| Joints | Kp (Nm/rad) | Kd (Nm s/rad) | Applied torque cap (Nm) |
| --- | ---: | ---: | ---: |
| Hip pitch/yaw and knees | 130 | 5 | 40 |
| Hip roll | 150 | 5 | 40 |
| Ankle pitch/roll | 14.2122306824 | 0.9047786593 | 25 |
| Waist yaw | 130 | 5 | 5 |
| Waist roll | 60 | 3 | 97 |
| Shoulder pitch/roll | 130 | 5 | 40 |
| Shoulder yaw and elbows | 7 | 0.45 | 12.5 |
| Wrist yaw | 7 | 1 | 1.5625 |
| Wrist pitch/roll | 7 | 0.4 | 0.03 |
| Head pitch/roll/yaw | 0.9869604111 | 0.0628318548 | 0.15 |

The small wrist/head and waist-yaw caps are historical **actual training
settings**, intentionally exported unchanged. They are not claims about motor
capability or newly recommended limits. In particular wrist pitch/roll0.03Nm
must not be silently replaced with0.5Nm while claiming to reproduce this
snapshot. A changed experiment needs a new profile and validation record.

The source table called the columns `kp_max` and `kd_max`; the active serial
adapter uses those values as fixed gains, not as randomization ranges or
additional upper bounds. Gain and effort randomization are disabled. Other
tracker randomization, including friction, encoder bias, COM and joint
armature/friction, belongs to the training recipe and is not a controller
implemented by this loader.

## Action and torque conventions

All positions are radians and all torques are output **serial joint** Nm.
Before action scaling the actor/collector clips each raw action to[-20,20]:

```text
q_target = default_pos_rad + 0.25 * clipped_action
qd_target = 0
tau_ff = 0
tau_raw = Kp * (q_target - q) + Kd * (qd_target - qd) + tau_ff
tau = clip(tau_raw, -effort_nm, +effort_nm)
```

`default_pos_rad` is the policy action offset, not a change to URDF zero or a
live reset command. Training resets use reference poses/noise; they are not
specified by this offset alone. The profile supplies no policy weights.

`mechanical_position_limit_rad` mirrors the bound URDF joint range. The current
training does not project target positions into an extra runtime envelope.
Physical joint constraints still apply. `archived_command_min_rad` and
`archived_command_max_rad` are provenance only and are **not** target clamps.
`velocity_rad_s` is used by the training velocity cost; it is not a hard
velocity clamp, datasheet no-load speed or MIT encoding VMAX.

The applied caps can differ from URDF `effort` metadata. Do not take the PD
gains from the JSON but silently inherit torque caps from another model or
framework default. External-PD controllers writing `qfrc_applied` must clamp
the computed torque themselves. Frameworks that create actuators must also
reconcile any inherited actuator or joint-force clamps with the profile.
MIT PMAX/VMAX/TMAX and physical motor peak/continuous limits are separate
concepts and are not configuration outputs of this loader.

Ankle pitch/roll PD and caps are in the serial model's joint space. No parallel
Jacobian, motor sign, encoder zero or MIT quantization is introduced into this
training profile. Do not send the serial gains directly to both ankle motors.
Head pitch/roll is also serial in the model; its physical linkage conversion
is a runtime responsibility. Head yaw is independent.

## Timing

| Quantity | This training snapshot |
| --- | --- |
| Policy target updates | 50Hz,20ms |
| Physics integration and PD recomputation | 200Hz,5ms |
| Physics steps per policy action | 4 |
| First-order torque lag | Disabled, tau=0 |
| Command delay buffer | Enabled,0 or1 physics tick,0 or5ms |
| Delay refresh cadence | 4 physics writes,20ms |
| Delay hold probability | 0.9, original tracker/mjlab buffer semantics |

Keep the current q/qd feedback in the200Hz PD loop while holding the policy
target between50Hz updates. The loader provides the delay parameters, not a
replacement stochastic delay implementation. Reuse the matching tracker/mjlab
command-buffer behavior when reproducing the timing.

The public viewer scenes use2ms integration; `supermimic_run` also previously
used2ms/10substeps. Importing the JSON does **not** change those clocks or its
hardcoded stepping loop. For this snapshot, explicitly align robot/task/env
step code, substeps, buffer cadence and configuration to5ms/4substeps. Merely
changing a global `DECIMATION` constant is insufficient if the simulator loop
still advances10substeps. Record an intentional2ms variant as a different
experiment rather than calling it an exact replay.

The verified physical runtime's ankle torque controller runs at500Hz and its
other motor targets at50Hz. Those are deployment rates, not the simulation
PD rate above. No hardware/runtime setting is modified by this publication.

## Model and snapshot scope

The public revision and source training share the owner-provided3.7391kg waist
inertial update, but **the complete models are not identical**. The source
training uses a separate, full-precision inertia model with different shoulder
armature and collision primitives; the public asset preserves the historical
geometry/armature plus the waist update. The profile records both hashes and
does not label this as bit-identical training reproduction. Applying these
controls to the public model is an explicit reusable starting configuration.

The source control JSON, PD table, adapter and source training model hashes are
recorded in `provenance`. A canonical hash of all31 original joint records lets
the loader verify the exported numbers; the published profile has its own
checksum. These checks detect accidental changes and are not a cryptographic
signature. The private training checkout is not required to load the profile.

For a new training run, save the profile file/hash, model file/hash, joint order,
effective compiled actuator parameters, integration rates and framework version
alongside every checkpoint. This profile does not supply reward/reference data,
an observation layout, a SuperMimic architecture or runtime calibration, and
does not make an old checkpoint compatible with a new controller.
