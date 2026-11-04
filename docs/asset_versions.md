# Robot asset versions and policy provenance

## v5 four-shoulder release (2026-11-04)

The default asset in this repository is
`models/sprite0825_v5_4340_shoulders/`. It contains the portable URDF, all 33
STL files from the source mesh directory, a floating-base external-PD MJCF,
and a joint-locked geometry inspection scene. The legacy v4 directory
`models/sprite0825/` remains unchanged.

The four upgraded joints are:

- `left_shoulder_pitch_joint`
- `left_shoulder_roll_joint`
- `right_shoulder_pitch_joint`
- `right_shoulder_roll_joint`

The source manifest identifies these motors as DM-J4340P-2EC, with rated
torque 14 Nm, peak torque 40 Nm, and a selected velocity limit of 9.3 rad/s.
The frozen URDF's four shoulder limits have `effort=14` and `velocity=9.3`.
These fields are preserved, not silently changed to current protocol ranges
or hardware peaks. Other joints retain the source asset's values too.
URDF effort, MIT protocol encoding range, physical peak torque, thermal
continuous rating and deployment command cap are different quantities.
This asset does not define a safe hardware torque envelope.

Shoulder yaw and the distal arm are not upgraded by the four-shoulder change.
The source mesh geometry and inertials are preserved; this release does not
claim that all CAD meshes or masses were redesigned for the motor swap, nor
does it incorporate later payload/inertia changes from newer experiments.

## Exact public source

Source repository: [BeijingDynamics/open_sprite_rl](https://github.com/BeijingDynamics/open_sprite_rl).
Pinned commit: `07a513188de64572e891ab21cacb6c79e083b044`.
Source candidate:
[`candidates/sprite0825_stage2_g74_model5999_4340_shoulders`](https://github.com/BeijingDynamics/open_sprite_rl/tree/07a513188de64572e891ab21cacb6c79e083b044/candidates/sprite0825_stage2_g74_model5999_4340_shoulders).

The URDF is byte-for-byte identical to
`assets/isaac/sprite0825_sanitized_v5_4340_shoulders/sprite0825_float.urdf`
in that candidate, with SHA-256:

```text
6c753d563b54103e4278c27a11a22e0f4f1076e4156f0873cb2325f1701a921b
```

The source Isaac and MuJoCo mesh copies were checked for byte equality. All
copied source blobs were verified against the pinned public Git tree.
[`manifest.json`](../models/sprite0825_v5_4340_shoulders/manifest.json) records
their Git blob IDs and SHA-256 values. [`ASSET_SHA256SUMS`](../ASSET_SHA256SUMS)
records the packaged files. No private worktree version substitutes for the
public source.

Packaging changes are limited to:

1. Sharing one mesh directory between URDF and MJCF with `meshdir="../"`.
2. Correcting an inherited v4 label on the external scene and adding camera framing.
3. Deriving the inspection model by removing the free joint and 31 hinges
   at zero coordinates. Its fixed appearance is not a standing controller.
4. XML formatting in the packaged MJCF files. Joint axes, limits, body
   transforms, inertias, collision shapes and contact settings are preserved.

The source `sprite0825.usd` and configuration layers remain in the linked RL
candidate. They are not required for this repository's MuJoCo entry point;
Isaac Lab users can import the portable URDF. Policies, deploy files and
runtime-specific configuration are deliberately not copied into this asset
release.

For MuJoCo, use the supplied MJCF scenes: importing a URDF directly uses
MuJoCo's importer defaults (including a fixed base), not the supplied
floating-base scene. The unmodified source URDF also repeats the sole-proxy
collision name on both feet; MuJoCo may warn and rename one on direct import.
The supplied MJCF scenes load without that warning.

## Validation of this publication

The packaged files were loaded successfully on Windows and on Linux (234),
without a policy or hardware connection. Both v5 scenes have 33 MuJoCo bodies,
35 geoms and 32 referenced meshes. The inspection scene has `nv=0`; the
floating scene has `njnt=32`, `nv=37` (31 hinges plus a free base) and `nu=0`.
The mesh directory also retains the unreferenced source `base_link.STL`.

`python scripts/validate_models.py` checks all 78 packaged asset files,
including legacy v4, against the checksum inventory. It also checks v5 joint
axes, position/effort metadata, shoulder velocity limits and positive mass /
positive-definite inertia tensors. Six focused regression tests can be run
with `python -m unittest discover -s scripts -v`. An offscreen v5 rendering
was inspected as well. These are asset/load checks, not policy or hardware
qualification.

## Do not confuse the three artifacts

| Artifact | Where to find it | What it proves |
| --- | --- | --- |
| v5 robot description | This repository's versioned v5 directory | Geometry, joint tree and frozen simulation asset parameters |
| G74 `model5999` candidate | Pinned RL source above | Provenance of the robot asset; its own policy and contract are separate |
| G74 `model3000` sim-to-real candidate | External deployment package referenced by the runtime admission launcher | A separately selected policy/deployment bundle; not supplied by this repository |

The runtime's
[`run_sprite0825_native_protected_policy_admission_on_253.sh`](https://github.com/BeijingDynamics/open_sprite_runtime/blob/main/run_sprite0825_native_protected_policy_admission_on_253.sh)
references an external `sprite0825_stage2_g74_model3000_sim2real_candidate`
directory. A hardware adaptation for v5 does not mean that a clone contains
that whole external directory. The older G60 `model3450` artifact shipped in
the runtime is not the same policy either. Check the specific launcher and
artifact hashes, rather than assuming every runtime entry point uses the
same bundle.

**This publication neither supplies nor certifies model3000, and does not
rename model5999 as model3000.** For deployment, obtain the matching policy,
contract, robot model and calibration as an explicitly validated set from the
runtime/RL project. No motor directions, zeros, bus IDs, gains or parallel
linkage calibration are inferred by opening this viewer.
