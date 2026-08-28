# Sprite0825 model

This directory is a minimal, self-contained release of the 2026-08-25 Sprite
robot description.

- `sprite0825_sanitized_v4.urdf`: exact URDF variant used by current training.
- `meshes/`: one shared copy of all visual/collision meshes.
- `mujoco/scene.xml`: stable, joint-locked viewer entry point.
- `mujoco/scene_external_pd.xml`: floating-base external-PD development model.

The visual model is intentionally locked. The external-PD model has 32 joints,
37 velocity DoFs including its floating base, and no built-in actuators. It
requires a controller that writes torque through `qfrc_applied`.

The physical ankle uses two coupled DM-J4310P motors, ball-joint rods, and a
universal joint. URDF/MJCF represent pitch and roll as generalized coordinates;
hardware deployment must apply the differential motor/joint mapping.
