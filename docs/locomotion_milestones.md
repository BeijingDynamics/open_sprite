# Locomotion milestones

This page preserves the original v4-era milestone record. For the default
public v5 asset and the distinction between robot assets and policy packages,
see [asset_versions.md](asset_versions.md). Current training/runtime progress
is maintained in the linked RL and runtime repositories.

## V38 Stage 1 whole-body tracking

V38 is the first frozen Sprite whole-body-tracking milestone that completed
the internal Isaac Lab and MuJoCo sim-to-sim qualification matrix.

- checkpoint SHA-256:
  `b8d1851de07e654c367d0ad9d6d2e32a9fbc9588e4f26aabd7e5e4d0d71166ff`
- actor observation: 164 values
- action: 31 joint-position residuals
- policy rate: 50 Hz
- actor horizontal base velocity: **not used**
- actor global anchor position: **not used**
- Isaac Lab: five seeds at 0.8x, 1.0x, and 1.15x, clean and pushed
- MuJoCo: three speeds, four start phases, and a 60 s endurance test

The V38 binary package is not included in this initial public release. Its
reference trajectory is derived from AMASS/BMLrub motion data, and the project
is auditing redistribution terms before publishing the trajectory-dependent
deploy package. The hash and qualification record are published here so later
releases can be matched to the frozen artifact without ambiguity.

## Stage 2 command locomotion

At the time of the original v4 release, Stage 2 was in development on the
Sprite0825 sanitized v4 asset. Its target was one command-conditioned policy supporting stable stand, natural
start/stop, continuous forward speed, and left/right turning in both Isaac Lab
and MuJoCo. Experimental checkpoints are deliberately not published as
hardware-ready policies.

No policy in this repository should be deployed to a physical robot without a
separate motor-interface, estimator, limits, emergency-stop path, and hardware
validation process.
