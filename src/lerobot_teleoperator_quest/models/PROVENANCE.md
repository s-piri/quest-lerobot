# Packaged robot model

`so101_new_calib.urdf` is copied byte for byte from
<https://github.com/TheRobotStudio/SO-ARM100>, path
`Simulation/SO101/so101_new_calib.urdf`, at commit
`141aea7bcac2744d92998e30268c413202a56f96` (2026-11-21).

- SHA-256: `651ea00a3efe02a3c984b21a1b52273bd1b320ed635b3eae92fd52e99d0a393b`
- Licence: Apache-2.0, reproduced beside this file as `SO-ARM100-LICENSE`.
- The upstream file was generated from the Onshape CAD model with
  onshape-to-robot. It is not modified here.

Only the joint chain is used. The file refers to mesh files under `assets/`,
which are not packaged. "New calibration" means each joint's zero is the middle
of its range, which is the convention of LeRobot's current calibration. That
does not make follower values equal to these joint values: a mapping still has
to be measured on the arm.

`kinematics.so101()` refuses to load a file whose checksum differs. To update
the model, replace the file, the commit above and `SO101_SHA256` together, then
regenerate `tests/fixtures/so101_fk.json` with `scripts/make_fk_fixtures.py`.
