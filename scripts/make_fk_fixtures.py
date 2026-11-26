"""Regenerate tests/fixtures/so101_fk.json with an independent URDF library.

Run it in a throwaway environment with `yourdfpy` and `numpy` installed, not in
the project's own one: the point is that this project's kinematics code is not
involved in producing the reference poses.

    python scripts/make_fk_fixtures.py
"""

import json
import random
from pathlib import Path

import numpy as np
import yourdfpy

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src" / "lerobot_teleoperator_quest" / "models" / "so101_new_calib.urdf"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def quaternion(matrix) -> list[float]:
    """xyzw from a rotation matrix, w non-negative."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2 * np.sqrt(1 + m[0][0] - m[1][1] - m[2][2])
        q = [s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    elif m[1][1] > m[2][2]:
        s = 2 * np.sqrt(1 + m[1][1] - m[0][0] - m[2][2])
        q = [(m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    else:
        s = 2 * np.sqrt(1 + m[2][2] - m[0][0] - m[1][1])
        q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s]
    q = [float(c) for c in q]
    return q if q[3] >= 0 else [-c for c in q]


def main():
    robot = yourdfpy.URDF.load(str(MODEL), load_meshes=False, build_scene_graph=True)
    limits = {j: (robot.joint_map[j].limit.lower, robot.joint_map[j].limit.upper) for j in JOINTS}
    rng = random.Random(101)
    cases = [[0.0] * 5]
    cases += [[limits[j][0] for j in JOINTS], [limits[j][1] for j in JOINTS]]
    for i in range(5):
        one = [0.0] * 5
        one[i] = 0.7
        cases.append(one)
    cases += [[rng.uniform(*limits[j]) for j in JOINTS] for _ in range(40)]

    records = []
    for joints in cases:
        robot.update_cfg(dict(zip(JOINTS, joints, strict=True)))
        transform = robot.get_transform("gripper_frame_link", "base_link")
        records.append(
            {
                "joints": joints,
                "position": [float(v) for v in transform[:3, 3]],
                "orientation": quaternion(transform[:3, :3]),
            }
        )
    # Every link, with the jaw joint moved too, for the renderer's forward kinematics.
    links = []
    for joints in cases[:16]:
        values = dict(zip(JOINTS, joints, strict=True))
        jaw = robot.joint_map["gripper"].limit
        values["gripper"] = rng.uniform(jaw.lower, jaw.upper)
        robot.update_cfg(values)
        poses = {}
        for name in robot.link_map:
            transform = robot.get_transform(name, "base_link")
            poses[name] = {
                "position": [float(v) for v in transform[:3, 3]],
                "orientation": quaternion(transform[:3, :3]),
            }
        links.append({"values": values, "links": poses})

    target = ROOT / "tests" / "fixtures" / "so101_fk.json"
    target.parent.mkdir(exist_ok=True)
    payload = {"generator": f"yourdfpy {yourdfpy.__version__}", "cases": records, "links": links}
    target.write_text(json.dumps(payload, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {len(records)} poses to {target}")


if __name__ == "__main__":
    main()
