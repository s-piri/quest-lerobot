"""Build the headset's arm model: slimmed meshes and the joint tree.

A development tool, not part of the package. It needs `open3d` and `numpy`, and
the upstream mesh folder at the commit recorded in models/PROVENANCE.md:

    python -I scripts/build_client_assets.py <SO-ARM100>/Simulation/SO101/assets

The upstream meshes total about 19 MB, far more triangles than a headset
should draw for two arms shown twice each. Each is simplified here, so the
files under client/assets are MODIFIED copies of the upstream Apache-2.0 work.
"""

import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from lerobot_teleoperator_quest.kinematics import (  # noqa: E402
    SO101_MODEL,
    SO101_SHA256,
    so101_scene,
)

TARGET = 2500  # triangles for a printed part
MOTOR = 1200  # the servo body appears six times
OUT = ROOT / "client" / "assets"


def write_binary_stl(path: Path, vertices: np.ndarray, triangles: np.ndarray):
    record = np.dtype([("normal", "<f4", 3), ("corners", "<f4", (3, 3)), ("attribute", "<u2")])
    data = np.zeros(len(triangles), dtype=record)
    corners = vertices[triangles].astype("<f4")
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    data["normal"] = normals / np.where(lengths > 0, lengths, 1)
    data["corners"] = corners
    header = b"quest-lerobot simplified SO-101 mesh".ljust(80, b" ")
    path.write_bytes(header + np.uint32(len(triangles)).tobytes() + data.tobytes())


def main():
    source = Path(sys.argv[1]).resolve(strict=True)
    tree = so101_scene()
    OUT.mkdir(parents=True, exist_ok=True)
    counts = {}
    for name in sorted({visual["mesh"] for visual in tree["visuals"]}):
        mesh = o3d.io.read_triangle_mesh(str(source / name))
        mesh.remove_duplicated_vertices()
        mesh.remove_degenerate_triangles()
        before = len(mesh.triangles)
        target = MOTOR if name.startswith("sts3215") else TARGET
        if before > target:
            mesh = mesh.simplify_quadric_decimation(target)
            mesh.remove_degenerate_triangles()
        vertices, triangles = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        if not len(triangles) or not np.isfinite(vertices).all():
            raise SystemExit(f"{name} did not survive simplification")
        write_binary_stl(OUT / name, vertices, triangles)
        counts[name] = len(triangles)
        print(f"{name:42} {before:7d} -> {len(triangles):5d} triangles")
    manifest = {
        "source": {
            "repository": "https://github.com/TheRobotStudio/SO-ARM100",
            "model": f"Simulation/SO101/{SO101_MODEL}",
            "model_sha256": SO101_SHA256,
            "meshes": "simplified from Simulation/SO101/assets; see README.md beside this file",
        },
        "meshes": counts,
        **tree,
    }
    (OUT / "so101.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
    drawn = sum(counts[visual["mesh"]] for visual in tree["visuals"])
    print(f"{len(counts)} meshes, {drawn} triangles drawn per arm")


if __name__ == "__main__":
    main()
