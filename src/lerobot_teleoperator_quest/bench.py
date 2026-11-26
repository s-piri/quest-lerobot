"""What the bench establishes about one arm, kept in one checked file.

A setup holds the joint mapping, and optionally the limits, the headset-to-base
rotation and a camera fixed to the arm. Everything in it is bound to one
official calibration file by fingerprint. Standard library only.

Fitting a mapping from measurements does not verify it: only moving the real
arm slowly under supervision does. See docs/bench.md.
"""

import argparse
import json
import sys
from dataclasses import dataclass, fields
from math import degrees, radians
from pathlib import Path

from .contracts import finite, vector
from .control import Limits
from .pose import Pose, unit_quaternion
from .so101 import FOLLOWER_KEYS, UrdfMapping, calibration_fingerprint

VERSION = 1


@dataclass(frozen=True)
class Camera:
    """A camera fixed to one link, in OpenCV conventions.

    `pose` is the camera frame in that link's frame: x right, y down, z out
    through the lens. Sizes are pixels; `distortion` is k1, k2, p1, p2, k3.
    """

    link: str
    pose: Pose
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self):
        if not isinstance(self.link, str) or not self.link:
            raise ValueError("camera link must be a link name")
        if not isinstance(self.pose, Pose):
            raise ValueError("camera pose must be a pose")
        for name in ("width", "height"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"camera {name} must be a positive whole number of pixels")
        for name in ("fx", "fy", "cx", "cy"):
            object.__setattr__(self, name, finite(getattr(self, name), f"camera {name}"))
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("camera focal lengths must be positive")
        object.__setattr__(self, "distortion", vector(self.distortion, 5, "camera distortion"))

    def to_dict(self) -> dict:
        return {
            "link": self.link,
            "position": list(self.pose.position),
            "orientation": list(self.pose.orientation),
            "width": self.width,
            "height": self.height,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "distortion": list(self.distortion),
        }

    @classmethod
    def from_dict(cls, data):
        names = {"link", "position", "orientation", "width", "height", "fx", "fy", "cx", "cy"}
        if not isinstance(data, dict) or not names <= set(data) <= names | {"distortion"}:
            raise ValueError("camera must contain exactly the documented fields")
        return cls(
            data["link"],
            Pose(data["position"], data["orientation"]),
            data["width"],
            data["height"],
            data["fx"],
            data["fy"],
            data["cx"],
            data["cy"],
            data.get("distortion", (0.0,) * 5),
        )


@dataclass(frozen=True)
class Setup:
    """One arm's bench results. `calibration` is the mapping's fingerprint."""

    mapping: UrdfMapping
    limits: Limits | None = None
    xr_to_base: tuple[float, ...] | None = None
    camera: Camera | None = None

    def __post_init__(self):
        if not isinstance(self.mapping, UrdfMapping):
            raise ValueError("a setup needs a URDF mapping")
        if self.limits is not None and not isinstance(self.limits, Limits):
            raise ValueError("limits must be limits or None")
        if self.camera is not None and not isinstance(self.camera, Camera):
            raise ValueError("camera must be a camera or None")
        if self.xr_to_base is not None:
            rotation = unit_quaternion(self.xr_to_base, "xr_to_base")
            object.__setattr__(self, "xr_to_base", rotation)

    @property
    def calibration(self) -> str:
        return self.mapping.calibration

    def to_dict(self) -> dict:
        limits = self.limits
        return {
            "version": VERSION,
            "calibration": self.mapping.calibration,
            "mapping": {
                "signs": list(self.mapping.signs),
                "zero_degrees": list(self.mapping.zero_degrees),
            },
            "limits": None
            if limits is None
            else {
                f.name: (
                    list(getattr(limits, f.name))
                    if isinstance(getattr(limits, f.name), tuple)
                    else getattr(limits, f.name)
                )
                for f in fields(Limits)
            },
            "xr_to_base": None if self.xr_to_base is None else list(self.xr_to_base),
            "camera": None if self.camera is None else self.camera.to_dict(),
        }

    @classmethod
    def from_dict(cls, data):
        names = {"version", "calibration", "mapping", "limits", "xr_to_base", "camera"}
        if not isinstance(data, dict) or set(data) != names:
            raise ValueError("setup must contain exactly the documented fields")
        if type(data["version"]) is not int or data["version"] != VERSION:
            raise ValueError(f"setup version must be {VERSION}")
        mapping = data["mapping"]
        if not isinstance(mapping, dict) or set(mapping) != {"signs", "zero_degrees"}:
            raise ValueError("mapping must contain signs and zero_degrees")
        limits = data["limits"]
        if limits is not None:
            wanted = {f.name for f in fields(Limits)}
            if not isinstance(limits, dict) or set(limits) != wanted:
                raise ValueError("limits must contain exactly the documented fields")
            limits = Limits(**limits)
        camera = data["camera"]
        return cls(
            UrdfMapping(mapping["signs"], mapping["zero_degrees"], data["calibration"]),
            limits,
            data["xr_to_base"],
            None if camera is None else Camera.from_dict(camera),
        )

    def save(self, path) -> None:
        text = json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n"
        Path(path).write_text(text, encoding="utf-8")

    @classmethod
    def load(cls, path):
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path} is not a setup file: {exc}") from exc
        return cls.from_dict(data)


def fit_mapping(samples, calibration: str) -> tuple[UrdfMapping, tuple[float, ...]]:
    """Fit each joint's direction and zero from poses measured at the bench.

    `samples` pairs the model's five joint angles in radians, as posed, with
    the follower's five readings in degrees. The model is
    `follower = zero + sign * degrees(joint)`. Returns the mapping and each
    joint's worst disagreement in degrees, which the caller must judge: a
    large one means a pose was matched poorly or a sample is wrong.

    A joint's direction is decided only if it was posed across at least 20
    degrees and one direction fits clearly better than the other.
    """
    pairs = []
    try:
        for model, follower in samples:
            pairs.append(
                (vector(model, 5, "model joints"), vector(follower, 5, "follower degrees"))
            )
    except TypeError as exc:
        raise ValueError("samples must pair model joints with follower degrees") from exc
    if len(pairs) < 2:
        raise ValueError("fitting needs at least two poses")
    signs, zeros, worst = [], [], []
    for joint in range(5):
        posed = [degrees(model[joint]) for model, _ in pairs]
        read = [follower[joint] for _, follower in pairs]
        if max(posed) - min(posed) < 20:
            raise ValueError(f"joint {joint + 1} was not posed across at least 20 degrees")
        fits = {}
        for sign in (1.0, -1.0):
            offsets = [r - sign * p for r, p in zip(read, posed, strict=True)]
            zero = sum(offsets) / len(offsets)
            fits[sign] = (max(abs(o - zero) for o in offsets), zero)
        sign = min(fits, key=lambda s: fits[s][0])
        if fits[-sign][0] < 3 * fits[sign][0] + 5:
            raise ValueError(f"joint {joint + 1}: the readings do not settle its direction")
        signs.append(sign)
        zeros.append(fits[sign][1])
        worst.append(fits[sign][0])
    return UrdfMapping(signs, zeros, calibration), tuple(worst)


# Model poses to copy at the bench: zero, then each joint alone by 45 degrees.
REFERENCE_POSES = tuple(
    tuple(radians(45) if joint == moved else 0.0 for joint in range(5))
    for moved in (None, 0, 1, 2, 3, 4)
)


def _fingerprint(path: Path) -> str:
    return calibration_fingerprint(json.loads(path.read_text(encoding="utf-8")))


def _fit(args) -> None:
    if args.out.exists():
        raise ValueError(f"{args.out} already exists; choose another name or move it away")
    records = json.loads(args.samples.read_text(encoding="utf-8"))
    if not isinstance(records, list):
        raise ValueError("the samples file must be a JSON list")
    samples = []
    for record in records:
        if not isinstance(record, dict) or set(record) != {"model", "follower"}:
            raise ValueError('each sample needs exactly "model" and "follower"')
        follower = record["follower"]
        if isinstance(follower, dict):
            missing = [key for key in FOLLOWER_KEYS[:5] if key not in follower]
            if missing:
                raise ValueError(f"a sample's follower readings lack {', '.join(missing)}")
            follower = [follower[key] for key in FOLLOWER_KEYS[:5]]
        samples.append((record["model"], follower))
    mapping, worst = fit_mapping(samples, _fingerprint(args.calibration))
    Setup(mapping).save(args.out)
    print(f"Wrote {args.out} from {len(samples)} poses.")
    rows = zip(FOLLOWER_KEYS[:5], mapping.signs, mapping.zero_degrees, worst, strict=True)
    for name, sign, zero, off in rows:
        flag = "  <- check how this joint was posed" if off > 5 else ""
        print(
            f"  {name:18} direction {sign:+.0f}  zero {zero:8.2f} deg  worst {off:5.2f} deg{flag}"
        )
    print("This is a fit, not a verification. Move the arm slowly under supervision first.")


def _show(args) -> None:
    setup = Setup.load(args.setup)
    print(f"{args.setup}: calibration {setup.calibration[:12]}...")
    print(f"  directions {setup.mapping.signs}")
    print(f"  zeros      {tuple(round(z, 2) for z in setup.mapping.zero_degrees)} deg")
    for name in ("limits", "xr_to_base", "camera"):
        print(f"  {name:10} {'set' if getattr(setup, name) is not None else 'not set'}")
    if args.calibration is None:
        return
    if _fingerprint(args.calibration) != setup.calibration:
        raise ValueError(
            "this setup was made for a different calibration file. Either the arm has been "
            "recalibrated or this is another arm's setup: measure the mapping again"
        )
    print("  It matches that calibration file.")


def main(argv=None) -> int:
    """`python -m lerobot_teleoperator_quest.bench`: fit and inspect setup files."""
    parser = argparse.ArgumentParser(
        prog="python -m lerobot_teleoperator_quest.bench",
        description="Fit a joint mapping from bench samples, or inspect a setup file.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    fit = commands.add_parser("fit", help="fit a joint mapping from bench samples")
    fit.add_argument("--samples", required=True, type=Path, help="JSON list of poses and readings")
    fit.add_argument(
        "--calibration", required=True, type=Path, help="the official calibration JSON"
    )
    fit.add_argument(
        "--out", required=True, type=Path, help="setup file to write; never overwritten"
    )
    show = commands.add_parser("show", help="print a setup file and check its calibration")
    show.add_argument("setup", type=Path)
    show.add_argument("--calibration", type=Path, help="the official calibration JSON")
    args = parser.parse_args(argv)
    try:
        (_fit if args.command == "fit" else _show)(args)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
