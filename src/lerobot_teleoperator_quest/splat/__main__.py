"""`python -m lerobot_teleoperator_quest.splat`: prepare a recording for reconstruction."""

import argparse
import json
import sys
from math import radians
from pathlib import Path

from ..bench import Setup
from ..so101 import calibration_fingerprint
from .poses import export


def _load(args) -> tuple[Setup, str]:
    setup = Setup.load(args.setup)
    calibration = calibration_fingerprint(json.loads(args.calibration.read_text(encoding="utf-8")))
    if calibration != setup.calibration:
        raise ValueError(
            "this setup was made for a different calibration file. Either the arm has been "
            "recalibrated or this is another arm's setup"
        )
    return setup, calibration


def _export(args) -> None:
    setup, calibration = _load(args)
    report = export(
        args.recording,
        setup,
        calibration,
        args.out,
        camera_delay=args.camera_delay,
        min_move=args.min_move,
        min_turn=radians(args.min_turn),
        max_speed=None if args.any_speed else args.max_speed,
    )
    print(f"Wrote {args.out}: kept {report['kept']} of {report['frames']} frames.")
    for why, count in report["dropped"].items():
        if count:
            print(f"  left out {count}: {why}")
    if report["kept"] < 40:
        print(
            "  Few views were kept. A splat usually needs many more, from many directions: "
            "scan longer and move the camera around the scene."
        )
    if args.meshes is not None:
        _masks(args, setup)
    else:
        print("  No masks were made, so the gripper will be reconstructed as part of the scene.")
        print("  Add --meshes with the folder of arm meshes (client/assets) to hide it.")
    print("  Poses are in the arm's base frame, in metres. They are a starting guess from the")
    print("  joints: let reconstruction refine them, and tell the trainer not to re-orient,")
    print("  re-centre or re-scale them.")


def _masks(args, setup: Setup) -> None:
    try:
        from .masks import load_meshes, write_masks
    except ImportError as error:
        raise ValueError(
            f"masks need numpy and OpenCV, which are not installed ({error.name}). "
            'Install the "splat" extra, or export without --meshes'
        ) from error
    result = write_masks(
        args.out, setup.camera, load_meshes(args.meshes, setup.camera), margin=args.mask_margin
    )
    least, most = round(result["least_hidden"] * 100), round(result["most_hidden"] * 100)
    span = f"{least}%" if least == most else f"{least}% to {most}%"
    print(f"  Masks hide the gripper: {span} of each picture.")
    if result["check"]:
        print(f"  Open {args.out / result['check']} to check them:")
        print("  red must cover the whole gripper and little else.")
    if most == 0:
        print("  <- nothing was hidden: the camera's mount is probably wrong. Recalibrate it.")
    elif most > 60:
        print(
            "  <- most of the picture is hidden: check the camera's mount, or lower --mask-margin."
        )


def _calibrate(args) -> None:
    # Imported here so that exporting never needs numpy or OpenCV.
    try:
        from .calibrate import Board, calibrate
    except ImportError as error:
        raise ValueError(
            f"calibrating needs numpy and OpenCV, which are not installed ({error.name}). "
            'Install the "splat" extra'
        ) from error
    if args.out.exists():
        raise ValueError(f"{args.out} already exists; choose another name or move it away")
    setup, calibration = _load(args)
    try:
        columns, rows = (int(part) for part in args.board.lower().split("x"))
    except ValueError as error:
        raise ValueError("--board is the inner corners across and down, like 9x6") from error
    board = Board(columns, rows, args.square)
    camera, report = calibrate(args.recording, setup, calibration, board, link=args.link)
    Setup(setup.mapping, setup.limits, setup.xr_to_base, camera).save(args.out)
    print(f"Wrote {args.out} with the camera on {camera.link}.")
    print(f"  used {report['used']} of {report['shots']} shots", end="")
    missed = report["board_not_found"] + report["unreadable"]
    print(f"; the whole board was not found in {missed}." if missed else ".")
    error_px, spread_mm = report["lens_error_pixels"], report["board_spread_metres"] * 1000
    print(
        f"  lens: {error_px:.2f} pixels of error"
        + ("  <- high: retake blurred shots" if error_px > 1 else "")
    )
    print(
        f"  mount: the views place the board within {spread_mm:.1f} mm of each other"
        + (
            "  <- high: check the joint mapping and that the board did not move"
            if spread_mm > 8
            else ""
        )
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m lerobot_teleoperator_quest.splat",
        description="Calibrate the wrist camera, and turn a recording into camera poses.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    def common(command):
        command.add_argument("recording", type=Path, help="folder with shots.jsonl and its images")
        command.add_argument("--setup", required=True, type=Path, help="the arm's setup file")
        command.add_argument(
            "--calibration", required=True, type=Path, help="the official calibration JSON"
        )

    command = commands.add_parser(
        "calibrate", help="find the camera's lens and mount from board shots"
    )
    common(command)
    command.add_argument("--board", default="9x6", help="inner corners across and down (9x6)")
    command.add_argument(
        "--square", required=True, type=float, help="side of one square, in metres"
    )
    command.add_argument("--link", default="gripper_link", help="the link the camera is fixed to")
    command.add_argument(
        "--out", required=True, type=Path, help="new setup file; never overwritten"
    )

    command = commands.add_parser("export", help="write a scene folder from a recording")
    common(command)
    command.add_argument("--out", required=True, type=Path, help="new scene folder to write")
    command.add_argument(
        "--camera-delay",
        type=float,
        default=0.0,
        help="seconds by which each image is older than the reading stored beside it (default 0)",
    )
    command.add_argument(
        "--min-move", type=float, default=0.01, help="metres between kept views (0.01)"
    )
    command.add_argument(
        "--min-turn", type=float, default=3.0, help="degrees between kept views (3)"
    )
    command.add_argument(
        "--max-speed",
        type=float,
        default=0.15,
        help="drop views taken while the camera moves faster than this, in metres a second (0.15)",
    )
    command.add_argument(
        "--any-speed", action="store_true", help="keep views whatever the camera speed"
    )
    command.add_argument("--meshes", type=Path, help="folder of arm meshes, to mask the gripper")
    command.add_argument(
        "--mask-margin", type=int, default=12, help="pixels to grow each mask by (12)"
    )
    args = parser.parse_args(argv)
    try:
        (_calibrate if args.command == "calibrate" else _export)(args)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
