#!/usr/bin/env python3
"""Step 04 - rotate every aligned G-code file around one shared XY part center.

All translated files from step 03, across every sample, are rotated together.
One center is calculated from their combined XY bounding box and then reused
for each file, so the CCF and PETG paths of a sample stay aligned with each
other and the samples stay aligned with one another.

Positive angles rotate counterclockwise when viewed from above (+Z). Use
``--angle -90`` for a clockwise rotation.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

from paths import SPLIT_DIR, ALIGNED_DIR


# Every "<name>_translated.gcode" written by step 03 is picked up automatically.
TRANSLATED_GLOB = "*_translated.gcode"

DEFAULT_ANGLE_DEGREES = 90.0
DEFAULT_INPUT_FOLDER = SPLIT_DIR
DEFAULT_OUTPUT_FOLDER = ALIGNED_DIR
DECIMAL_PLACES = 3

NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
X_RE = re.compile(rf"(?<![A-Za-z])X\s*({NUMBER})", re.IGNORECASE)
Y_RE = re.compile(rf"(?<![A-Za-z])Y\s*({NUMBER})", re.IGNORECASE)
MOTION_RE = re.compile(r"(?:^|\s)G0*[01](?=\s|$)", re.IGNORECASE)
RELATIVE_RE = re.compile(r"(?:^|\s)G0*91(?=\s|$)", re.IGNORECASE)
ARC_RE = re.compile(r"(?:^|\s)G0*[23](?=\s|$)", re.IGNORECASE)


def code_before_comment(line: str) -> str:
    """Return executable text while ignoring a semicolon comment."""
    return line.partition(";")[0]


def find_shared_center(paths: list[Path]) -> tuple[float, float, tuple[float, ...]]:
    """Find the center of the combined XY bounding box of all motion paths."""
    x_values: list[float] = []
    y_values: list[float] = []

    for path in paths:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as file:
            for line in file:
                code = code_before_comment(line)
                if not MOTION_RE.search(code):
                    continue

                x_match = X_RE.search(code)
                y_match = Y_RE.search(code)
                if x_match:
                    x_values.append(float(x_match.group(1)))
                if y_match:
                    y_values.append(float(y_match.group(1)))

    if not x_values or not y_values:
        raise ValueError("No usable G0/G1 XY coordinates were found.")

    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    return center_x, center_y, (min_x, max_x, min_y, max_y)


def rotate_xy(
    x: float,
    y: float,
    center_x: float,
    center_y: float,
    cosine: float,
    sine: float,
) -> tuple[float, float]:
    """Rotate an XY point around the requested center."""
    dx = x - center_x
    dy = y - center_y
    rotated_x = center_x + dx * cosine - dy * sine
    rotated_y = center_y + dx * sine + dy * cosine
    return rotated_x, rotated_y


def format_coordinate(value: float) -> str:
    """Format coordinates compactly and avoid a negative zero."""
    if abs(value) < 0.5 * 10 ** (-DECIMAL_PLACES):
        value = 0.0
    result = f"{value:.{DECIMAL_PLACES}f}".rstrip("0").rstrip(".")
    return result if result not in {"", "-0"} else "0"


def replace_or_insert_xy(
    code: str,
    x_match: re.Match[str] | None,
    y_match: re.Match[str] | None,
    new_x: float,
    new_y: float,
) -> str:
    """Replace XY words; insert a missing modal axis because rotation couples XY."""
    replacements: list[tuple[int, int, str]] = []

    if x_match:
        replacements.append(
            (x_match.start(), x_match.end(), f"X{format_coordinate(new_x)}")
        )
    if y_match:
        replacements.append(
            (y_match.start(), y_match.end(), f"Y{format_coordinate(new_y)}")
        )

    # Work from right to left so original match positions remain valid.
    for start, end, text in sorted(replacements, reverse=True):
        code = code[:start] + text + code[end:]

    missing_words: list[str] = []
    if not x_match:
        missing_words.append(f"X{format_coordinate(new_x)}")
    if not y_match:
        missing_words.append(f"Y{format_coordinate(new_y)}")

    if missing_words:
        motion_match = MOTION_RE.search(code)
        if not motion_match:
            raise ValueError("Cannot insert a missing axis without a G0/G1 command.")
        insertion = " " + " ".join(missing_words)
        code = code[: motion_match.end()] + insertion + code[motion_match.end() :]

    return code


def rotate_file(
    source: Path,
    destination: Path,
    center_x: float,
    center_y: float,
    angle_degrees: float,
) -> int:
    """Rotate all absolute G0/G1 XY endpoints in one file."""
    angle_radians = math.radians(angle_degrees)
    cosine = math.cos(angle_radians)
    sine = math.sin(angle_radians)
    current_x: float | None = None
    current_y: float | None = None
    rotated_move_count = 0
    output_lines: list[str] = []

    with source.open("r", encoding="utf-8", errors="replace", newline="") as file:
        source_lines = file.readlines()

    for line_number, line in enumerate(source_lines, start=1):
        code, separator, comment = line.partition(";")

        if RELATIVE_RE.search(code):
            raise ValueError(
                f"{source.name}, line {line_number}: G91 relative positioning is "
                "not supported by this absolute-coordinate script."
            )
        if ARC_RE.search(code):
            raise ValueError(
                f"{source.name}, line {line_number}: G2/G3 arcs require rotating "
                "their I/J data and are not present in the supplied files."
            )

        if not MOTION_RE.search(code):
            output_lines.append(line)
            continue

        x_match = X_RE.search(code)
        y_match = Y_RE.search(code)
        if not x_match and not y_match:
            output_lines.append(line)
            continue

        target_x = float(x_match.group(1)) if x_match else current_x
        target_y = float(y_match.group(1)) if y_match else current_y
        if target_x is None or target_y is None:
            raise ValueError(
                f"{source.name}, line {line_number}: the first XY move does not "
                "define both axes, so its modal position is unknown."
            )

        new_x, new_y = rotate_xy(
            target_x, target_y, center_x, center_y, cosine, sine
        )
        rotated_code = replace_or_insert_xy(
            code, x_match, y_match, new_x, new_y
        )
        output_lines.append(rotated_code + separator + comment)

        # Track the original coordinate system for subsequent modal moves.
        current_x, current_y = target_x, target_y
        rotated_move_count += 1

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as file:
        file.writelines(output_lines)

    return rotated_move_count


def angle_label(angle_degrees: float) -> str:
    """Create a filename-safe angle label."""
    if angle_degrees.is_integer():
        number = str(abs(int(angle_degrees)))
    else:
        number = str(abs(angle_degrees)).replace(".", "p")
    direction = "minus" if angle_degrees < 0 else "plus"
    return f"{direction}_{number}deg"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rotate the configured CCF/PETG G-code files around one shared "
            "XY bounding-box center. Positive angles are counterclockwise."
        )
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_FOLDER,
        help=(
            "Folder holding the step 01 sample subfolders "
            f"(default: {DEFAULT_INPUT_FOLDER})."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=f"Output folder (default: {DEFAULT_OUTPUT_FOLDER}).",
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=DEFAULT_ANGLE_DEGREES,
        help="Rotation in degrees; +90 is counterclockwise, -90 is clockwise.",
    )
    parser.add_argument("--center-x", type=float, default=None)
    parser.add_argument("--center-y", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    input_directory = args.input_dir.resolve()
    output_directory = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else DEFAULT_OUTPUT_FOLDER.resolve()
    )
    if not input_directory.is_dir():
        raise FileNotFoundError(f"Step 03 output folder not found: {input_directory}")

    # One flat list across every sample, so a single shared center is used.
    paths = sorted(input_directory.glob(f"*/{TRANSLATED_GLOB}"))

    if not paths:
        raise FileNotFoundError(
            f"No {TRANSLATED_GLOB} files found under {input_directory}. "
            "Run step 03 first."
        )

    if (args.center_x is None) != (args.center_y is None):
        raise ValueError("Provide both --center-x and --center-y, or neither.")

    auto_x, auto_y, bounds = find_shared_center(paths)
    if args.center_x is None:
        center_x, center_y = auto_x, auto_y
        center_source = "automatically calculated"
    else:
        center_x, center_y = args.center_x, args.center_y
        center_source = "user supplied"

    print(
        f"Combined bounds: X={bounds[0]:.3f} to {bounds[1]:.3f}, "
        f"Y={bounds[2]:.3f} to {bounds[3]:.3f}"
    )
    print(
        f"Rotation center ({center_source}): "
        f"X={center_x:.3f}, Y={center_y:.3f}"
    )
    print(f"Rotation angle: {args.angle:g} degrees")

    suffix = angle_label(args.angle)
    total_moves = 0
    for source in paths:
        # source.parent.name is the sample folder, mirrored in the output.
        destination = (
            output_directory
            / source.parent.name
            / f"{source.stem}_rotated_{suffix}{source.suffix}"
        )
        move_count = rotate_file(
            source, destination, center_x, center_y, args.angle
        )
        total_moves += move_count
        print(
            f"Created: {source.parent.name}/{destination.name} "
            f"({move_count} XY moves rotated)"
        )

    print(f"Done: {len(paths)} files and {total_moves} XY moves processed.")
    print(f"Output folder: {output_directory}")


if __name__ == "__main__":
    main()
