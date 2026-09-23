#!/usr/bin/env python3
"""Step 03 - translate absolute X/Y positions in the split motion G-code.

Moves every sample from its NX coordinates onto the printer bed. The same
offset is applied to all four files of a sample so the PETG and CCF paths stay
aligned with each other.

With no command-line arguments, every sample subfolder produced by step 01 is
processed. Filleted variants from step 02 are preferred where they exist, so a
sample that was not filleted still works. A single file can also be translated
directly:

    python scripts/03_translate_paths.py in.gcode out.gcode --x-offset 5 --y-offset 40
"""

from __future__ import annotations

import argparse
import re
from decimal import Decimal, InvalidOperation
from pathlib import Path

from paths import SPLIT_DIR


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
AXIS_RE = re.compile(rf"(?<![A-Za-z0-9_])([XY])({NUMBER})", re.IGNORECASE)
GCODE_RE = re.compile(rf"(?<![A-Za-z0-9_])G({NUMBER})", re.IGNORECASE)
OTHER_COMMAND_RE = re.compile(r"(?<![A-Za-z0-9_])[MT]\d", re.IGNORECASE)

# Suffix added to the stem of every translated file.
TRANSLATED_SUFFIX = "_translated"

# Base name of each file to translate, in the order they are reported. For each
# one the "_filleted" variant is used when step 02 produced it.
TRANSLATED_BASE_NAMES = (
    "ccf_near_outer_boundary",
    "petg_inner_boundary",
    "petg_outer_boundary",
    "petg_full_surfaces",
)

DEFAULT_X_OFFSET = Decimal("5")
DEFAULT_Y_OFFSET = Decimal("40")


def discover_default_jobs() -> list[tuple[Path, Path]]:
    """Return (input, output) pairs for every sample folder made by step 01."""
    jobs: list[tuple[Path, Path]] = []

    for sample_dir in sorted(p for p in SPLIT_DIR.iterdir() if p.is_dir()):
        for base_name in TRANSLATED_BASE_NAMES:
            filleted = sample_dir / f"{base_name}_filleted.gcode"
            plain = sample_dir / f"{base_name}.gcode"
            source = filleted if filleted.is_file() else plain

            if not source.is_file():
                print(f"Skipping missing file: {source}")
                continue

            jobs.append(
                (source, sample_dir / f"{source.stem}{TRANSLATED_SUFFIX}.gcode")
            )

    return jobs


def format_number(value: Decimal) -> str:
    """Return a normal decimal string without unnecessary trailing zeros."""
    if value == 0:
        return "0"
    text = format(value, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text


def translate_gcode(text: str, x_offset: Decimal, y_offset: Decimal) -> tuple[str, int, int]:
    """Translate absolute X/Y motion coordinates while preserving comments."""
    absolute_xy = True
    modal_motion: int | None = None
    x_changes = 0
    y_changes = 0
    output: list[str] = []

    for line in text.splitlines(keepends=True):
        code, separator, comment = line.partition(";")
        g_values: list[Decimal] = []

        for match in GCODE_RE.finditer(code):
            try:
                g_values.append(Decimal(match.group(1)))
            except InvalidOperation:
                continue

        # G90/G91 control X/Y distance mode. G90.1/G91.1 are intentionally ignored.
        if Decimal("90") in g_values:
            absolute_xy = True
        if Decimal("91") in g_values:
            absolute_xy = False

        explicit_motion: int | None = None
        for value in g_values:
            if value in (Decimal("0"), Decimal("1"), Decimal("2"), Decimal("3")):
                explicit_motion = int(value)

        if explicit_motion is not None:
            modal_motion = explicit_motion
        if Decimal("80") in g_values:
            modal_motion = None

        has_non_motion_gcode = bool(g_values) and explicit_motion is None
        has_other_command = bool(OTHER_COMMAND_RE.search(code))
        is_modal_motion_line = (
            modal_motion is not None
            and not has_non_motion_gcode
            and not has_other_command
            and bool(AXIS_RE.search(code))
        )
        should_translate = absolute_xy and (
            explicit_motion is not None or is_modal_motion_line
        )

        if should_translate:
            def replace_axis(match: re.Match[str]) -> str:
                nonlocal x_changes, y_changes
                axis = match.group(1)
                original = Decimal(match.group(2))
                if axis.upper() == "X":
                    translated = original + x_offset
                    x_changes += 1
                else:
                    translated = original + y_offset
                    y_changes += 1
                return axis + format_number(translated)

            code = AXIS_RE.sub(replace_axis, code)

        output.append(code + separator + comment)

    return "".join(output), x_changes, y_changes


def translate_file(
    input_path: Path,
    output_path: Path,
    x_offset: Decimal,
    y_offset: Decimal,
) -> None:
    """Read, translate, and save one G-code file."""
    if not input_path.is_file():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    if input_path.resolve() == output_path.resolve():
        raise ValueError("The output path must be different from the input path.")

    with input_path.open("r", encoding="utf-8", newline="") as source:
        original_text = source.read()

    translated_text, x_changes, y_changes = translate_gcode(
        original_text, x_offset, y_offset
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as destination:
        destination.write(translated_text)

    print(f"Created: {output_path}")
    print(f"Translated X coordinates: {x_changes}")
    print(f"Translated Y coordinates: {y_changes}")
    print(f"Offsets: X + {x_offset}, Y + {y_offset}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Add an offset to absolute X/Y G-code positions. With no "
            "input/output arguments, process every sample folder under "
            "data/01_split/."
        )
    )
    parser.add_argument("input", nargs="?", type=Path, help="Input .gcode file")
    parser.add_argument("output", nargs="?", type=Path, help="Output .gcode file")
    parser.add_argument("--x-offset", type=Decimal, default=DEFAULT_X_OFFSET)
    parser.add_argument("--y-offset", type=Decimal, default=DEFAULT_Y_OFFSET)
    args = parser.parse_args()

    if (args.input is None) != (args.output is None):
        parser.error("provide both input and output, or provide neither")

    if args.input is not None and args.output is not None:
        jobs = [(args.input, args.output)]
    else:
        if not SPLIT_DIR.is_dir():
            parser.error(f"step 01 output folder not found: {SPLIT_DIR}")
        jobs = discover_default_jobs()
        if not jobs:
            parser.error(f"no G-code files found under {SPLIT_DIR}")

    try:
        for input_path, output_path in jobs:
            translate_file(
                input_path,
                output_path,
                args.x_offset,
                args.y_offset,
            )
    except (FileNotFoundError, OSError, ValueError) as error:
        parser.exit(1, f"Error: {error}\n")


if __name__ == "__main__":
    main()
