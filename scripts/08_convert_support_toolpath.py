"""Convert a PrusaSlicer PETG support job to an Anisoprint PETG toolpath.

The output is intentionally a toolpath module, not a complete printer job.  It
contains XYZ motion, absolute PETG E values, Anisoprint travel retractions and
PETG fan commands.  Prusa/MK4 homing, probing, temperature, purge, acceleration,
progress and shutdown commands are removed so they cannot conflict with the
Anisoprint start/end or composite-tool-change code.

Conversion rules
----------------
* Start at the first ``;LAYER_CHANGE`` and stop at the final ``;TYPE:Custom``.
* Keep all printed features by default.  The input object itself is named
  ``Support.stl``; filtering only ``;TYPE:Support material`` would incorrectly
  remove its perimeters, infill, bridges and overhang walls.
* Preserve positive Prusa E increments (the source uses 1.75 mm PETG), convert
  them to cumulative absolute E, and ignore Prusa E-only prime/recover values.
* Replace every Prusa retract/wipe cycle with the configured Anisoprint 9 mm
  absolute-E retract.  Recovery is emitted only after travel positioning and
  immediately before the next depositing move.
* Convert G2/G3 arcs to short G1 chords because the existing Anisoprint PETG
  converter is G0/G1 based.
* Use full 3D XYZ coordinates and retain the source layer heights/geometry.

The defaults below match the files supplied with this task.  They can be
changed in the configuration section or overridden with command-line paths.
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

from paths import INPUT_DIR, MODULES_DIR


# ===========================================================================
# FILE LOCATIONS
# ===========================================================================

INPUT_GCODE = INPUT_DIR / "prusaslicer_petg_support.gcode"
OUTPUT_GCODE = MODULES_DIR / "anisoprint_petg_support.gcode"


# ===========================================================================
# ANISOPRINT PETG PARAMETERS
# ===========================================================================

# Aura/Composer material travel retract (7 mm material + 2 mm additional).
TRAVEL_RETRACTION_MM = 9.0
RETRACTION_FEEDRATE_MM_MIN = 2100

# Preserve slower source speeds, but cap faster Prusa deposition/travel moves.
DEFAULT_PRINT_FEEDRATE_MM_MIN = 1800
MAX_PRINT_FEEDRATE_MM_MIN = 1800
DEFAULT_TRAVEL_FEEDRATE_MM_MIN = 7200
MAX_TRAVEL_FEEDRATE_MM_MIN = 7200
Z_TRAVEL_FEEDRATE_MM_MIN = 720

# Keep the Prusa volumetric intent.  Change only for an intentional flow tune.
EXTRUSION_SCALE = 1.0

# Anisoprint uses fan channel P1.  Aura PETG commonly uses 76, with 127 for
# bridges.  Values use the 0..255 PWM scale.
PETG_FAN_PWM = 76
BRIDGE_FAN_PWM = 127
FAN_PORT = 1

# Maximum chord length used while replacing a G2/G3 arc with G1 moves.
MAX_ARC_CHORD_MM = 0.50

# False follows PrusaSlicer's own retract decisions.  This avoids applying a
# large 9 mm Anisoprint retract to every tiny no-extrusion repositioning move.
# Set True only if every travel in your workflow must be retracted.
RETRACT_ON_EVERY_TRAVEL = False

# The default keeps the complete Support.stl print, including its own walls,
# infill and bridges plus Prusa-generated support/interface.  Setting this True
# keeps only features whose ;TYPE begins with "Support material".
ONLY_PRUSA_GENERATED_SUPPORT_TYPES = False


NUMBER_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
VALUE_RE_TEMPLATE = r"(?<![A-Za-z]){letter}\s*({number})"
MOTION_RE = re.compile(r"^(G0|G00|G1|G01|G2|G02|G3|G03)(?:\s|$)", re.I)


def get_value(command: str, letter: str) -> Optional[float]:
    """Return a G-code word value, or None when the word is absent."""

    pattern = VALUE_RE_TEMPLATE.format(letter=re.escape(letter), number=NUMBER_RE)
    match = re.search(pattern, command, flags=re.IGNORECASE)
    return float(match.group(1)) if match else None


def fmt(value: float, decimals: int = 5) -> str:
    """Format a number without negative zero."""

    if abs(value) < 0.5 * 10 ** (-decimals):
        value = 0.0
    return f"{value:.{decimals}f}"


def split_command(raw_line: str) -> str:
    """Remove an inline comment and surrounding whitespace."""

    return raw_line.split(";", 1)[0].strip()


def capped_feedrate(source: Optional[float], depositing: bool, z_only: bool) -> int:
    """Select a conservative target feedrate while preserving slower moves."""

    if depositing:
        default = DEFAULT_PRINT_FEEDRATE_MM_MIN
        maximum = MAX_PRINT_FEEDRATE_MM_MIN
    elif z_only:
        default = Z_TRAVEL_FEEDRATE_MM_MIN
        maximum = Z_TRAVEL_FEEDRATE_MM_MIN
    else:
        default = DEFAULT_TRAVEL_FEEDRATE_MM_MIN
        maximum = MAX_TRAVEL_FEEDRATE_MM_MIN

    selected = default if source is None or source <= 0 else source
    return max(1, int(round(min(selected, maximum))))


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float


@dataclass
class ConversionStats:
    layers: int = 0
    source_deposition_moves: int = 0
    output_deposition_segments: int = 0
    travel_moves: int = 0
    retracts: int = 0
    recovers: int = 0
    arcs_linearized: int = 0
    skipped_feature_moves: int = 0
    deposited_filament_mm: float = 0.0
    deposited_path_mm: float = 0.0


def point_distance(a: Point, b: Point) -> float:
    return math.sqrt(
        (b.x - a.x) ** 2
        + (b.y - a.y) ** 2
        + (b.z - a.z) ** 2
    )


def linearize_arc(
    start: Point,
    end: Point,
    i_offset: float,
    j_offset: float,
    clockwise: bool,
    max_chord_mm: float,
) -> list[Point]:
    """Return G1 endpoints for an XY-plane I/J arc; final point is exact."""

    center_x = start.x + i_offset
    center_y = start.y + j_offset
    radius = math.hypot(start.x - center_x, start.y - center_y)

    if radius <= 1e-9:
        raise ValueError("Arc radius is zero.")

    start_angle = math.atan2(start.y - center_y, start.x - center_x)
    end_angle = math.atan2(end.y - center_y, end.x - center_x)
    sweep = end_angle - start_angle

    if clockwise:
        while sweep >= -1e-12:
            sweep -= 2.0 * math.pi
    else:
        while sweep <= 1e-12:
            sweep += 2.0 * math.pi

    arc_length_xy = abs(sweep) * radius
    segment_count = max(1, int(math.ceil(arc_length_xy / max_chord_mm)))

    points: list[Point] = []
    for index in range(1, segment_count + 1):
        fraction = index / segment_count
        angle = start_angle + sweep * fraction
        point = Point(
            center_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
            start.z + (end.z - start.z) * fraction,
        )
        points.append(point)

    # Do not allow floating-point arc reconstruction to alter the exact endpoint.
    points[-1] = end
    return points


class PrusaToAnisoprintConverter:
    def __init__(
        self,
        retract_on_every_travel: bool = RETRACT_ON_EVERY_TRAVEL,
        support_types_only: bool = ONLY_PRUSA_GENERATED_SUPPORT_TYPES,
    ) -> None:
        self.retract_on_every_travel = retract_on_every_travel
        self.support_types_only = support_types_only

        self.output: list[str] = []
        self.stats = ConversionStats()

        self.started = False
        self.finished = False
        self.absolute_xyz = True
        self.absolute_source_e = False
        self.source_e_position = 0.0
        self.source_feedrate: Optional[float] = None

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.z: Optional[float] = None
        self.position_is_emitted = False

        self.feature_type: Optional[str] = None
        self.absolute_output_e = 0.0
        self.has_deposited = False
        self.output_e_is_retracted = False
        self.current_fan_pwm: Optional[int] = None

    def append_header(self, input_path: Path) -> None:
        filter_note = (
            "only Prusa-generated Support material types"
            if self.support_types_only
            else "all printed features of Support.stl"
        )
        self.output.extend(
            [
                "; ANISOPRINT PETG SUPPORT TOOLPATH",
                f"; Converted from: {input_path.name}",
                "; Prusa/MK4 machine startup and shutdown commands removed",
                f"; Feature selection: {filter_note}",
                "; Positive Prusa relative E converted to cumulative absolute E",
                (
                    f"; Travel retract = {TRAVEL_RETRACTION_MM:.3f} mm; "
                    "recovery occurs immediately before deposition"
                ),
                f"; G2/G3 arc chord limit = {MAX_ARC_CHORD_MM:.3f} mm",
                "G90",
                "M82",
                "G92 E0",
                f"M106 P{FAN_PORT} S{PETG_FAN_PWM} ; PETG cooling fan",
                "",
            ]
        )
        self.current_fan_pwm = PETG_FAN_PWM

    def complete_point(self) -> Optional[Point]:
        if self.x is None or self.y is None or self.z is None:
            return None
        return Point(self.x, self.y, self.z)

    def feature_is_kept(self) -> bool:
        if not self.support_types_only:
            return True
        return bool(
            self.feature_type
            and self.feature_type.lower().startswith("support material")
        )

    def desired_fan_pwm(self) -> int:
        if self.feature_type and self.feature_type.lower().startswith("bridge"):
            return BRIDGE_FAN_PWM
        return PETG_FAN_PWM

    def set_fan_for_deposition(self) -> None:
        desired = self.desired_fan_pwm()
        if desired != self.current_fan_pwm:
            label = "bridge cooling fan" if desired == BRIDGE_FAN_PWM else "PETG cooling fan"
            self.output.append(f"M106 P{FAN_PORT} S{desired} ; {label}")
            self.current_fan_pwm = desired

    def retract(self, reason: str) -> None:
        if not self.has_deposited or self.output_e_is_retracted:
            return
        commanded_e = self.absolute_output_e - TRAVEL_RETRACTION_MM
        self.output.append(
            f"G1 F{RETRACTION_FEEDRATE_MM_MIN} E{fmt(commanded_e)} ; {reason}"
        )
        self.output_e_is_retracted = True
        self.stats.retracts += 1

    def recover(self) -> None:
        if not self.output_e_is_retracted:
            return
        self.output.append(
            f"G1 F{RETRACTION_FEEDRATE_MM_MIN} E{fmt(self.absolute_output_e)} "
            "; PETG travel recovery at deposition start"
        )
        self.output_e_is_retracted = False
        self.stats.recovers += 1

    def emit_first_position(self, target: Point) -> None:
        self.output.append(
            f"G0 X{fmt(target.x, 3)} Y{fmt(target.y, 3)} "
            f"Z{fmt(target.z, 3)} F{DEFAULT_TRAVEL_FEEDRATE_MM_MIN} "
            "; first support position"
        )
        self.position_is_emitted = True
        self.stats.travel_moves += 1

    def emit_travel(self, start: Point, end: Point, source_feed: Optional[float]) -> None:
        if point_distance(start, end) <= 1e-12:
            return
        if self.retract_on_every_travel:
            self.retract("PETG travel retract before travel")

        z_only = abs(end.x - start.x) <= 1e-12 and abs(end.y - start.y) <= 1e-12
        feed = capped_feedrate(source_feed, depositing=False, z_only=z_only)
        self.output.append(
            f"G0 X{fmt(end.x, 3)} Y{fmt(end.y, 3)} Z{fmt(end.z, 3)} F{feed}"
        )
        self.stats.travel_moves += 1

    def emit_deposition_points(
        self,
        start: Point,
        points: Sequence[Point],
        total_delta_e: float,
        source_feed: Optional[float],
    ) -> None:
        if not points or total_delta_e <= 0:
            return

        self.recover()
        self.set_fan_for_deposition()
        feed = capped_feedrate(source_feed, depositing=True, z_only=False)
        delta_per_segment = total_delta_e * EXTRUSION_SCALE / len(points)
        previous = start

        for point in points:
            self.absolute_output_e += delta_per_segment
            self.output.append(
                f"G1 X{fmt(point.x, 3)} Y{fmt(point.y, 3)} Z{fmt(point.z, 3)} "
                f"E{fmt(self.absolute_output_e)} F{feed}"
            )
            self.stats.output_deposition_segments += 1
            self.stats.deposited_path_mm += point_distance(previous, point)
            previous = point

        self.stats.source_deposition_moves += 1
        self.stats.deposited_filament_mm += total_delta_e * EXTRUSION_SCALE
        self.has_deposited = True

    def source_delta_e(self, e_word: Optional[float]) -> Optional[float]:
        if e_word is None:
            return None

        if self.absolute_source_e:
            delta = e_word - self.source_e_position
            self.source_e_position = e_word
            return delta

        self.source_e_position += e_word
        return e_word

    def target_from_words(
        self,
        x_word: Optional[float],
        y_word: Optional[float],
        z_word: Optional[float],
    ) -> tuple[Optional[Point], Optional[Point]]:
        """Update XYZ state and return (previous complete point, target point)."""

        previous = self.complete_point()

        if self.absolute_xyz:
            if x_word is not None:
                self.x = x_word
            if y_word is not None:
                self.y = y_word
            if z_word is not None:
                self.z = z_word
        else:
            if self.x is None or self.y is None or self.z is None:
                raise ValueError(
                    "Relative XYZ motion occurred before a complete starting "
                    "position was known. Convert the source to absolute XYZ first."
                )
            self.x += x_word or 0.0
            self.y += y_word or 0.0
            self.z += z_word or 0.0

        return previous, self.complete_point()

    def handle_comment(self, stripped: str) -> None:
        upper = stripped.upper()

        if upper == ";LAYER_CHANGE":
            if not self.started:
                # The MK4 purge location is not a valid starting state for a
                # reusable Anisoprint module.  Rebuild the first complete point
                # from the print-region coordinates instead.
                self.started = True
                self.x = self.y = self.z = None
                self.position_is_emitted = False
            self.stats.layers += 1
            self.output.append(";LAYER_CHANGE")
            return

        if not self.started:
            return

        if upper == ";TYPE:CUSTOM":
            self.finished = True
            return

        if upper.startswith(";TYPE:"):
            self.feature_type = stripped.split(":", 1)[1].strip()
            self.output.append(f";TYPE:{self.feature_type}")
            return

        preserved_prefixes = (
            ";Z:",
            ";HEIGHT:",
            ";WIDTH:",
            ";BEFORE_LAYER_CHANGE",
            ";AFTER_LAYER_CHANGE",
        )
        if upper.startswith(preserved_prefixes):
            self.output.append(stripped)

    def handle_command(self, command: str, source_line_number: int) -> None:
        upper = command.upper()

        if re.match(r"^G90(?:\s|$)", upper):
            self.absolute_xyz = True
            return
        if re.match(r"^G91(?:\s|$)", upper):
            self.absolute_xyz = False
            return
        if re.match(r"^M82(?:\s|$)", upper):
            self.absolute_source_e = True
            return
        if re.match(r"^M83(?:\s|$)", upper):
            self.absolute_source_e = False
            return

        if re.match(r"^G92(?:\s|$)", upper):
            if any(get_value(command, axis) is not None for axis in ("X", "Y", "Z")):
                raise ValueError(
                    f"Source line {source_line_number}: XYZ G92 offsets are unsupported."
                )
            reset_e = get_value(command, "E")
            if reset_e is not None:
                self.source_e_position = reset_e
            return

        motion_match = MOTION_RE.match(command)
        if not motion_match:
            # Deliberately discard all Prusa/MK4 machine-specific commands.
            return

        motion_word = motion_match.group(1).upper()
        x_word = get_value(command, "X")
        y_word = get_value(command, "Y")
        z_word = get_value(command, "Z")
        e_word = get_value(command, "E")
        f_word = get_value(command, "F")

        if f_word is not None:
            self.source_feedrate = f_word
        delta_e = self.source_delta_e(e_word)

        if not self.started:
            return

        # A negative source E (including negative-E wipe motion) marks the
        # Prusa retract cycle.  Replace the small Prusa value with 9 mm once.
        if delta_e is not None and delta_e < -1e-9:
            self.retract("PETG travel retract before Prusa wipe/travel")

        has_xyz = x_word is not None or y_word is not None or z_word is not None
        if not has_xyz:
            # Feed-only and E-only moves update source state above.  Positive
            # Prusa recovery is intentionally delayed to actual deposition.
            return

        previous, target = self.target_from_words(x_word, y_word, z_word)
        if target is None:
            return

        if not self.position_is_emitted:
            self.emit_first_position(target)
            return

        if previous is None:
            raise ValueError(
                f"Source line {source_line_number}: incomplete previous XYZ state."
            )

        depositing = delta_e is not None and delta_e > 1e-9 and self.feature_is_kept()

        if not depositing:
            if delta_e is not None and delta_e > 1e-9:
                self.stats.skipped_feature_moves += 1
                self.retract("PETG retract before excluded feature")

            if motion_word in ("G2", "G02", "G3", "G03"):
                i_word = get_value(command, "I")
                j_word = get_value(command, "J")
                if i_word is None and j_word is None:
                    raise ValueError(
                        f"Source line {source_line_number}: only I/J arcs are supported."
                    )
                # In standard G-code, an omitted I or J offset means zero.
                i_word = 0.0 if i_word is None else i_word
                j_word = 0.0 if j_word is None else j_word
                travel_points = linearize_arc(
                    previous,
                    target,
                    i_word,
                    j_word,
                    clockwise=motion_word in ("G2", "G02"),
                    max_chord_mm=MAX_ARC_CHORD_MM,
                )
                self.stats.arcs_linearized += 1
                arc_previous = previous
                for point in travel_points:
                    self.emit_travel(arc_previous, point, self.source_feedrate)
                    arc_previous = point
            else:
                self.emit_travel(previous, target, self.source_feedrate)
            return

        if motion_word in ("G2", "G02", "G3", "G03"):
            i_word = get_value(command, "I")
            j_word = get_value(command, "J")
            if i_word is None and j_word is None:
                raise ValueError(
                    f"Source line {source_line_number}: only I/J arcs are supported."
                )
            # In standard G-code, an omitted I or J offset means zero.
            i_word = 0.0 if i_word is None else i_word
            j_word = 0.0 if j_word is None else j_word
            points = linearize_arc(
                previous,
                target,
                i_word,
                j_word,
                clockwise=motion_word in ("G2", "G02"),
                max_chord_mm=MAX_ARC_CHORD_MM,
            )
            self.stats.arcs_linearized += 1
        else:
            points = [target]

        self.emit_deposition_points(
            previous,
            points,
            total_delta_e=delta_e,
            source_feed=self.source_feedrate,
        )

    def convert_lines(self, lines: Iterable[str], input_path: Path) -> list[str]:
        self.append_header(input_path)

        for source_line_number, raw_line in enumerate(lines, start=1):
            stripped = raw_line.strip()
            if not stripped:
                continue
            if stripped.startswith(";"):
                self.handle_comment(stripped)
                if self.finished:
                    break
                continue

            command = split_command(stripped)
            if command:
                self.handle_command(command, source_line_number)

        if not self.started:
            raise ValueError("No ;LAYER_CHANGE marker was found in the Prusa G-code.")
        if not self.has_deposited:
            raise ValueError("No depositing moves were found in the selected print region.")

        self.retract("Final PETG travel retract")
        if self.current_fan_pwm != 0:
            self.output.append(f"M106 P{FAN_PORT} S0 ; PETG fan off")
            self.current_fan_pwm = 0

        final_commanded_e = (
            self.absolute_output_e - TRAVEL_RETRACTION_MM
            if self.output_e_is_retracted
            else self.absolute_output_e
        )
        self.output.extend(
            [
                "",
                f"; Layers converted = {self.stats.layers}",
                f"; Deposited path = {self.stats.deposited_path_mm:.3f} mm",
                f"; Deposited filament E = {self.stats.deposited_filament_mm:.5f} mm",
                f"; Final deposited absolute E = {self.absolute_output_e:.5f} mm",
                f"; Final commanded absolute E = {final_commanded_e:.5f} mm",
                f"; Final PETG retract = {TRAVEL_RETRACTION_MM:.3f} mm",
            ]
        )
        return self.output


def convert_prusa_support_gcode(
    input_path: Path,
    output_path: Path,
    retract_on_every_travel: bool = RETRACT_ON_EVERY_TRAVEL,
    support_types_only: bool = ONLY_PRUSA_GENERATED_SUPPORT_TYPES,
) -> ConversionStats:
    if not input_path.is_file():
        raise FileNotFoundError(f"Input G-code does not exist:\n{input_path}")

    lines = input_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    converter = PrusaToAnisoprintConverter(
        retract_on_every_travel=retract_on_every_travel,
        support_types_only=support_types_only,
    )
    output_lines = converter.convert_lines(lines, input_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")
    return converter.stats


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert PrusaSlicer PETG support G-code to Anisoprint PETG G-code."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=INPUT_GCODE,
        help=f"Prusa input G-code (default: {INPUT_GCODE})",
    )
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=OUTPUT_GCODE,
        help=f"Anisoprint output G-code (default: {OUTPUT_GCODE})",
    )
    parser.add_argument(
        "--retract-all-travels",
        action="store_true",
        help="Use a 9 mm retract before every travel, not only Prusa retract cycles.",
    )
    parser.add_argument(
        "--support-types-only",
        action="store_true",
        help=(
            "Keep only ;TYPE:Support material/interface. Do not use this for the "
            "complete Support.stl object."
        ),
    )
    return parser


def main() -> None:
    args = build_argument_parser().parse_args()
    stats = convert_prusa_support_gcode(
        args.input,
        args.output,
        retract_on_every_travel=args.retract_all_travels,
        support_types_only=args.support_types_only,
    )

    print("Finished.")
    print(f"Layers: {stats.layers}")
    print(f"Source deposition moves: {stats.source_deposition_moves}")
    print(f"Output deposition segments: {stats.output_deposition_segments}")
    print(f"G2/G3 arcs linearized: {stats.arcs_linearized}")
    print(f"Travel retracts/recoveries: {stats.retracts}/{stats.recovers}")
    print(f"Deposited path: {stats.deposited_path_mm:.3f} mm")
    print(f"Deposited filament: {stats.deposited_filament_mm:.5f} mm")
    print(f"Saved to:\n{args.output}")


if __name__ == "__main__":
    main()
