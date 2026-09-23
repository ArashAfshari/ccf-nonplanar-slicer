"""Step 09 - print the PETG support before the composite part, in one job.

This script combines:

1. an Anisoprint PETG *support toolpath module* produced by step 08,
   ``08_convert_support_toolpath.py``; and
2. a complete multilayer CCF/PETG job produced by step 07,
   ``07_assemble_multilayer_job.py``.

It does not simply concatenate the two files.  It creates a PETG-first
startup, removes the second startup, keeps the support's final retraction,
switches safely from T0 to T1, carries the support's absolute E coordinate
into every PETG command of the part, and inserts the missing PETG recovery
immediately before the first part-deposition move.

Important geometry limitation
-----------------------------
Printing every support layer before returning to the part's lowest Z can
cause a nozzle collision.  A point-based local overlap check is therefore
enabled by default.  The script stops when a part deposition endpoint is
below a nearby support endpoint.  Resolve the Z relationship in CAD/slicing,
use ``--part-z-offset`` only when that translation is intentional, or use
``--allow-potential-overlap`` only after an independent toolpath review.
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

from paths import MODULES_DIR, JOBS_DIR


# ===========================================================================
# DEFAULT FILE LOCATIONS -- command-line paths override these values
# ===========================================================================

SUPPORT_GCODE = MODULES_DIR / "anisoprint_petg_support.gcode"
MULTILAYER_PART_GCODE = JOBS_DIR / "ccf_petg_multilayer.gcode"
OUTPUT_GCODE = JOBS_DIR / "support_then_ccf_petg_multilayer.gcode"


# ===========================================================================
# TRANSITION AND VALIDATION SETTINGS
# ===========================================================================

TOOLCHANGE_TOTAL_RETRACT_MM = 12.0
TOOLCHANGE_Z_LIFT_MM = 5.0
RETRACT_FEED_MM_MIN = 2100
Z_TRAVEL_FEED_MM_MIN = 1500
XY_TRAVEL_FEED_MM_MIN = 7200

# Shift the complete multilayer model upward by exactly one 0.32 mm layer.
# This applies to every absolute Z motion from macrolayer 1 onward, including
# CCF, PETG and model-related tool-change moves.  The sacrificial CCF
# preparation line at Y=0 keeps its original Z coordinates.
PART_Z_OFFSET_MM = 0.46

# Local endpoint check.  This is deliberately conservative and is not a full
# swept-nozzle collision simulation.
GEOMETRY_XY_CHECK_RADIUS_MM = 0.35
GEOMETRY_NEGATIVE_TOLERANCE_MM = 0.05
ALLOW_POTENTIAL_OVERLAP = False


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
AXIS_RE = re.compile(rf"(?<![A-Za-z])([XYZEUVI])\s*({NUMBER})", re.I)
MOTION_COMMANDS = {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}
PART_BODY_MARKER = "SACRIFICIAL AURA-STYLE CCF PREPARATION LINE"
FIRST_MODEL_MARKER = "MACROLAYER 1 / LAYER 1"


def read_lines(path: Path) -> list[str]:
    if not path.is_file():
        raise FileNotFoundError(f"Input G-code does not exist:\n{path}")
    return path.read_text(encoding="utf-8", errors="ignore").splitlines()


def code_only(line: str) -> str:
    return line.split(";", 1)[0].strip()


def command(line: str) -> str:
    code = code_only(line)
    return code.split()[0].upper() if code else ""


def axis_values(line: str) -> dict[str, float]:
    return {
        match.group(1).upper(): float(match.group(2))
        for match in AXIS_RE.finditer(code_only(line))
    }


def fmt(value: float, decimals: int = 5) -> str:
    if abs(value) < 0.5 * 10 ** (-decimals):
        value = 0.0
    text = f"{value:.{decimals}f}"
    return text.rstrip("0").rstrip(".") if "." in text else text


def replace_axis_in_motion(line: str, axis: str, offset: float) -> str:
    """Offset one axis word in a motion command without changing comments."""

    if abs(offset) <= 1e-12 or command(line) not in MOTION_COMMANDS:
        return line

    code, separator, comment_text = line.partition(";")
    pattern = re.compile(
        rf"(?<![A-Za-z])({re.escape(axis)})\s*({NUMBER})",
        re.I,
    )

    def replacement(match: re.Match[str]) -> str:
        shifted = float(match.group(2)) + offset
        return f"{match.group(1)}{fmt(shifted)}"

    shifted_code = pattern.sub(replacement, code)
    return shifted_code + (separator + comment_text if separator else "")


def offset_absolute_e_comment(line: str, offset: float) -> str:
    """Update audit comments whose E values become global after integration."""

    if not line.lstrip().startswith(";") or abs(offset) <= 1e-12:
        return line

    # The repeated source-module summaries ("Final deposited absolute E")
    # intentionally remain local to one thermoplastic module.  Only the
    # multilayer integrator's already-global transition/shutdown values need
    # the support offset.
    phrases = (
        "T0 E DEPOSITED VALUE",
        "T0 DEPOSITED E",
    )
    if not any(phrase in line.upper() for phrase in phrases):
        return line

    pattern = re.compile(rf"(=\s*)({NUMBER})")
    match = pattern.search(line)
    if not match:
        return line
    shifted = float(match.group(2)) + offset
    return line[: match.start(2)] + fmt(shifted) + line[match.end(2) :]


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float
    line_number: int


@dataclass
class MotionScan:
    last_x: Optional[float] = None
    last_y: Optional[float] = None
    last_z: Optional[float] = None
    last_axis: Optional[float] = None
    max_z: Optional[float] = None
    max_axis: Optional[float] = None
    deposition_points: list[Point] | None = None


def scan_motion(
    lines: Iterable[str],
    extrusion_axis: str,
    start_after_marker: Optional[str] = None,
) -> MotionScan:
    """Track modal XYZ/extrusion state and collect depositing endpoints."""

    extrusion_axis = extrusion_axis.upper()
    result = MotionScan(deposition_points=[])
    absolute_xyz = True
    absolute_extrusion = True
    axis_position = 0.0
    enabled = start_after_marker is None

    for line_number, line in enumerate(lines, start=1):
        if start_after_marker and start_after_marker.upper() in line.upper():
            enabled = True

        cmd = command(line)
        values = axis_values(line)

        if cmd == "G90":
            absolute_xyz = True
            continue
        if cmd == "G91":
            absolute_xyz = False
            continue
        if cmd == "M82":
            absolute_extrusion = True
            continue
        if cmd == "M83":
            absolute_extrusion = False
            continue

        if cmd == "G92":
            if "X" in values:
                result.last_x = values["X"]
            if "Y" in values:
                result.last_y = values["Y"]
            if "Z" in values:
                result.last_z = values["Z"]
            if extrusion_axis in values:
                axis_position = values[extrusion_axis]
                result.last_axis = axis_position
            continue

        if cmd not in MOTION_COMMANDS:
            continue

        previous_axis = axis_position

        for name in ("X", "Y", "Z"):
            if name not in values:
                continue
            attribute = f"last_{name.lower()}"
            previous = getattr(result, attribute)
            if absolute_xyz:
                setattr(result, attribute, values[name])
            else:
                if previous is None:
                    raise ValueError(
                        f"Line {line_number}: relative {name} occurred before its "
                        "absolute position was known."
                    )
                setattr(result, attribute, previous + values[name])

        if result.last_z is not None:
            result.max_z = (
                result.last_z
                if result.max_z is None
                else max(result.max_z, result.last_z)
            )

        if extrusion_axis in values:
            if absolute_extrusion:
                axis_position = values[extrusion_axis]
            else:
                axis_position += values[extrusion_axis]
            result.last_axis = axis_position
            result.max_axis = (
                axis_position
                if result.max_axis is None
                else max(result.max_axis, axis_position)
            )

            has_xyz_word = any(name in values for name in ("X", "Y", "Z"))
            complete_xyz = None not in (result.last_x, result.last_y, result.last_z)
            if enabled and axis_position > previous_axis + 1e-9 and has_xyz_word and complete_xyz:
                assert result.deposition_points is not None
                result.deposition_points.append(
                    Point(
                        result.last_x,  # type: ignore[arg-type]
                        result.last_y,  # type: ignore[arg-type]
                        result.last_z,  # type: ignore[arg-type]
                        line_number,
                    )
                )

    return result


def clean_support_module(lines: list[str]) -> list[str]:
    """Remove only the support module's local mode/reset header commands."""

    cleaned: list[str] = []
    deposition_seen = False
    reset_count = 0

    for line in lines:
        cmd = command(line)
        values = axis_values(line)

        if cmd in MOTION_COMMANDS and "E" in values and any(
            name in values for name in ("X", "Y", "Z")
        ):
            deposition_seen = True

        if cmd in {"G90", "M82"}:
            continue

        if cmd == "G92" and "E" in values:
            reset_count += 1
            if deposition_seen:
                raise ValueError(
                    "The support contains a G92 E reset after deposition started. "
                    "Only a reusable support module with one initial G92 E0 is supported."
                )
            if abs(values["E"]) > 1e-9:
                raise ValueError("The support's initial extrusion reset must be G92 E0.")
            continue

        cleaned.append(line)

    if reset_count != 1:
        raise ValueError(
            f"Expected exactly one initial G92 E0 in the support; found {reset_count}."
        )

    forbidden = ("M104 S0", "M140 S0", "M530 S0")
    for line in cleaned:
        upper = code_only(line).upper()
        if any(upper.startswith(item) for item in forbidden) or upper == "G28":
            raise ValueError(
                "The support input contains printer shutdown/homing commands. "
                "Use the module generated by 08_convert_support_toolpath.py."
            )

    return cleaned


def find_marker_index(lines: list[str], marker: str) -> int:
    matches = [index for index, line in enumerate(lines) if marker.upper() in line.upper()]
    if len(matches) != 1:
        raise ValueError(
            f"Expected one '{marker}' marker in the multilayer file; found {len(matches)}."
        )
    return matches[0]


def extract_part_body(lines: list[str]) -> list[str]:
    marker_index = find_marker_index(lines, PART_BODY_MARKER)
    start = marker_index
    if marker_index > 0 and lines[marker_index - 1].lstrip().startswith("; ==="):
        start -= 1
    return lines[start:]


def extract_layer_count(lines: list[str]) -> int:
    for line in lines:
        match = re.match(r"\s*M530\s+L(\d+)\b", code_only(line), flags=re.I)
        if match:
            count = int(match.group(1))
            if count < 1:
                break
            return count
    raise ValueError("Could not find a positive M530 L<layer_count> in the part file.")


def extract_temperature(lines: list[str], tool: int, wait: bool = True) -> int:
    command_word = "M109" if wait else "M104"
    temperatures: list[int] = []
    for line in lines:
        code = code_only(line)
        if not re.match(rf"^{command_word}(?:\s|$)", code, flags=re.I):
            continue
        tool_match = re.search(r"(?<![A-Za-z])T\s*(\d+)", code, flags=re.I)
        temp_match = re.search(rf"(?<![A-Za-z])S\s*({NUMBER})", code, flags=re.I)
        if tool_match and temp_match and int(tool_match.group(1)) == tool:
            value = int(round(float(temp_match.group(1))))
            if value > 0:
                temperatures.append(value)
    if not temperatures:
        raise ValueError(f"Could not determine the T{tool} print temperature.")
    return max(temperatures)


def extract_bed_temperature(lines: list[str]) -> int:
    values: list[int] = []
    for line in lines:
        code = code_only(line)
        if not re.match(r"^M14[09](?:\s|$)", code, flags=re.I):
            continue
        match = re.search(rf"(?<![A-Za-z])S\s*({NUMBER})", code, flags=re.I)
        if match and float(match.group(1)) > 0:
            values.append(int(round(float(match.group(1)))))
    if not values:
        raise ValueError("Could not determine the positive bed temperature.")
    return max(values)


def extract_standby_and_preheat(lines: list[str], tool: int, print_temp: int) -> tuple[int, int]:
    values: set[int] = set()
    for line in lines:
        code = code_only(line)
        if not re.match(r"^M104(?:\s|$)", code, flags=re.I):
            continue
        tool_match = re.search(r"(?<![A-Za-z])T\s*(\d+)", code, flags=re.I)
        temp_match = re.search(rf"(?<![A-Za-z])S\s*({NUMBER})", code, flags=re.I)
        if tool_match and temp_match and int(tool_match.group(1)) == tool:
            value = int(round(float(temp_match.group(1))))
            if 0 < value < print_temp:
                values.add(value)
    if not values:
        raise ValueError(f"Could not determine standby/preheat temperatures for T{tool}.")
    return min(values), max(values)


def find_preparation_xy(part_body: list[str]) -> tuple[float, float]:
    """Find the preparation-line XY target before its first M1001."""

    for line in part_body:
        if command(line) == "M1001":
            break
        values = axis_values(line)
        if command(line) in MOTION_COMMANDS and "X" in values and "Y" in values:
            return values["X"], values["Y"]
    raise ValueError("Could not find the sacrificial CCF preparation XY position.")


def build_startup(
    layer_count: int,
    t0_print: int,
    t1_standby: int,
    bed_temp: int,
    part_z_offset: float,
) -> list[str]:
    return [
        "; ============================================================",
        "; ANISOPRINT SUPPORT-FIRST MULTILAYER CUSTOM G-CODE",
        "; Sequence: complete PETG support (T0) -> composite part (T1/T0)",
        f"; Composite macro-layers: {layer_count}",
        f"; Global multilayer-part Z shift: +{fmt(part_z_offset, 3)} mm",
        "; ============================================================",
        "",
        f"; LAYER_COUNT:{layer_count}",
        f"M530 L{layer_count}",
        "",
        "; ---- ANISOPRINT STARTUP: PETG SUPPORT TOOL FIRST ----",
        f"M104 S{t0_print} T0",
        f"M104 S{t1_standby} T1",
        f"M140 S{bed_temp}",
        "G21",
        "G90",
        "M82",
        "G28",
        "M530 S1",
        "G1 Z10 F900",
        "; Start change extruder",
        "M400",
        f"M109 S{t0_print} T0",
        "T0 R ; switch extruder",
        "; End change extruder",
        "G92 E0 ; reset T0 thermoplastic extrusion once",
        "G92 V0 ; reset T1 matrix extrusion once",
        "G92 U0 ; reset continuous-fiber extrusion once",
        f"M190 S{bed_temp}",
        "",
        "; ================= COMPLETE PETG SUPPORT / T0 =================",
    ]


def build_support_to_ccf_transition(
    deposited_e: float,
    current_e: float,
    safe_z: float,
    prep_x: float,
    prep_y: float,
    t0_standby: int,
    t1_preheat: int,
    t1_print: int,
) -> list[str]:
    target_e = deposited_e - TOOLCHANGE_TOTAL_RETRACT_MM
    existing_retract = deposited_e - current_e
    additional_retract = current_e - target_e

    if existing_retract < -1e-6:
        raise ValueError("Support final E is above its deposited E state.")
    if additional_retract < -1e-6:
        raise ValueError(
            f"Support already ends with a {existing_retract:.5f} mm retract, which "
            f"exceeds the configured {TOOLCHANGE_TOTAL_RETRACT_MM:.5f} mm toolchange retract."
        )

    return [
        "",
        "; ================= SUPPORT -> CCF PART =================",
        "; Start change extruder",
        "M400",
        f"; Support deposited E = {fmt(deposited_e)}",
        f"; Existing support retract = {fmt(existing_retract)} mm",
        f"; Additional retract = {fmt(additional_retract)} mm",
        f"G1 F{RETRACT_FEED_MM_MIN} E{fmt(target_e)} ; {fmt(TOOLCHANGE_TOTAL_RETRACT_MM)} mm total T0 retract",
        f"M104 S{t1_preheat} T1",
        f"G0 Z{fmt(safe_z)} F{Z_TRAVEL_FEED_MM_MIN}",
        "M1013",
        f"M104 S{t0_standby} T0",
        "M400",
        "M106 P1 S0",
        f"M109 S{t1_print} T1",
        "T1 ; switch extruder",
        "M1013 R",
        "; End change extruder",
        "M400",
        (
            f"G0 X{fmt(prep_x)} Y{fmt(prep_y)} F{XY_TRAVEL_FEED_MM_MIN} "
            "; safe XY travel before lowering to CCF preparation Z"
        ),
    ]


def transform_part_body(
    part_body: list[str],
    e_offset: float,
    part_z_offset: float,
) -> tuple[list[str], int]:
    """Offset global T0 E, optionally shift model Z, and add first recovery."""

    output: list[str] = []
    absolute_xyz = True
    model_started = False
    recovery_count = 0

    for line in part_body:
        upper = line.upper()
        cmd = command(line)
        original_values = axis_values(line)

        if FIRST_MODEL_MARKER in upper:
            model_started = True

        transformed = replace_axis_in_motion(line, "E", e_offset)
        transformed = offset_absolute_e_comment(transformed, e_offset)

        if model_started and absolute_xyz:
            transformed = replace_axis_in_motion(transformed, "Z", part_z_offset)

        is_first_part_deposition = (
            model_started
            and recovery_count == 0
            and cmd in MOTION_COMMANDS
            and "E" in original_values
            and any(name in original_values for name in ("X", "Y", "Z"))
        )
        if is_first_part_deposition:
            output.append(
                f"G1 F{RETRACT_FEED_MM_MIN} E{fmt(e_offset)} "
                "; recover parked T0 immediately before first part deposition"
            )
            recovery_count += 1

        output.append(transformed)

        if cmd == "G90":
            absolute_xyz = True
        elif cmd == "G91":
            absolute_xyz = False

    if recovery_count != 1:
        raise ValueError(
            "Could not insert exactly one PETG recovery before the first part deposition."
        )
    return output, recovery_count


@dataclass(frozen=True)
class ClearanceResult:
    minimum_dz: float
    support_point: Point
    part_point: Point


def minimum_local_endpoint_clearance(
    support_points: list[Point],
    part_points: list[Point],
    radius: float,
    part_z_offset: float,
) -> Optional[ClearanceResult]:
    """Return minimum part-Z minus support-Z for nearby deposition endpoints."""

    if not support_points or not part_points:
        return None
    if radius <= 0:
        raise ValueError("GEOMETRY_XY_CHECK_RADIUS_MM must be positive.")

    cell_size = radius
    grid: dict[tuple[int, int], list[Point]] = {}
    for point in support_points:
        key = (math.floor(point.x / cell_size), math.floor(point.y / cell_size))
        grid.setdefault(key, []).append(point)

    best: Optional[ClearanceResult] = None
    radius_squared = radius * radius

    for original_part_point in part_points:
        part_point = Point(
            original_part_point.x,
            original_part_point.y,
            original_part_point.z + part_z_offset,
            original_part_point.line_number,
        )
        key_x = math.floor(part_point.x / cell_size)
        key_y = math.floor(part_point.y / cell_size)

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for support_point in grid.get((key_x + dx, key_y + dy), []):
                    xy_distance_squared = (
                        (part_point.x - support_point.x) ** 2
                        + (part_point.y - support_point.y) ** 2
                    )
                    if xy_distance_squared > radius_squared:
                        continue
                    dz = part_point.z - support_point.z
                    if best is None or dz < best.minimum_dz:
                        best = ClearanceResult(dz, support_point, part_point)

    return best


def validate_combined_output(
    lines: list[str],
    expected_max_e: float,
) -> None:
    codes = [code_only(line).upper() for line in lines]

    if sum(code == "G28" for code in codes) != 2:
        raise ValueError("Combined output must contain exactly startup and shutdown G28 commands.")
    if sum(code.startswith("G92 E0") for code in codes) != 1:
        raise ValueError("Combined output must contain exactly one G92 E0.")
    if sum(code.startswith("M530 S0") for code in codes) != 1:
        raise ValueError("Combined output must contain exactly one final M530 S0.")

    joined = "\n".join(lines).upper()
    required_markers = (
        "COMPLETE PETG SUPPORT / T0",
        "SUPPORT -> CCF PART",
        PART_BODY_MARKER,
        FIRST_MODEL_MARKER,
        "RECOVER PARKED T0 IMMEDIATELY BEFORE FIRST PART DEPOSITION",
        "ANISOPRINT END",
    )
    positions = []
    for marker in required_markers:
        position = joined.find(marker.upper())
        if position < 0:
            raise ValueError(f"Combined output is missing required marker: {marker}")
        positions.append(position)
    if positions != sorted(positions):
        raise ValueError("Combined output phases are not in the required order.")

    scan = scan_motion(lines, "E")
    if scan.max_axis is None or abs(scan.max_axis - expected_max_e) > 1e-4:
        raise ValueError(
            "Unexpected final global E range: "
            f"expected max {expected_max_e:.5f}, got {scan.max_axis}."
        )


def integrate_support_then_part(
    support_path: Path,
    part_path: Path,
    output_path: Path,
    part_z_offset: float = PART_Z_OFFSET_MM,
    allow_potential_overlap: bool = ALLOW_POTENTIAL_OVERLAP,
) -> None:
    support_raw = read_lines(support_path)
    part_raw = read_lines(part_path)

    support_scan = scan_motion(support_raw, "E")
    if support_scan.max_axis is None or support_scan.last_axis is None:
        raise ValueError("Could not determine support deposited/current E states.")
    if None in (support_scan.last_x, support_scan.last_y, support_scan.last_z, support_scan.max_z):
        raise ValueError("Could not determine the support's final XYZ/max-Z state.")
    if not support_scan.deposition_points:
        raise ValueError("No PETG support deposition moves were found.")

    support_deposited_e = support_scan.max_axis
    support_current_e = support_scan.last_axis
    support_clean = clean_support_module(support_raw)

    layer_count = extract_layer_count(part_raw)
    t0_print = extract_temperature(part_raw, tool=0, wait=True)
    t1_print = extract_temperature(part_raw, tool=1, wait=True)
    bed_temp = extract_bed_temperature(part_raw)
    t0_standby, _t0_preheat = extract_standby_and_preheat(part_raw, 0, t0_print)
    t1_standby, t1_preheat = extract_standby_and_preheat(part_raw, 1, t1_print)

    part_body = extract_part_body(part_raw)
    prep_x, prep_y = find_preparation_xy(part_body)
    transformed_part, _ = transform_part_body(
        part_body,
        e_offset=support_deposited_e,
        part_z_offset=part_z_offset,
    )

    part_e_scan = scan_motion(part_raw, "E", start_after_marker=FIRST_MODEL_MARKER)
    part_v_scan = scan_motion(part_raw, "V", start_after_marker=FIRST_MODEL_MARKER)
    part_points = (part_e_scan.deposition_points or []) + (part_v_scan.deposition_points or [])
    clearance = minimum_local_endpoint_clearance(
        support_scan.deposition_points,
        part_points,
        radius=GEOMETRY_XY_CHECK_RADIUS_MM,
        part_z_offset=part_z_offset,
    )

    if (
        clearance is not None
        and clearance.minimum_dz < -GEOMETRY_NEGATIVE_TOLERANCE_MM
        and not allow_potential_overlap
    ):
        suggested_offset = -clearance.minimum_dz + GEOMETRY_NEGATIVE_TOLERANCE_MM
        raise ValueError(
            "Potential sequential-print collision detected.\n"
            f"A part deposition endpoint is {abs(clearance.minimum_dz):.5f} mm below "
            f"a support endpoint within {GEOMETRY_XY_CHECK_RADIUS_MM:.3f} mm XY.\n"
            f"Support: X{clearance.support_point.x:.3f} Y{clearance.support_point.y:.3f} "
            f"Z{clearance.support_point.z:.5f} (line {clearance.support_point.line_number})\n"
            f"Part:    X{clearance.part_point.x:.3f} Y{clearance.part_point.y:.3f} "
            f"Z{clearance.part_point.z:.5f} (line {clearance.part_point.line_number})\n"
            f"Endpoint-only minimum extra Z for a {GEOMETRY_NEGATIVE_TOLERANCE_MM:.3f} mm "
            f"nonnegative margin is about {suggested_offset:.5f} mm. This is not a "
            "nozzle-clearance guarantee. Correct/verify the geometry, set an intentional "
            "--part-z-offset, or use --allow-potential-overlap only after review."
        )

    safe_z = support_scan.max_z + TOOLCHANGE_Z_LIFT_MM  # type: ignore[operator]
    output_lines: list[str] = []
    output_lines += build_startup(
        layer_count=layer_count,
        t0_print=t0_print,
        t1_standby=t1_standby,
        bed_temp=bed_temp,
        part_z_offset=part_z_offset,
    )
    output_lines += support_clean
    output_lines += build_support_to_ccf_transition(
        deposited_e=support_deposited_e,
        current_e=support_current_e,
        safe_z=safe_z,
        prep_x=prep_x,
        prep_y=prep_y,
        t0_standby=t0_standby,
        t1_preheat=t1_preheat,
        t1_print=t1_print,
    )
    output_lines += transformed_part

    if part_e_scan.max_axis is None:
        raise ValueError("Could not determine the multilayer part's maximum local E.")
    expected_max_e = support_deposited_e + part_e_scan.max_axis
    validate_combined_output(output_lines, expected_max_e=expected_max_e)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(output_lines) + "\n", encoding="utf-8")

    print("Support-first multilayer G-code created successfully:")
    print(f"  {output_path}")
    print()
    print(f"Support deposited E             : {support_deposited_e:.5f} mm")
    print(f"Support final commanded E       : {support_current_e:.5f} mm")
    print(f"Support existing final retract  : {support_deposited_e - support_current_e:.5f} mm")
    print(f"Support maximum motion Z        : {support_scan.max_z:.5f} mm")
    print(f"Support-to-CCF safe travel Z     : {safe_z:.5f} mm")
    print(f"Composite macro-layers          : {layer_count}")
    print(f"Part Z offset                    : {part_z_offset:.5f} mm")
    print(f"Final global deposited E maximum: {expected_max_e:.5f} mm")
    print("First part PETG recovery         : after engage, before deposition")
    print("Second startup/shutdown          : removed/preserved correctly")
    if clearance is not None:
        status = "OVERRIDE ACCEPTED" if clearance.minimum_dz < 0 else "nonnegative"
        print(
            f"Local endpoint clearance check   : {clearance.minimum_dz:.5f} mm "
            f"({status})"
        )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Print a converted PETG support module fully, then run an existing "
            "Anisoprint multilayer CCF/PETG part job."
        )
    )
    parser.add_argument("support", nargs="?", type=Path, default=SUPPORT_GCODE)
    parser.add_argument("part", nargs="?", type=Path, default=MULTILAYER_PART_GCODE)
    parser.add_argument("output", nargs="?", type=Path, default=OUTPUT_GCODE)
    parser.add_argument(
        "--part-z-offset",
        type=float,
        default=PART_Z_OFFSET_MM,
        help=(
            "Intentional Z translation applied to model macro-layers only; the "
            "sacrificial CCF preparation line is not shifted (default: 0)."
        ),
    )
    parser.add_argument(
        "--allow-potential-overlap",
        action="store_true",
        default=ALLOW_POTENTIAL_OVERLAP,
        help=(
            "Generate despite a part endpoint lying below nearby completed support. "
            "Use only after independent geometry/toolpath review."
        ),
    )
    return parser


def main() -> None:
    args = build_argument_parser().parse_args()
    integrate_support_then_part(
        support_path=args.support,
        part_path=args.part,
        output_path=args.output,
        part_z_offset=args.part_z_offset,
        allow_potential_overlap=args.allow_potential_overlap,
    )


if __name__ == "__main__":
    main()
