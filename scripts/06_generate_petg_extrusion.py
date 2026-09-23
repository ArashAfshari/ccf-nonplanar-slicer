"""
Step 06 - pure thermoplastic motion G-code -> extrusion G-code converter.

Produces the three PETG modules consumed by step 07: the inner boundary, the
full bottom/top surfaces, and the outer boundary.

Engage/retract classification
-----------------------------
A G1 next to a G0 travel may be a material-free positioning move rather than a
deposition move. Which G1 moves count as material-free differs between the
closed outer boundary and the open inner/surface paths, so each job selects a
mode explicitly:

``VERTICAL_ONLY``
    Only a purely vertical approach/lift adjacent to a G0 is material-free.
    XY printing moves still extrude even when they are the last G1 before a
    G0. Used for the inner boundary and the full surfaces.

``ANY_ADJACENT``
    Any G1 immediately before or after a G0 is material-free. Used for the
    outer boundary, whose loops enter and leave along the path direction.

These two modes were previously two separate scripts. The behaviour of each is
preserved exactly; only the settings block is shared.

This converter contains ONLY:
- XYZ motion handling
- thermoplastic E calculation
- optional feedrate handling
- non-depositing G1 engage/retract moves next to G0 travel
- absolute-E travel retraction before leaving a deposited path
- absolute-E recovery only after travel positioning is complete

It intentionally contains NO:
- tool changes (T0/T1/etc.)
- CCF commands
- thermoplastic <-> CCF transitions
- cutter commands
- purge/prime sequences
- temperature commands
- bed commands
- material switching logic

Extrusion model:
    L = sqrt(dx^2 + dy^2 + dz^2)

    dE = L * extrusion_width * deposition_height
         / filament_cross_section_area
         * flow_multiplier

Absolute extrusion is generated using M82.
"""

import math
import re
from dataclasses import dataclass
from pathlib import Path

from paths import ALIGNED_DIR, MODULES_DIR


# ============================================================
# FILE LOCATIONS
# ============================================================

# See the module docstring for what each mode means.
VERTICAL_ONLY = "vertical_only"
ANY_ADJACENT = "any_adjacent"


@dataclass(frozen=True)
class ConversionJob:
    """One motion file, its PETG output, and its engage/retract rule."""

    input_file: Path
    output_file: Path
    non_depositing_mode: str


# The PETG geometry comes from the first sample; step 05 takes the fiber track
# from the second. Change PETG_SAMPLE to use a different sample's PETG paths.
PETG_SAMPLE = "petg_source"
INPUT_DIRECTORY = ALIGNED_DIR / PETG_SAMPLE
OUTPUT_DIRECTORY = MODULES_DIR

# Each input is converted independently. Therefore, every output file receives
# its own G92 E0 initialization and maintains a separate extrusion state.
CONVERSION_JOBS = (
    ConversionJob(
        INPUT_DIRECTORY
        / "petg_inner_boundary_filleted_translated_rotated_plus_90deg.gcode",
        OUTPUT_DIRECTORY / "anisoprint_petg_inner.gcode",
        VERTICAL_ONLY,
    ),
    ConversionJob(
        INPUT_DIRECTORY
        / "petg_full_surfaces_filleted_translated_rotated_plus_90deg.gcode",
        OUTPUT_DIRECTORY / "anisoprint_petg_full_surfaces.gcode",
        VERTICAL_ONLY,
    ),
    ConversionJob(
        INPUT_DIRECTORY
        / "petg_outer_boundary_translated_rotated_plus_90deg.gcode",
        OUTPUT_DIRECTORY / "anisoprint_petg_outer_boundary.gcode",
        ANY_ADJACENT,
    ),
)


# ============================================================
# THERMOPLASTIC PARAMETERS
# ============================================================

FILAMENT_DIAMETER_MM = 1.75

EXTRUSION_WIDTH_MM = 0.40
DEPOSITION_HEIGHT_MM = 0.32

FLOW_MULTIPLIER = 1.00

PRINT_FEEDRATE_MM_MIN = 1800
TRAVEL_FEEDRATE_MM_MIN = 7200

# Aura/Composer travel retraction for this material/printer combination:
# 7 mm material travel retract + 2 mm printer additional retract = 9 mm.
TRAVEL_RETRACTION_MM = 9.0
RETRACTION_FEEDRATE_MM_MIN = 2100

ENABLE_TRAVEL_RETRACTION = True

# Only vertical approach/lift moves adjacent to G0 are non-depositing.
# XY printing moves must extrude even when they are the last G1 before G0.
POSITION_TOLERANCE_MM = 0.0001


# If the first point in the motion file is only the starting
# point of the path, leave this as None.
#
# If you already know the XYZ position immediately BEFORE the
# first depositing G1 move, enter it here:
#
# START_XYZ = (35.500, -6.800, 0.900)
#
START_XYZ = None


# ============================================================
# EXTRUSION CALCULATION
# ============================================================

FILAMENT_AREA_MM2 = math.pi * (FILAMENT_DIAMETER_MM / 2.0) ** 2

EXTRUSION_PER_PATH_MM = (
    EXTRUSION_WIDTH_MM
    * DEPOSITION_HEIGHT_MM
    * FLOW_MULTIPLIER
    / FILAMENT_AREA_MM2
)


def get_value(line, letter):
    pattern = rf"(?<![A-Za-z]){letter}\s*(-?(?:\d+(?:\.\d*)?|\.\d+))"
    match = re.search(pattern, line, flags=re.IGNORECASE)

    if match:
        return float(match.group(1))

    return None


def fmt(value, decimals=5):
    return f"{value:.{decimals}f}"


def make_e_only_move(e_value, comment):
    """Create an absolute-E retract/recovery move without XYZ motion."""

    return (
        f"G1 "
        f"F{RETRACTION_FEEDRATE_MM_MIN} "
        f"E{fmt(e_value, 5)} "
        f"; {comment}"
    )


def get_xyz_motion_type(raw_line):
    """Return normalized G0/G1 for an XYZ move, otherwise None."""

    stripped = raw_line.strip()

    if not stripped or stripped.startswith(";"):
        return None

    command = stripped.split(";", 1)[0].strip()

    if not command:
        return None

    motion = re.match(
        r"^(G0|G00|G1|G01)(?:\s|$)",
        command,
        flags=re.IGNORECASE,
    )

    if not motion:
        return None

    if all(
        get_value(command, axis) is None
        for axis in ("X", "Y", "Z")
    ):
        return None

    if motion.group(1).upper() in ("G0", "G00"):
        return "G0"

    return "G1"


def convert_motion_gcode(input_path, output_path, non_depositing_mode=VERTICAL_ONLY):

    if non_depositing_mode not in (VERTICAL_ONLY, ANY_ADJACENT):
        raise ValueError(
            f"Unknown non_depositing_mode: {non_depositing_mode!r}. "
            f"Use {VERTICAL_ONLY!r} or {ANY_ADJACENT!r}."
        )

    if not input_path.exists():
        raise FileNotFoundError(
            f"Input G-code does not exist:\n{input_path}"
        )

    lines = input_path.read_text(
        encoding="utf-8",
        errors="ignore"
    ).splitlines()

    # Adjacency alone does NOT imply an engage/retract move. Record
    # neighbors here; classify actual displacement after resolving XYZ.
    xyz_motions = []

    for line_index, raw_line in enumerate(lines):
        normalized_motion = get_xyz_motion_type(raw_line)

        if normalized_motion is not None:
            xyz_motions.append((line_index, normalized_motion))

    post_travel_g1_lines = set()
    pre_travel_g1_lines = set()
    # ANY_ADJACENT mode only: every G1 touching a G0 on either side.
    adjacent_g1_lines = set()

    for motion_index, (line_index, normalized_motion) in enumerate(xyz_motions):

        if normalized_motion != "G1":
            continue

        previous_motion = (
            xyz_motions[motion_index - 1][1]
            if motion_index > 0
            else None
        )

        next_motion = (
            xyz_motions[motion_index + 1][1]
            if motion_index + 1 < len(xyz_motions)
            else None
        )

        if previous_motion == "G0":
            post_travel_g1_lines.add(line_index)

        if next_motion == "G0":
            pre_travel_g1_lines.add(line_index)

        if previous_motion == "G0" or next_motion == "G0":
            adjacent_g1_lines.add(line_index)

    output = []

    # Only positioning/extrusion mode initialization.
    # No tools, temperatures, transitions, purging, etc.
    output.append("; PURE THERMOPLASTIC EXTRUSION")
    output.append("; Full XYZ distance used for non-planar motion")
    if non_depositing_mode == VERTICAL_ONLY:
        output.append(
            "; Only vertical G1 approach/lift next to G0 is non-depositing"
        )
    else:
        output.append("; G1 next to G0 = engage/retract without extrusion")
    output.append(
        f"; Travel retract = {TRAVEL_RETRACTION_MM:.3f} mm before travel"
    )
    output.append(
        "; Travel recovery occurs after G0/engage positioning, "
        "immediately before deposition"
    )
    output.append(
        f"; E coefficient = {EXTRUSION_PER_PATH_MM:.9f} mm/mm"
    )
    output.append("G90")
    output.append("M82")
    output.append("G92 E0")
    output.append("")

    if START_XYZ is None:
        current_x = None
        current_y = None
        current_z = None
    else:
        current_x, current_y, current_z = START_XYZ

    absolute_xyz = True
    absolute_e = 0.0

    first_complete_position = START_XYZ is not None

    total_path_length = 0.0
    total_filament = 0.0
    previous_output_motion = None

    has_deposited = False
    e_is_retracted = False

    for line_index, raw_line in enumerate(lines):

        stripped = raw_line.strip()

        if not stripped:
            continue

        # Preserve comments only.
        if stripped.startswith(";"):
            output.append(stripped)
            continue

        command = stripped.split(";", 1)[0].strip()

        if not command:
            continue

        upper = command.upper()

        # ----------------------------------------------------
        # XYZ coordinate mode
        # ----------------------------------------------------

        if re.match(r"^G90(?:\s|$)", upper):
            absolute_xyz = True
            continue

        if re.match(r"^G91(?:\s|$)", upper):
            absolute_xyz = False
            continue

        # Ignore original extrusion mode and original E reset.
        if re.match(r"^M82(?:\s|$)", upper):
            continue

        if re.match(r"^M83(?:\s|$)", upper):
            continue

        if re.match(r"^G92(?:\s|$)", upper):

            if (
                get_value(command, "X") is not None
                or get_value(command, "Y") is not None
                or get_value(command, "Z") is not None
            ):
                raise ValueError(
                    "XYZ G92 coordinate offsets are not supported."
                )

            continue

        # ----------------------------------------------------
        # Process only G0 / G1 motion
        # ----------------------------------------------------

        motion = re.match(
            r"^(G0|G00|G1|G01)(?:\s|$)",
            upper
        )

        if not motion:
            # Everything unrelated to XYZ motion is discarded.
            continue

        motion_type = motion.group(1)

        x = get_value(command, "X")
        y = get_value(command, "Y")
        z = get_value(command, "Z")
        f = get_value(command, "F")

        if x is None and y is None and z is None:
            continue

        # ----------------------------------------------------
        # Determine target XYZ
        # ----------------------------------------------------

        if absolute_xyz:

            target_x = x if x is not None else current_x
            target_y = y if y is not None else current_y
            target_z = z if z is not None else current_z

        else:

            if (
                current_x is None
                or current_y is None
                or current_z is None
            ):
                raise ValueError(
                    "Relative motion found before starting XYZ "
                    "was known. Set START_XYZ."
                )

            target_x = current_x + (x if x is not None else 0.0)
            target_y = current_y + (y if y is not None else 0.0)
            target_z = current_z + (z if z is not None else 0.0)

        if (
            target_x is None
            or target_y is None
            or target_z is None
        ):
            current_x = target_x
            current_y = target_y
            current_z = target_z
            continue

        # ----------------------------------------------------
        # First coordinate = starting point
        # ----------------------------------------------------

        if not first_complete_position:

            output.append(
                f"G0 "
                f"X{fmt(target_x, 3)} "
                f"Y{fmt(target_y, 3)} "
                f"Z{fmt(target_z, 3)} "
                f"F{TRAVEL_FEEDRATE_MM_MIN}"
            )

            current_x = target_x
            current_y = target_y
            current_z = target_z

            first_complete_position = True
            previous_output_motion = "G0"

            continue

        # ----------------------------------------------------
        # Calculate full 3D non-planar distance
        # ----------------------------------------------------

        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        segment_length = math.sqrt(
            dx * dx
            + dy * dy
            + dz * dz
        )

        if segment_length <= 1e-12:

            current_x = target_x
            current_y = target_y
            current_z = target_z

            continue

        # ----------------------------------------------------
        # G1 = thermoplastic deposition
        # ----------------------------------------------------

        if motion_type in ("G1", "G01"):
            if f is None:
                feedrate = PRINT_FEEDRATE_MM_MIN
            else:
                feedrate = int(round(f))

            if non_depositing_mode == VERTICAL_ONLY:
                vertical_move = math.hypot(dx, dy) <= POSITION_TOLERANCE_MM
                approach_move = (
                    vertical_move
                    and dz < -POSITION_TOLERANCE_MM
                    and (
                        line_index in post_travel_g1_lines
                        or previous_output_motion == "G0"
                    )
                )
                exit_move = (
                    vertical_move
                    and dz > POSITION_TOLERANCE_MM
                    and line_index in pre_travel_g1_lines
                )
                non_depositing_move = approach_move or exit_move
            else:
                non_depositing_move = (
                    line_index in adjacent_g1_lines
                    or previous_output_motion == "G0"
                )
                exit_move = line_index in pre_travel_g1_lines

            if non_depositing_move:

                # If this is the material-free exit G1 immediately before
                # a G0 block, retract BEFORE moving away from the deposited
                # path. This prevents ooze during both the exit move and the
                # following travel moves.
                if (
                    ENABLE_TRAVEL_RETRACTION
                    and exit_move
                    and has_deposited
                    and not e_is_retracted
                ):

                    output.append(
                        make_e_only_move(
                            absolute_e - TRAVEL_RETRACTION_MM,
                            "PETG travel retract before exit/travel",
                        )
                    )

                    e_is_retracted = True

                output.append(
                    f"G1 "
                    f"X{fmt(target_x, 3)} "
                    f"Y{fmt(target_y, 3)} "
                    f"Z{fmt(target_z, 3)} "
                    f"F{feedrate}"
                )

            else:
                # All G0 moves and the first material-free engage G1 have
                # now finished. Restore nozzle pressure only at the actual
                # deposition start, immediately before the first G1 with E.
                if e_is_retracted:

                    output.append(
                        make_e_only_move(
                            absolute_e,
                            "PETG travel unretract at deposition start",
                        )
                    )

                    e_is_retracted = False

                delta_e = (
                    segment_length
                    * EXTRUSION_PER_PATH_MM
                )

                absolute_e += delta_e

                output.append(
                    f"G1 "
                    f"X{fmt(target_x, 3)} "
                    f"Y{fmt(target_y, 3)} "
                    f"Z{fmt(target_z, 3)} "
                    f"E{fmt(absolute_e, 5)} "
                    f"F{feedrate}"
                )

                total_path_length += segment_length
                total_filament += delta_e
                has_deposited = True

            previous_output_motion = "G1"

        # ----------------------------------------------------
        # G0 = travel only
        # ----------------------------------------------------

        else:

            if f is None:
                feedrate = TRAVEL_FEEDRATE_MM_MIN
            else:
                feedrate = int(round(f))

            # Some input files do not provide a material-free G1 before G0.
            # In that case, retract here, before the first G0 in the block.
            # The state flag prevents repeated retractions for consecutive G0s.
            if (
                ENABLE_TRAVEL_RETRACTION
                and has_deposited
                and not e_is_retracted
            ):

                output.append(
                    make_e_only_move(
                        absolute_e - TRAVEL_RETRACTION_MM,
                        "PETG travel retract before G0",
                    )
                )

                e_is_retracted = True

            output.append(
                f"G0 "
                f"X{fmt(target_x, 3)} "
                f"Y{fmt(target_y, 3)} "
                f"Z{fmt(target_z, 3)} "
                f"F{feedrate}"
            )

            previous_output_motion = "G0"

        current_x = target_x
        current_y = target_y
        current_z = target_z

    output.append("")
    output.append(
        f"; Total thermoplastic path = "
        f"{total_path_length:.3f} mm"
    )

    output.append(
        f"; Total filament E = "
        f"{total_filament:.5f} mm"
    )

    output.append(
        f"; Final deposited absolute E = "
        f"{absolute_e:.5f} mm"
    )

    if e_is_retracted:
        output.append(
            f"; Final commanded absolute E = "
            f"{absolute_e - TRAVEL_RETRACTION_MM:.5f} mm"
        )
        output.append(
            f"; Existing final PETG travel retract = "
            f"{TRAVEL_RETRACTION_MM:.3f} mm"
        )
    else:
        output.append(
            f"; Final commanded absolute E = "
            f"{absolute_e:.5f} mm"
        )
        output.append("; Existing final PETG travel retract = 0.000 mm")

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    output_path.write_text(
        "\n".join(output) + "\n",
        encoding="utf-8"
    )

    print("Finished.")
    print(
        f"E coefficient = "
        f"{EXTRUSION_PER_PATH_MM:.9f} mm/mm"
    )
    print(
        f"Path length = "
        f"{total_path_length:.3f} mm"
    )
    print(
        f"Filament = "
        f"{total_filament:.5f} mm"
    )
    print(
        f"Saved to:\n{output_path}"
    )


if __name__ == "__main__":
    for job_number, job in enumerate(CONVERSION_JOBS, start=1):
        print(
            f"\nConverting file {job_number}/{len(CONVERSION_JOBS)}:\n"
            f"{job.input_file.name}\n"
            f"Engage/retract mode: {job.non_depositing_mode}"
        )
        convert_motion_gcode(
            job.input_file,
            job.output_file,
            job.non_depositing_mode,
        )
