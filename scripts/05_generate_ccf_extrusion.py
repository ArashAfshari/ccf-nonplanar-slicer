#!/usr/bin/env python3
"""
Hardcoded Anisoprint CCF converter for NON-PLANAR motion G-code
==============================================================

This version DOES NOT read an Anisoprint reference G-code.

You only provide:
    1. MOTION_GCODE
    2. OUTPUT_GCODE

All CCF parameters are hardcoded below.

Main calculation for every non-planar XYZ segment:

    ds = sqrt(dx^2 + dy^2 + dz^2)

    dU = ds * FIBER_PER_MM

    dV = ds * MATRIX_PER_MM

For the current Anisoprint CCF setup:

    FIBER_PER_MM = 1.0
    MATRIX_PER_MM = 0.04182871600797469

Therefore:

    dU = ds
    dV = ds * 0.04182871600797469

U and V are written as cumulative absolute extrusion coordinates.

IMPORTANT
---------
- G0 = travel / ends a CCF strand.
- Consecutive G1 XYZ moves = one continuous CCF strand.
- First G1 point can establish the starting XYZ position.
- The first XYZ G1 after G0 is an engage move without U/V extrusion.
- The final XYZ G1 before G0 or end-of-file is a retract move without U/V extrusion.
- Cutter is triggered CUT_DISTANCE_MM before the end.
- After cutter activation, V continues but U no longer increases.
- After the final V retraction, the nozzle retraces the last part of the
  strand with slow G0 moves to iron/release the cut fiber before M1002.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from paths import ALIGNED_DIR, MODULES_DIR


# =============================================================================
# 1. FILE PATHS
# =============================================================================

# NOTE: the fiber track is taken from the second sample, whose loop allocation
# places the CCF where it is wanted, while the PETG modules in step 06 come
# from the first sample. Change CCF_SAMPLE to use a different sample's fiber.
CCF_SAMPLE = "ccf_source"

MOTION_GCODE = (
    ALIGNED_DIR
    / CCF_SAMPLE
    / "ccf_near_outer_boundary_translated_rotated_plus_90deg.gcode"
)
OUTPUT_GCODE = MODULES_DIR / "anisoprint_ccf.gcode"


# =============================================================================
# 2. ANISOPRINT CCF PARAMETERS
# =============================================================================

# -------------------------------------------------------------------------
# CCF geometry
# -------------------------------------------------------------------------

CCF_WIDTH_MM = 0.65
CCF_HEIGHT_MM = 0.32

FIBER_DIAMETER_MM = 0.35
PLASTIC_FILAMENT_DIAMETER_MM = 1.75

EXTRUSION_MULTIPLIER = 0.9


# -------------------------------------------------------------------------
# Fiber feed
# -------------------------------------------------------------------------

# 1 mm of nozzle path -> 1 mm continuous fiber.
FIBER_PER_MM = 1.0


# -------------------------------------------------------------------------
# Matrix filament calculation
# -------------------------------------------------------------------------
#
# Composite cross-section:
#
#     A_total = width * height
#
# Carbon fiber cross-section:
#
#     A_fiber = pi * d_fiber^2 / 4
#
# Matrix material area:
#
#     A_matrix = A_total - A_fiber
#
# Input PETG filament area:
#
#     A_filament = pi * d_filament^2 / 4
#
# Required V filament per 1 mm CCF path:
#
#     V/mm =
#         (A_matrix / A_filament)
#         * extrusion_multiplier
#
# For the hardcoded parameters this gives approximately:
#
#     0.041828716 mm V per mm path
#

FIBER_AREA_MM2 = math.pi * (FIBER_DIAMETER_MM ** 2) / 4.0

TOTAL_COMPOSITE_AREA_MM2 = CCF_WIDTH_MM * CCF_HEIGHT_MM

MATRIX_AREA_MM2 = TOTAL_COMPOSITE_AREA_MM2 - FIBER_AREA_MM2

PLASTIC_FILAMENT_AREA_MM2 = (
    math.pi * (PLASTIC_FILAMENT_DIAMETER_MM ** 2) / 4.0
)

MATRIX_PER_MM = (
    MATRIX_AREA_MM2
    / PLASTIC_FILAMENT_AREA_MM2
    * EXTRUSION_MULTIPLIER
)


# -------------------------------------------------------------------------
# Fiber cutter / restart parameters
# -------------------------------------------------------------------------

# Distance between cutter and effective nozzle exit.
CUT_DISTANCE_MM = 46.0

# Amount of U pushed before starting a new continuous strand.
FIBER_RESTART_LENGTH_MM = 47.0

# Slow-start length at beginning of CCF strand.
FIBER_START_LENGTH_MM = 5.0

# Distance retraced along the already deposited strand after the final
# V retraction. Aura uses this slow move to iron/release the cut fiber tail
# before ending CCF mode with M1002.
FINISH_IRONING_DISTANCE_MM = 20.0


# -------------------------------------------------------------------------
# Speeds
# -------------------------------------------------------------------------

# G-code F units are mm/min.

FIBER_RESTART_FEED = 600.0   # 10 mm/s
CCF_START_FEED = 240.0       # 4 mm/s
CCF_NORMAL_FEED = 300.0      # 5 mm/s
CCF_FINISH_FEED = 180.0      # 3 mm/s
FINISH_IRONING_FEED = 300.0  # 5 mm/s

TRAVEL_FEED = 7200.0


# -------------------------------------------------------------------------
# V retraction
# -------------------------------------------------------------------------

V_RETRACTION_MM = 9.0
V_RETRACT_FEED = 2100.0
CCF_START_V = 14.0


# -------------------------------------------------------------------------
# Cutter commands
# -------------------------------------------------------------------------

CUTTER_GCODE = [
    "M400",
    "M280 P0 S30",
    "G4 P200",
    "M280 P0 S90",
    "M400",
]


# -------------------------------------------------------------------------
# CCF mode commands
# -------------------------------------------------------------------------

CCF_START_COMMAND = "M1001"
CCF_END_COMMAND = "M1002"


# =============================================================================
# 3. CONVERTER OPTIONS
# =============================================================================

ENABLE_CUTTER = True
ENABLE_V_RETRACTION = True
ENABLE_FINISH_IRONING = True

# Your example consists only of XYZ G1 points.
# Therefore the first G1 is interpreted as starting position and extrusion
# begins from the second G1.
FIRST_G1_ESTABLISHES_POSITION = True

# Usually False.
# When False, the CCF feeds defined above are used.
PRESERVE_INPUT_FEED = False

XYZ_DECIMALS = 5
UV_DECIMALS = 5


# =============================================================================
# 4. DATA TYPES
# =============================================================================

@dataclass
class Point:
    x: float
    y: float
    z: float


@dataclass
class Vertex:
    point: Point
    feed: Optional[float] = None


# =============================================================================
# 5. BASIC G-CODE PARSING
# =============================================================================

NUMBER_RE = re.compile(
    r"([A-Za-z])"
    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
)


def strip_comment(line: str) -> str:
    return line.split(";", 1)[0].strip()


def parse_words(line: str) -> dict[str, float]:
    clean = strip_comment(line)

    result = {}

    for key, value in NUMBER_RE.findall(clean):
        result[key.upper()] = float(value)

    return result


def get_command(line: str) -> Optional[str]:
    clean = strip_comment(line)

    if not clean:
        return None

    first = clean.split()[0].upper()

    if first in ("G0", "G00"):
        return "G0"

    if first in ("G1", "G01"):
        return "G1"

    if first == "G90":
        return "G90"

    if first == "G91":
        return "G91"

    return first


# =============================================================================
# 6. GEOMETRY
# =============================================================================

def distance_3d(a: Point, b: Point) -> float:
    """
    TRUE non-planar distance.
    """

    dx = b.x - a.x
    dy = b.y - a.y
    dz = b.z - a.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


def interpolate(a: Point, b: Point, t: float) -> Point:
    return Point(
        x=a.x + (b.x - a.x) * t,
        y=a.y + (b.y - a.y) * t,
        z=a.z + (b.z - a.z) * t,
    )


def path_length(path: list[Vertex]) -> float:
    total = 0.0

    for i in range(1, len(path)):
        total += distance_3d(
            path[i - 1].point,
            path[i].point,
        )

    return total


def reverse_tail_points(
    path: list[Vertex],
    distance_mm: float,
) -> list[Point]:
    """
    Return points that retrace the final ``distance_mm`` of ``path``.

    The returned points start immediately before the final path point and
    continue backwards along the original non-planar XYZ geometry. The last
    point is interpolated when the requested distance ends inside a segment.
    """

    if distance_mm <= 1e-12 or len(path) < 2:
        return []

    remaining = min(
        distance_mm,
        path_length(path),
    )

    points: list[Point] = []

    for i in range(len(path) - 1, 0, -1):

        segment_end = path[i].point
        segment_start = path[i - 1].point

        segment_length = distance_3d(
            segment_start,
            segment_end,
        )

        if segment_length <= 1e-12:
            continue

        if remaining >= segment_length - 1e-10:

            points.append(segment_start)
            remaining -= segment_length

            if remaining <= 1e-10:
                break

        else:

            points.append(
                interpolate(
                    segment_end,
                    segment_start,
                    remaining / segment_length,
                )
            )

            remaining = 0.0
            break

    return points


# =============================================================================
# 7. READ MOTION G-CODE
# =============================================================================

def extract_paths(motion_file: Path) -> list[list[Vertex]]:
    """
    Consecutive G1 XYZ moves are treated as one CCF strand.

    G0 terminates the current strand.
    """

    lines = motion_file.read_text(
        encoding="utf-8",
        errors="ignore",
    ).splitlines()

    absolute_mode = True

    current_x = None
    current_y = None
    current_z = None

    paths: list[list[Vertex]] = []

    active_path: list[Vertex] = []

    def current_point() -> Optional[Point]:
        if (
            current_x is None
            or current_y is None
            or current_z is None
        ):
            return None

        return Point(
            float(current_x),
            float(current_y),
            float(current_z),
        )

    def close_path():
        nonlocal active_path

        if len(active_path) >= 2:

            cleaned = [active_path[0]]

            for vertex in active_path[1:]:

                if (
                    distance_3d(
                        cleaned[-1].point,
                        vertex.point,
                    )
                    > 1e-10
                ):
                    cleaned.append(vertex)

            if len(cleaned) >= 2:
                paths.append(cleaned)

        active_path = []

    for raw_line in lines:

        cmd = get_command(raw_line)
        values = parse_words(raw_line)

        if cmd == "G90":
            absolute_mode = True
            continue

        if cmd == "G91":
            absolute_mode = False
            continue

        if cmd not in ("G0", "G1"):
            continue

        has_xyz = any(
            axis in values
            for axis in ("X", "Y", "Z")
        )

        if not has_xyz:
            continue

        before = current_point()

        # -------------------------------------------------------------
        # Resolve new coordinates
        # -------------------------------------------------------------

        if "X" in values:
            if absolute_mode:
                current_x = values["X"]
            else:
                if current_x is None:
                    raise ValueError(
                        "Relative X encountered before X is known."
                    )

                current_x += values["X"]

        if "Y" in values:
            if absolute_mode:
                current_y = values["Y"]
            else:
                if current_y is None:
                    raise ValueError(
                        "Relative Y encountered before Y is known."
                    )

                current_y += values["Y"]

        if "Z" in values:
            if absolute_mode:
                current_z = values["Z"]
            else:
                if current_z is None:
                    raise ValueError(
                        "Relative Z encountered before Z is known."
                    )

                current_z += values["Z"]

        after = current_point()

        # -------------------------------------------------------------
        # G0 = travel
        # -------------------------------------------------------------

        if cmd == "G0":
            close_path()
            continue

        if after is None:
            continue

        feed = values.get("F")

        # -------------------------------------------------------------
        # New G1 path
        # -------------------------------------------------------------

        if not active_path:

            if before is not None:

                active_path.append(
                    Vertex(before)
                )

                active_path.append(
                    Vertex(after, feed)
                )

            elif FIRST_G1_ESTABLISHES_POSITION:

                active_path.append(
                    Vertex(after, feed)
                )

            else:

                raise ValueError(
                    "First G1 does not have a known XYZ starting point."
                )

        else:

            active_path.append(
                Vertex(after, feed)
            )

    close_path()

    return paths


# =============================================================================
# 8. CUT A PATH AT EXACT DISTANCES
# =============================================================================

def split_path(
    path: list[Vertex],
    split_distances: list[float],
):
    """
    Splits segments at precise cumulative path distances.

    This is needed because the 46 mm cutter location may fall
    inside a G1 segment.
    """

    total = path_length(path)

    split_distances = sorted(
        {
            d
            for d in split_distances
            if 0.0 < d < total
        }
    )

    output = []

    cumulative = 0.0
    split_index = 0

    for i in range(1, len(path)):

        p0 = path[i - 1].point
        p1 = path[i].point

        original_feed = path[i].feed

        segment_length = distance_3d(
            p0,
            p1,
        )

        if segment_length <= 1e-12:
            continue

        piece_start = p0
        piece_start_distance = cumulative

        while split_index < len(split_distances):

            split_distance = split_distances[
                split_index
            ]

            segment_end_distance = (
                cumulative
                + segment_length
            )

            if (
                split_distance
                >= segment_end_distance - 1e-10
            ):
                break

            if (
                split_distance
                <= cumulative + 1e-10
            ):
                split_index += 1
                continue

            t = (
                split_distance
                - cumulative
            ) / segment_length

            split_point = interpolate(
                p0,
                p1,
                t,
            )

            output.append(
                (
                    piece_start,
                    split_point,
                    piece_start_distance,
                    split_distance,
                    original_feed,
                )
            )

            piece_start = split_point
            piece_start_distance = split_distance

            split_index += 1

        output.append(
            (
                piece_start,
                p1,
                piece_start_distance,
                cumulative + segment_length,
                original_feed,
            )
        )

        cumulative += segment_length

    return output


# =============================================================================
# 9. FORMATTING
# =============================================================================

def fmt_xyz(value: float) -> str:
    return (
        f"{value:.{XYZ_DECIMALS}f}"
        .rstrip("0")
        .rstrip(".")
    )


def fmt_uv(value: float) -> str:
    return f"{value:.{UV_DECIMALS}f}"


def fmt_feed(value: float) -> str:
    return f"{value:.0f}"


def make_g0(
    point: Point,
    feed: float = TRAVEL_FEED,
) -> str:
    return (
        f"G0 "
        f"X{fmt_xyz(point.x)} "
        f"Y{fmt_xyz(point.y)} "
        f"Z{fmt_xyz(point.z)} "
        f"F{fmt_feed(feed)}"
    )


def make_g1(
    point: Point,
    v: float,
    feed: float,
    u: Optional[float] = None,
) -> str:

    line = (
        f"G1 "
        f"X{fmt_xyz(point.x)} "
        f"Y{fmt_xyz(point.y)} "
        f"Z{fmt_xyz(point.z)} "
        f"V{fmt_uv(v)}"
    )

    if u is not None:
        line += f" U{fmt_uv(u)}"

    line += f" F{fmt_feed(feed)}"

    return line


def make_g1_without_extrusion(
    point: Point,
    feed: float,
) -> str:
    """Create an XYZ G1 engage/retract move without U or V."""

    return (
        f"G1 "
        f"X{fmt_xyz(point.x)} "
        f"Y{fmt_xyz(point.y)} "
        f"Z{fmt_xyz(point.z)} "
        f"F{fmt_feed(feed)}"
    )


# =============================================================================
# 10. CCF CONVERSION
# =============================================================================

def convert_to_ccf(
    paths: list[list[Vertex]],
) -> list[str]:

    if not paths:
        raise ValueError(
            "No usable G1 XYZ paths were found."
        )

    output = []

    # -----------------------------------------------------------------
    # Header
    # -----------------------------------------------------------------

    output.extend(
        [
            "; ============================================================",
            "; NON-PLANAR ANISOPRINT CCF G-CODE",
            "; Generated with hardcoded CCF parameters",
            ";",
            "; ds = sqrt(dx^2 + dy^2 + dz^2)",
            f"; dU = ds * {FIBER_PER_MM:.12f}",
            f"; dV = ds * {MATRIX_PER_MM:.12f}",
            ";",
            f"; CCF width             = {CCF_WIDTH_MM} mm",
            f"; CCF height            = {CCF_HEIGHT_MM} mm",
            f"; Fiber diameter        = {FIBER_DIAMETER_MM} mm",
            f"; Matrix filament dia.  = {PLASTIC_FILAMENT_DIAMETER_MM} mm",
            f"; Extrusion multiplier  = {EXTRUSION_MULTIPLIER}",
            f"; Cut distance          = {CUT_DISTANCE_MM} mm",
            f"; Fiber restart         = {FIBER_RESTART_LENGTH_MM} mm",
            f"; Finish ironing        = {FINISH_IRONING_DISTANCE_MM} mm",
            "; First/last XYZ G1 of each strand = motion only, no U/V",
            "; ============================================================",
            "",
            "G90",
            "M82",
            "G92 U0",
            "G92 V0",
        ]
    )

    # Absolute U/V coordinates.
    U = 0.0
    V = 0.0

    v_is_retracted = False

    # -----------------------------------------------------------------
    # Process each continuous strand
    # -----------------------------------------------------------------

    for path_number, path in enumerate(
        paths,
        start=1,
    ):

        total_motion_length = path_length(path)

        if total_motion_length <= 1e-10:
            continue

        if len(path) < 4:
            raise ValueError(
                f"Path {path_number} must contain at least three XYZ "
                "segments: engage, deposition, and retract."
            )

        # The first segment is the engaging G1 immediately after G0.
        # The final segment is the retracting G1 before G0/end-of-file.
        # Neither segment deposits continuous fiber or matrix material.
        engage_length = distance_3d(
            path[0].point,
            path[1].point,
        )

        retract_length = distance_3d(
            path[-2].point,
            path[-1].point,
        )

        deposition_start_distance = engage_length
        deposition_end_distance = (
            total_motion_length
            - retract_length
        )

        deposition_length = (
            deposition_end_distance
            - deposition_start_distance
        )

        if deposition_length <= 1e-10:
            raise ValueError(
                f"Path {path_number} has no depositing CCF distance "
                "after excluding its engage and retract moves."
            )

        if (
            ENABLE_CUTTER
            and deposition_length
            <= CUT_DISTANCE_MM
        ):
            raise ValueError(
                f"Path {path_number} deposition length is "
                f"{deposition_length:.3f} mm, but the cutter "
                f"distance is {CUT_DISTANCE_MM:.3f} mm.\n"
                f"Disable ENABLE_CUTTER for short test paths."
            )

        # -------------------------------------------------------------
        # Cutter occurs 46 mm before geometric endpoint
        # -------------------------------------------------------------

        if ENABLE_CUTTER:

            cutter_distance = (
                deposition_end_distance
                - CUT_DISTANCE_MM
            )

        else:

            cutter_distance = deposition_end_distance

        # We split at:
        # 1. end of slow start
        # 2. exact cutter location
        split_positions = [
            deposition_start_distance,
            deposition_start_distance
            + min(
                FIBER_START_LENGTH_MM,
                deposition_length,
            ),
            deposition_end_distance,
        ]

        if ENABLE_CUTTER:
            split_positions.append(
                cutter_distance
            )

        pieces = split_path(
            path,
            split_positions,
        )

        # -------------------------------------------------------------
        # Strand start
        # -------------------------------------------------------------

        output.extend(
            [
                "",
                "; ------------------------------------------------------------",
                f"; CCF STRAND {path_number}",
                f"; TOTAL XYZ LENGTH = {total_motion_length:.5f} mm",
                f"; ENGAGE LENGTH = {engage_length:.5f} mm (no U/V)",
                f"; CCF DEPOSITION LENGTH = {deposition_length:.5f} mm",
                f"; RETRACT LENGTH = {retract_length:.5f} mm (no U/V)",
                "; ------------------------------------------------------------",
            ]
        )

        # Aura style.
        output.append(
            f"{CCF_START_COMMAND} "
            f"L{int(math.floor(deposition_length))}"
        )

        # -------------------------------------------------------------
        # Fiber restart
        # -------------------------------------------------------------

        U += FIBER_RESTART_LENGTH_MM

        output.append(
            f"G1 "
            f"F{fmt_feed(FIBER_RESTART_FEED)} "
            f"U{fmt_uv(U)}"
        )

        # -------------------------------------------------------------
        # Move to CCF start point
        # -------------------------------------------------------------

        start_point = path[0].point

        output.append(
            make_g0(start_point)
        )

        # -------------------------------------------------------------
        # Start / restore matrix filament before CCF deposition
        # Required Anisoprint sequence:
        #   M1001 L...
        #   G1 F600 U...
        #   G0 X... Y... Z...
        #   G1 F2100 V14
        #   G4 P0
        # -------------------------------------------------------------

        V = CCF_START_V

        output.append(
            f"G1 "
            f"F{fmt_feed(V_RETRACT_FEED)} "
            f"V{fmt_xyz(CCF_START_V)}"
        )

        v_is_retracted = False

        output.append("G4 P0")

        cutter_done = False

        # -------------------------------------------------------------
        # Deposition
        # -------------------------------------------------------------

        for (
            p0,
            p1,
            distance_start,
            distance_end,
            input_feed,
        ) in pieces:

            ds = distance_3d(
                p0,
                p1,
            )

            if ds <= 1e-12:
                continue

            # ---------------------------------------------------------
            # Engage/retract XYZ motion only: do not advance U or V.
            # The exact deposition boundaries are included in the split
            # positions, so a long engage/retract segment is fully skipped.
            # ---------------------------------------------------------

            engage_move = (
                distance_end
                <= deposition_start_distance + 1e-8
            )

            retract_move = (
                distance_start
                >= deposition_end_distance - 1e-8
            )

            if engage_move or retract_move:

                if (
                    PRESERVE_INPUT_FEED
                    and input_feed is not None
                ):
                    feed = input_feed
                elif engage_move:
                    feed = CCF_START_FEED
                else:
                    feed = CCF_FINISH_FEED

                output.append(
                    make_g1_without_extrusion(
                        point=p1,
                        feed=feed,
                    )
                )

                continue

            # ---------------------------------------------------------
            # Activate cutter exactly before post-cut section
            # ---------------------------------------------------------

            if (
                ENABLE_CUTTER
                and not cutter_done
                and distance_start
                >= cutter_distance - 1e-8
            ):

                output.append(
                    "; ===== CUT CONTINUOUS FIBER ====="
                )

                output.extend(
                    CUTTER_GCODE
                )

                cutter_done = True

            # ---------------------------------------------------------
            # V calculation
            # ---------------------------------------------------------

            dV = (
                ds
                * MATRIX_PER_MM
            )

            V += dV

            # ---------------------------------------------------------
            # U calculation
            # ---------------------------------------------------------

            pre_cut = (
                not ENABLE_CUTTER
                or distance_end
                <= cutter_distance + 1e-8
            )

            if pre_cut:

                dU = (
                    ds
                    * FIBER_PER_MM
                )

                U += dU

                output_U = U

            else:

                # Fiber has already been cut.
                # Existing residual fiber between cutter and nozzle
                # is deposited, therefore U does not advance.
                output_U = None

            # ---------------------------------------------------------
            # Feed rate
            # ---------------------------------------------------------

            if (
                PRESERVE_INPUT_FEED
                and input_feed is not None
            ):

                feed = input_feed

            else:

                if (
                    ENABLE_CUTTER
                    and distance_start
                    >= cutter_distance - 1e-8
                ):

                    feed = CCF_FINISH_FEED

                elif (
                    distance_start
                    < deposition_start_distance
                    + FIBER_START_LENGTH_MM
                    - 1e-8
                ):

                    feed = CCF_START_FEED

                else:

                    feed = CCF_NORMAL_FEED

            output.append(
                make_g1(
                    point=p1,
                    u=output_U,
                    v=V,
                    feed=feed,
                )
            )

            # ---------------------------------------------------------
            # Cutter exactly after segment ending at cutter position
            # ---------------------------------------------------------

            if (
                ENABLE_CUTTER
                and not cutter_done
                and abs(
                    distance_end
                    - cutter_distance
                )
                <= 1e-7
            ):

                output.append(
                    "; ===== CUT CONTINUOUS FIBER ====="
                )

                output.extend(
                    CUTTER_GCODE
                )

                cutter_done = True

        # -------------------------------------------------------------
        # End-of-strand V retraction
        # -------------------------------------------------------------

        if ENABLE_V_RETRACTION:

            V -= V_RETRACTION_MM

            output.append(
                f"G1 "
                f"F{fmt_feed(V_RETRACT_FEED)} "
                f"V{fmt_uv(V)} "
                f"; V retract"
            )

            v_is_retracted = True

        # -------------------------------------------------------------
        # Aura-style finish ironing / fiber-tail release
        # -------------------------------------------------------------
        # Retrace the final part of the strand only after V has been
        # retracted. Using the original points in reverse keeps the slow
        # G0 move on the deposited non-planar path instead of inventing a
        # straight move that could cross empty space or the printed part.

        if ENABLE_FINISH_IRONING:

            if not ENABLE_V_RETRACTION:
                raise ValueError(
                    "ENABLE_FINISH_IRONING requires "
                    "ENABLE_V_RETRACTION so the ironing move occurs "
                    "after the final V retract."
                )

            ironing_points = reverse_tail_points(
                path,
                FINISH_IRONING_DISTANCE_MM,
            )

            if ironing_points:

                output.append(
                    "; ===== FINISH IRON / RELEASE CCF TAIL ====="
                )

                for ironing_point in ironing_points:

                    output.append(
                        make_g0(
                            point=ironing_point,
                            feed=FINISH_IRONING_FEED,
                        )
                    )

        output.append(
            CCF_END_COMMAND
        )

        output.append(
            f"; END CCF STRAND {path_number}"
        )

    return output


# =============================================================================
# 11. PRINT PARAMETER CHECK
# =============================================================================

def print_parameters():

    print()
    print("========================================")
    print("HARDCODED CCF PARAMETERS")
    print("========================================")

    print(
        f"CCF cross-section: "
        f"{CCF_WIDTH_MM} x {CCF_HEIGHT_MM} mm"
    )

    print(
        f"Fiber diameter: "
        f"{FIBER_DIAMETER_MM} mm"
    )

    print(
        f"Fiber area: "
        f"{FIBER_AREA_MM2:.9f} mm^2"
    )

    print(
        f"Total CCF area: "
        f"{TOTAL_COMPOSITE_AREA_MM2:.9f} mm^2"
    )

    print(
        f"Matrix area: "
        f"{MATRIX_AREA_MM2:.9f} mm^2"
    )

    print(
        f"1.75 mm filament area: "
        f"{PLASTIC_FILAMENT_AREA_MM2:.9f} mm^2"
    )

    print()

    print(
        f"U per path mm = "
        f"{FIBER_PER_MM:.12f}"
    )

    print(
        f"V per path mm = "
        f"{MATRIX_PER_MM:.12f}"
    )

    print()

    print(
        f"Cut distance = "
        f"{CUT_DISTANCE_MM} mm"
    )

    print(
        f"Fiber restart = "
        f"{FIBER_RESTART_LENGTH_MM} mm"
    )

    print(
        f"Finish ironing distance = "
        f"{FINISH_IRONING_DISTANCE_MM} mm"
    )

    print("========================================")
    print()


# =============================================================================
# 12. MAIN
# =============================================================================

def main():

    motion_file = Path(
        MOTION_GCODE
    )

    output_file = Path(
        OUTPUT_GCODE
    )

    if not motion_file.exists():

        print()
        print("ERROR:")
        print(
            f"Motion G-code not found:\n"
            f"{motion_file}"
        )

        print()
        print(
            "Edit MOTION_GCODE at the top "
            "of this Python file."
        )

        return

    print_parameters()

    paths = extract_paths(
        motion_file
    )

    print(
        f"Continuous CCF paths found: "
        f"{len(paths)}"
    )

    for i, path in enumerate(
        paths,
        start=1,
    ):

        length = path_length(path)

        print(
            f"  Path {i}: "
            f"{length:.5f} mm"
        )

    converted = convert_to_ccf(
        paths
    )

    output_file.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_file.write_text(
        "\n".join(converted) + "\n",
        encoding="utf-8",
    )

    print()
    print(
        "Conversion finished."
    )

    print(
        f"Output:\n"
        f"{output_file}"
    )


if __name__ == "__main__":
    main()
