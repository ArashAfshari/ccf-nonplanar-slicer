"""Step 07 - stack the single-layer modules into a complete multilayer job.

Takes the four aligned single-layer modules from steps 05 and 06 and builds one
printable Anisoprint job: solid PETG end layers at the bottom and top, and
CCF-reinforced layers in between, with tool changes, temperatures, fans and
the Anisoprint start/end code.
"""

from pathlib import Path
import math
import re

from paths import MODULES_DIR, JOBS_DIR

# ============================================================
# EDIT THESE SETTINGS
# ============================================================
# Four already aligned, single-layer G-codes from steps 05 and 06. The script
# preserves their XYZ geometry. FULL_PETG_GCODE represents layer 1 and is
# translated in Z for the second, penultimate and final layers.
FULL_PETG_GCODE = MODULES_DIR / "anisoprint_petg_full_surfaces.gcode"
PETG_OUTER_GCODE = MODULES_DIR / "anisoprint_petg_outer_boundary.gcode"
CCF_GCODE = MODULES_DIR / "anisoprint_ccf.gcode"
PETG_INNER_GCODE = MODULES_DIR / "anisoprint_petg_inner.gcode"
OUTPUT_GCODE = JOBS_DIR / "ccf_petg_multilayer.gcode"

NUMBER_OF_LAYERS = 13        # Parametric total layer count, minimum 5
PETG_ONLY_END_LAYERS = 2     # Full-PETG layers at both bottom and top
LAYER_HEIGHT = 0.32          # mm. Layer n gets Z offset (n-1)*LAYER_HEIGHT

# ============================================================
# ANISOPRINT SETTINGS
# Based on the existing one-layer integration script/reference
# ============================================================
T0_PRINT_TEMP = 240          # PETG / thermoplastic tool
T0_PREHEAT_TEMP = 180
T0_WAIT_TEMP = 180

T1_PRINT_TEMP = 240          # CCF / composite tool
T1_PREHEAT_TEMP = 180
T1_WAIT_TEMP = 180

BED_TEMP = 60
CCF_FAN_PWM = 127
PETG_FAN_PWM = 76

# Apply the requested 30% reduction only to material-depositing motions.
# Rapid positioning, Z travel, retraction, priming and tool-change feeds keep
# their original values.
PRINT_FEED_SCALE = 0.70

# Restart the continuous fiber only after the tool has moved close to the
# CCF engage point. Aura leaves about 0.707 mm of material-free engage travel
# after the U restart, then restores the matrix V coordinate.
CCF_RESTART_APPROACH_DISTANCE = 0.707

CCF_FIBER_RESTART = 47.0

# One sacrificial Aura-style CCF preparation line is printed before the model.
# Its state is carried into the first mixed layer, so the model restores V
# instead of performing the stationary V14 prime on the part.
CCF_PREP_X = 73.0
CCF_PREP_Y = 0.0
CCF_PREP_Z = 0.175
CCF_PREP_SAFE_Z = 0.675
CCF_PREP_LENGTH = 150.0
CCF_PREP_FINAL_U = 152.6
CCF_PREP_DEPOSITED_V = 17.58904
CCF_PREP_CURRENT_V = 8.58904

TOOLCHANGE_Z_LIFT = 5.0
TOOLCHANGE_RETRACT = 12.0
PETG_INTERLAYER_RETRACT = 9.0
RETRACT_FEED = 2100
TRAVEL_Z_FEED = 1500

# The reference keeps U absolute across CCF layers and resets it only after
# it has grown to about 10,000 mm. Set to None to disable this safeguard.
U_RESET_THRESHOLD = 10000.0

# Composer A4 nominal printable XY range from the reference profile.
BED_X_MIN, BED_X_MAX = 0.0, 297.0
BED_Y_MIN, BED_Y_MAX = 0.0, 210.0

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
AXIS_RE = re.compile(rf"(?<![A-Za-z])([XYZEUVI])({NUMBER})", re.IGNORECASE)
Z_RE = re.compile(rf"(?<![A-Za-z])Z({NUMBER})", re.IGNORECASE)
FEED_RE = re.compile(rf"(?<![A-Za-z])F({NUMBER})", re.IGNORECASE)


def code_only(line: str) -> str:
    """Return the G-code part before ';' comments."""
    return line.split(";", 1)[0].strip()


def axis_values(line: str) -> dict[str, float]:
    """Extract XYZ/E/U/V/I values from one G-code line."""
    code = code_only(line)
    return {m.group(1).upper(): float(m.group(2)) for m in AXIS_RE.finditer(code)}


def command(line: str) -> str:
    code = code_only(line)
    if not code:
        return ""
    return code.split()[0].upper()


def fmt(value: float, decimals: int = 5) -> str:
    text = f"{value:.{decimals}f}"
    return text.rstrip("0").rstrip(".") if "." in text else text


def read_lines(path: Path) -> list[str]:
    if not path.is_file():
        raise FileNotFoundError(f"Input file not found: {path}")
    return path.read_text(encoding="utf-8", errors="ignore").splitlines()


def is_full_petg_layer(layer_number: int, total_layers: int) -> bool:
    """Return True for the configured PETG-only bottom and top skin layers."""
    return (
        layer_number <= PETG_ONLY_END_LAYERS
        or layer_number > total_layers - PETG_ONLY_END_LAYERS
    )


def strip_source_mode_setup(lines: list[str], axes_to_strip: set[str]) -> list[str]:
    """
    Remove source-local G90/M82/G92 setup commands because the integrated
    file establishes absolute XYZ/extrusion modes globally.

    Actual path commands, M1001/M1002, cutting commands, U/V/E values,
    and comments are retained.
    """
    cleaned = []
    for line in lines:
        c = code_only(line).upper()

        if c in {"G90", "M82"}:
            continue

        if c.startswith("G92"):
            vals = axis_values(line)
            if vals and set(vals).issubset(axes_to_strip):
                continue

        cleaned.append(line)

    return cleaned


def shift_z_in_line(line: str, z_offset: float) -> str:
    """
    Add z_offset to Z coordinates in motion commands only.

    X/Y, E, U, V, feed rate, M-codes, comments, etc. are unchanged.
    Because the integrated file uses G90, this produces an exact vertical
    translation of the original non-planar geometry.
    """
    if abs(z_offset) < 1e-12:
        return line

    code, sep, comment = line.partition(";")
    cmd = code.strip().split(maxsplit=1)[0].upper() if code.strip() else ""

    # Only actual motion coordinates are translated.
    if cmd not in {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}:
        return line

    def repl(match: re.Match) -> str:
        old_z = float(match.group(1))
        return f"Z{fmt(old_z + z_offset, 5)}"

    shifted_code = Z_RE.sub(repl, code)
    return shifted_code + (sep + comment if sep else "")


def shift_layer_z(lines: list[str], z_offset: float) -> list[str]:
    return [shift_z_in_line(line, z_offset) for line in lines]


def offset_axis_in_line(line: str, axis: str, offset: float) -> str:
    """Add an offset to an absolute E/U/V coordinate in motion commands."""
    if abs(offset) < 1e-12:
        return line

    code, sep, comment = line.partition(";")
    cmd = code.strip().split(maxsplit=1)[0].upper() if code.strip() else ""
    if cmd not in {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}:
        return line

    pattern = re.compile(rf"(?<![A-Za-z])({axis})({NUMBER})", re.IGNORECASE)

    def repl(match: re.Match) -> str:
        value = float(match.group(2)) + offset
        return f"{match.group(1)}{fmt(value, 5)}"

    shifted_code = pattern.sub(repl, code)
    return shifted_code + (sep + comment if sep else "")


def transform_layer(
    lines: list[str],
    z_offset: float,
    e_offset: float = 0.0,
    u_offset: float = 0.0,
    v_offset: float = 0.0,
) -> list[str]:
    """
    Translate one source layer in Z and shift absolute extrusion coordinates.

    The source one-layer files use absolute extrusion coordinates. Repeated
    layers therefore must continue E/U/V from the previous layer instead of
    replaying the same absolute values from zero.
    """
    out = []
    for line in lines:
        line = shift_z_in_line(line, z_offset)
        line = offset_axis_in_line(line, "E", e_offset)
        line = offset_axis_in_line(line, "U", u_offset)
        line = offset_axis_in_line(line, "V", v_offset)
        out.append(line)
    return out


def set_feed_rate(line: str, feed: float) -> str:
    """Replace or add an F value while preserving the line comment."""
    code, sep, comment = line.partition(";")
    new_feed = f"F{fmt(feed, 3)}"

    if FEED_RE.search(code):
        code = FEED_RE.sub(new_feed, code, count=1)
    else:
        code = code.rstrip() + " " + new_feed

    return code + (sep + comment if sep else "")


def scale_deposition_feed_rates(
    lines: list[str],
    extrusion_axes: set[str],
    scale: float,
) -> tuple[list[str], int]:
    """
    Scale only material-depositing G0/G1/G2/G3 motions.

    A motion is considered depositing when it contains XYZ movement and at
    least one requested absolute extrusion coordinate (E for PETG, U/V for
    CCF) increases. Retractions, primes/restores without XYZ, tool changes and
    positioning feeds are deliberately left unchanged.
    """
    if not 0 < scale <= 1:
        raise ValueError("Deposition feed scale must be greater than 0 and at most 1.")

    axes = {axis.upper() for axis in extrusion_axes}
    unsupported = axes - {"E", "U", "V"}
    if unsupported:
        raise ValueError(f"Unsupported extrusion axes: {sorted(unsupported)}")

    output: list[str] = []
    positions: dict[str, float | None] = {axis: None for axis in axes}
    modal_source_feed: float | None = None
    scaled_moves = 0
    motion_commands = {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}

    for line in lines:
        source_feed_match = FEED_RE.search(code_only(line))
        explicit_source_feed = (
            float(source_feed_match.group(1)) if source_feed_match else None
        )
        if explicit_source_feed is not None:
            modal_source_feed = explicit_source_feed

        cmd = command(line)
        vals = axis_values(line)

        if cmd == "G92":
            for axis in axes:
                if axis in vals:
                    positions[axis] = vals[axis]
            output.append(line)
            continue

        depositing = False
        if cmd in motion_commands and any(axis in vals for axis in ("X", "Y", "Z")):
            for axis in axes:
                if axis not in vals:
                    continue
                previous = positions[axis]
                if previous is None:
                    depositing = vals[axis] > 1e-9
                elif vals[axis] > previous + 1e-9:
                    depositing = True

        if depositing:
            if modal_source_feed is None:
                raise ValueError(
                    "A depositing motion has no explicit or inherited source feed: "
                    f"{line}"
                )
            line = set_feed_rate(line, modal_source_feed * scale)
            scaled_moves += 1

        if cmd in motion_commands:
            for axis in axes:
                if axis in vals:
                    positions[axis] = vals[axis]

        output.append(line)

    return output, scaled_moves


def replace_xyz_in_motion(line: str, xyz: dict[str, float]) -> str:
    """Replace XYZ values in a motion line while preserving F and comments."""
    code, sep, comment = line.partition(";")

    for axis in ("X", "Y", "Z"):
        if axis not in xyz:
            continue
        pattern = re.compile(
            rf"(?<![A-Za-z])({axis})({NUMBER})",
            re.IGNORECASE,
        )
        code, replacements = pattern.subn(
            lambda match: f"{match.group(1)}{fmt(xyz[axis], 5)}",
            code,
            count=1,
        )
        if replacements != 1:
            raise ValueError(
                f"Could not replace {axis} in CCF positioning line: {line}"
            )

    return code + (sep + comment if sep else "")


def arrange_ccf_start_sequence(lines: list[str]) -> list[str]:
    """
    Put every CCF strand start in the safe Aura-style order:

        position close to engage point -> M1001 -> U restart ->
        material-free engage -> V restore/prime -> dwell -> deposition

    The source CCF file is expected to contain this original sequence:

        M1001 -> U-only restart -> G0 XYZ -> V-only restore/prime ->
        optional non-motion commands -> material-free G1 XYZ engage

    Its final G0 position is moved along the existing engage segment so only
    CCF_RESTART_APPROACH_DISTANCE remains after the U restart. The engage G1
    remains material-free: no U or V is added to it.
    """
    result = list(lines)
    search_from = 0

    while True:
        try:
            m1001_index = next(
                i
                for i in range(search_from, len(result))
                if command(result[i]) == "M1001"
            )
        except StopIteration:
            break

        def next_command_index(start: int) -> int:
            for i in range(start, len(result)):
                if command(result[i]):
                    return i
            raise ValueError("Incomplete CCF start sequence after M1001.")

        u_index = next_command_index(m1001_index + 1)
        u_vals = axis_values(result[u_index])
        if (
            command(result[u_index]) not in {"G0", "G00", "G1", "G01"}
            or "U" not in u_vals
            or any(axis in u_vals for axis in ("X", "Y", "Z", "E", "V"))
        ):
            raise ValueError(
                "Expected a U-only continuous-fiber restart immediately "
                f"after M1001, found: {result[u_index]}"
            )

        # Idempotency: previously arranged sources already have the
        # material-free G1 engage immediately after U. Validate that canonical
        # order and leave it unchanged.
        position_index = next_command_index(u_index + 1)
        position_vals = axis_values(result[position_index])
        already_arranged = (
            command(result[position_index]) in {"G1", "G01"}
            and all(axis in position_vals for axis in ("X", "Y", "Z"))
            and not any(axis in position_vals for axis in ("E", "U", "V"))
        )
        if already_arranged:
            v_index = next_command_index(position_index + 1)
            v_vals = axis_values(result[v_index])
            if (
                command(result[v_index]) not in {"G0", "G00", "G1", "G01"}
                or "V" not in v_vals
                or any(axis in v_vals for axis in ("X", "Y", "Z", "E", "U"))
            ):
                raise ValueError(
                    "An already-arranged CCF engage must be followed by a "
                    f"V-only restore/prime, found: {result[v_index]}"
                )

            previous_position = None
            for previous_index in range(m1001_index - 1, -1, -1):
                previous_vals = axis_values(result[previous_index])
                if (
                    command(result[previous_index])
                    in {"G0", "G00", "G1", "G01"}
                    and all(axis in previous_vals for axis in ("X", "Y", "Z"))
                ):
                    previous_position = previous_vals
                    break
            if previous_position is None:
                raise ValueError(
                    "Could not find CCF positioning before an arranged M1001."
                )
            engage_distance = math.sqrt(sum(
                (previous_position[axis] - position_vals[axis]) ** 2
                for axis in ("X", "Y", "Z")
            ))
            if abs(engage_distance - CCF_RESTART_APPROACH_DISTANCE) > 0.002:
                raise ValueError(
                    "An arranged CCF start does not preserve the required "
                    f"0.707 mm engage distance (found {engage_distance:.5f} mm)."
                )
            search_from = v_index + 1
            continue

        if (
            command(result[position_index]) not in {"G0", "G00"}
            or not all(axis in position_vals for axis in ("X", "Y", "Z"))
            or any(axis in position_vals for axis in ("E", "U", "V"))
        ):
            raise ValueError(
                "Expected a material-free XYZ G0 after the U restart, found: "
                f"{result[position_index]}"
            )

        v_index = next_command_index(position_index + 1)
        v_vals = axis_values(result[v_index])
        if (
            command(result[v_index]) not in {"G0", "G00", "G1", "G01"}
            or "V" not in v_vals
            or any(axis in v_vals for axis in ("X", "Y", "Z", "E", "U"))
        ):
            raise ValueError(
                "Expected a V-only matrix restore/prime after CCF positioning, "
                f"found: {result[v_index]}"
            )

        engage_index = v_index + 1
        while engage_index < len(result):
            engage_cmd = command(result[engage_index])
            engage_vals = axis_values(result[engage_index])
            if engage_cmd in {"G0", "G00", "G1", "G01"} and any(
                axis in engage_vals for axis in ("X", "Y", "Z")
            ):
                break
            if engage_cmd == "M1002":
                raise ValueError(
                    "Could not find a material-free CCF engage motion after "
                    "the V restore/prime."
                )
            engage_index += 1

        if engage_index >= len(result):
            raise ValueError(
                "Could not find a material-free CCF engage motion after the "
                "V restore/prime."
            )

        engage_vals = axis_values(result[engage_index])
        if (
            command(result[engage_index]) not in {"G1", "G01"}
            or not all(axis in engage_vals for axis in ("X", "Y", "Z"))
            or any(axis in engage_vals for axis in ("E", "U", "V"))
        ):
            raise ValueError(
                "The first CCF XYZ motion after the V restore must be a "
                f"material-free G1 engage move, found: {result[engage_index]}"
            )

        dx = position_vals["X"] - engage_vals["X"]
        dy = position_vals["Y"] - engage_vals["Y"]
        dz = position_vals["Z"] - engage_vals["Z"]
        engage_distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        if engage_distance <= 1e-9:
            raise ValueError("The CCF engage move has zero XYZ length.")

        remaining = min(CCF_RESTART_APPROACH_DISTANCE, engage_distance)
        scale = remaining / engage_distance
        near_engage_xyz = {
            "X": engage_vals["X"] + dx * scale,
            "Y": engage_vals["Y"] + dy * scale,
            "Z": engage_vals["Z"] + dz * scale,
        }
        near_position_line = replace_xyz_in_motion(
            result[position_index],
            near_engage_xyz,
        )

        between_m1001_and_u = result[m1001_index + 1 : u_index]
        between_u_and_position = result[u_index + 1 : position_index]
        between_position_and_v = result[position_index + 1 : v_index]
        between_v_and_engage = result[v_index + 1 : engage_index]

        replacement = [
            "; Position near CCF engage point before restarting fiber",
            *between_u_and_position,
            result[position_index],
            near_position_line,
            result[m1001_index],
            *between_m1001_and_u,
            result[u_index],
            *between_position_and_v,
            result[engage_index],
            "; Restore/prime CCF matrix only after material-free engage",
            result[v_index],
            *between_v_and_engage,
        ]

        result[m1001_index : engage_index + 1] = replacement
        search_from = m1001_index + len(replacement)

    return result


def insert_petg_restore_before_deposition(
    lines: list[str],
    restore_e_to: float,
) -> list[str]:
    """
    Restore T0 pressure immediately before a PETG block starts depositing.

    T0 remains retracted while the block performs its leading positioning and
    material-free engage moves. Recovery is inserted immediately before the
    first motion containing both XYZ and E, so unretraction cannot ooze during
    positioning. This is used after a tool change and between repeated layers.
    """
    for index, line in enumerate(lines):
        vals = axis_values(line)
        cmd = command(line)

        if "E" not in vals:
            continue

        is_motion = cmd in {
            "G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"
        }
        has_xyz = any(axis in vals for axis in ("X", "Y", "Z"))

        if is_motion and has_xyz:
            restore = [
                (
                    "; Restore T0 at PETG deposition start to previous "
                    f"deposited absolute E = {fmt(restore_e_to)}"
                ),
                f"G1 F{RETRACT_FEED} E{fmt(restore_e_to)}",
            ]
            return lines[:index] + restore + lines[index:]

        if is_motion:
            raise ValueError(
                "The PETG source contains an E-only motion before its first "
                "XYZ+E deposition move. Pressure-recovery placement would "
                "be ambiguous; remove that leading E-only motion from the "
                "single-layer PETG source."
            )

    raise ValueError(
        "Could not find an XYZ+E deposition move in the PETG block for "
        "T0 pressure recovery."
    )


def last_m1002_index(lines: list[str]) -> int:
    indices = [i for i, line in enumerate(lines) if command(line) == "M1002"]
    if not indices:
        raise ValueError(
            "CCF file contains no M1002. The CCF process must be terminated "
            "before changing from T1 to T0."
        )
    return indices[-1]


def analyze_ccf(lines: list[str]) -> tuple[float, float, float, float, float, float]:
    """
    Return:
        last_z,
        first_v_prime,
        deposited_v_before_final_retract,
        current_v_after_source_retract,
        toolchange_target_v,
        final_u

    In the Anisoprint reference V is absolute across CCF layers. The first
    CCF use contains a matrix prime (normally V14). Later CCF layers restore
    V to the previous deposited value and continue from there, so the V14
    prime must not be added again as fresh material on every repeated layer.

    U is also absolute across CCF layers (apart from occasional G92 U0 resets).
    """
    stop = last_m1002_index(lines)
    prefix = lines[: stop + 1]

    last_z = None
    v_events = []
    u_events = []

    for i, line in enumerate(prefix):
        vals = axis_values(line)
        cmd = command(line)
        if "Z" in vals:
            last_z = vals["Z"]
        # Ignore G92 coordinate resets when determining the source CCF prime
        # and path values. We need the first actual motion V (normally V14).
        if "V" in vals and cmd in {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}:
            has_xyz = any(a in vals for a in ("X", "Y", "Z"))
            v_events.append((i, vals["V"], has_xyz, line))
        if "U" in vals and cmd in {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}:
            u_events.append((i, vals["U"], line))

    if last_z is None:
        raise ValueError("Could not find a Z coordinate in the CCF file.")
    if not v_events:
        raise ValueError("Could not find any V extrusion values in the CCF file.")
    if not u_events:
        raise ValueError("Could not find any U fiber extrusion values in the CCF file.")

    first_v = v_events[0][1]
    _, final_v, final_has_xyz, _ = v_events[-1]

    if not final_has_xyz:
        deposited_v = None
        for _, v, has_xyz, _ in reversed(v_events[:-1]):
            if has_xyz:
                deposited_v = v
                break
        if deposited_v is None:
            raise ValueError("Could not determine V before the final CCF retract.")
        current_v = final_v
    else:
        deposited_v = final_v
        current_v = final_v

    already_retracted = max(0.0, deposited_v - current_v)
    if already_retracted > TOOLCHANGE_RETRACT + 1e-6:
        raise ValueError(
            f"CCF file already retracts V by {already_retracted:.5f} mm, which is "
            f"greater than the configured tool-change retract of "
            f"{TOOLCHANGE_RETRACT:.5f} mm."
        )

    target_v = deposited_v - TOOLCHANGE_RETRACT
    final_u = u_events[-1][1]
    return last_z, first_v, deposited_v, current_v, target_v, final_u

def analyze_thermoplastic(lines: list[str]) -> tuple[float, float, float, float]:
    """
    Return:
        last_z,
        deposited_e_before_final_source_retract,
        current_e_after_source_retract,
        toolchange_target_e
    """
    last_z = None
    e_events = []

    for i, line in enumerate(lines):
        vals = axis_values(line)
        if "Z" in vals:
            last_z = vals["Z"]
        if "E" in vals:
            has_xyz = any(a in vals for a in ("X", "Y", "Z"))
            e_events.append((i, vals["E"], has_xyz, line))

    if last_z is None:
        raise ValueError("Could not find a Z coordinate in the thermoplastic file.")
    if not e_events:
        raise ValueError("Could not find any E extrusion values in the thermoplastic file.")

    _, final_e, final_has_xyz, _ = e_events[-1]

    if not final_has_xyz:
        deposited_e = None
        for _, e, has_xyz, _ in reversed(e_events[:-1]):
            if has_xyz:
                deposited_e = e
                break
        if deposited_e is None:
            raise ValueError("Could not determine E before the final thermoplastic retract.")
        current_e = final_e
    else:
        deposited_e = final_e
        current_e = final_e

    already_retracted = max(0.0, deposited_e - current_e)
    if already_retracted > TOOLCHANGE_RETRACT + 1e-6:
        raise ValueError(
            f"Thermoplastic file already retracts E by {already_retracted:.5f} mm, "
            f"which is greater than the configured tool-change retract of "
            f"{TOOLCHANGE_RETRACT:.5f} mm."
        )

    target_e = deposited_e - TOOLCHANGE_RETRACT
    return last_z, deposited_e, current_e, target_e


def xy_range(*line_groups: list[str]) -> tuple[float | None, float | None, float | None, float | None]:
    xs, ys = [], []
    for lines in line_groups:
        for line in lines:
            vals = axis_values(line)
            if "X" in vals:
                xs.append(vals["X"])
            if "Y" in vals:
                ys.append(vals["Y"])

    return (
        min(xs) if xs else None,
        max(xs) if xs else None,
        min(ys) if ys else None,
        max(ys) if ys else None,
    )


def z_range(*line_groups: list[str]) -> tuple[float | None, float | None]:
    zs = []
    for lines in line_groups:
        for line in lines:
            vals = axis_values(line)
            if "Z" in vals and command(line) in {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}:
                zs.append(vals["Z"])
    return (min(zs) if zs else None, max(zs) if zs else None)


def make_startup(number_of_layers: int) -> list[str]:
    """Anisoprint startup; T1 is used first for the sacrificial CCF line."""
    return [
        "; ============================================================",
        "; ANISOPRINT MULTILAYER CUSTOM G-CODE",
        f"; Layers 1-{PETG_ONLY_END_LAYERS}: full PETG",
        (
            f"; Layers {PETG_ONLY_END_LAYERS + 1}-"
            f"{number_of_layers - PETG_ONLY_END_LAYERS}: "
            "PETG outer -> CCF -> PETG inner"
        ),
        (
            f"; Layers {number_of_layers - PETG_ONLY_END_LAYERS + 1}-"
            f"{number_of_layers}: full PETG"
        ),
        "; Four independent one-layer source G-codes are required",
        f"; Repeated layers: {number_of_layers}",
        f"; Layer height / Z translation: {fmt(LAYER_HEIGHT, 3)} mm",
        "; Corner-radius/fillet generation: disabled",
        f"; CCF and PETG deposition feed scale: {fmt(PRINT_FEED_SCALE, 2)}",
        "; ============================================================",
        "",
        f"; LAYER_COUNT:{number_of_layers}",
        f"M530 L{number_of_layers}",
        "",
        "; ---- ANISOPRINT STARTUP: CCF TOOL FIRST ----",
        f"M104 S{T1_PRINT_TEMP} T1",
        f"M140 S{BED_TEMP}",
        "G21",
        "G90",
        "M82",
        "G28",
        "M530 S1",
        "G1 Z10 F900",
        "; Start change extruder",
        "M400",
        f"M104 S{T0_WAIT_TEMP} T0",
        f"M109 S{T1_PRINT_TEMP} T1",
        "T1 R ; switch extruder",
        "; End change extruder",
        "G92 E0 ; reset T0 thermoplastic extrusion",
        "G92 V0 ; reset T1 matrix extrusion",
        "G92 U0 ; reset continuous fiber extrusion",
        f"M190 S{BED_TEMP}",
    ]


def make_ccf_preparation_line() -> list[str]:
    """
    Print the sacrificial Aura CCF preparation line before model deposition.

    These commands reproduce the reference's 150 mm Y0 preparation sequence.
    The returned U/V states are carried into the first mixed layer by main();
    the model's first CCF strand therefore performs a normal restore after
    engage instead of a stationary V14 prime on the part.
    """
    return [
        "",
        "; ============================================================",
        "; SACRIFICIAL AURA-STYLE CCF PREPARATION LINE",
        "; Keeps U47 restart and V14 prime away from the model",
        "; ============================================================",
        "M400",
        f"M106 P1 S{CCF_FAN_PWM}",
        f"G0 Z{fmt(CCF_PREP_SAFE_Z)} F{TRAVEL_Z_FEED}",
        f"G0 X{fmt(CCF_PREP_X)} Y{fmt(CCF_PREP_Y)} F7200",
        f"M1001 L{int(CCF_PREP_LENGTH)}",
        f"G1 F600 U{fmt(CCF_FIBER_RESTART)}",
        f"G0 X{fmt(CCF_PREP_X + 0.5)} Y{fmt(CCF_PREP_Y)} "
        f"Z{fmt(CCF_PREP_Z)} F7200",
        f"G1 F{RETRACT_FEED} V14",
        "G4 P0",
        f"G1 X74.3 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V14.01904 U47.8 F480",
        f"G1 X79.3 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V14.13804 U52.8 F480",
        f"G1 X79.8 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V14.14994 F600",
        f"G1 X178.6 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V16.50138 U151.6 F600",
        f"G1 X179.1 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V16.51328 U152.6 F600",
        "; ===== CUT SACRIFICIAL CCF =====",
        "M400",
        "M280 P0 S30",
        "G4 P200",
        "M280 P0 S90",
        "M400",
        f"G1 X186.1 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V16.67988 F180",
        f"G1 X224.3 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} "
        "V17.58904 F360",
        f"G1 F{RETRACT_FEED} V8.58904",
        f"G0 X233.5 Y{fmt(CCF_PREP_Y)} Z{fmt(CCF_PREP_Z)} F300",
        "M1002",
        "; END SACRIFICIAL CCF PREPARATION LINE",
        "G0 Z10 F1500",
    ]


def make_layer_header(
    layer_number: int,
    z_offset: float,
) -> list[str]:
    """Start one layer with the exact outer PETG boundary."""
    return [
        "",
        "; ============================================================",
        f"; MACROLAYER {layer_number} / LAYER {layer_number}",
        f"; Z OFFSET = {fmt(z_offset, 3)} mm",
        "; ============================================================",
        f";MACROLAYER:{layer_number}",
        f";LAYER:{layer_number}",
        f"M532 L{layer_number}",
        "M400",
        f"M106 P1 S{PETG_FAN_PWM}",
        "",
        "; ============== EXACT OUTER PETG BOUNDARY / T0 ==============",
    ]


def make_full_petg_layer_header(
    layer_number: int,
    z_offset: float,
) -> list[str]:
    """Start a complete PETG-only bottom or top skin layer."""
    return [
        "",
        "; ============================================================",
        f"; MACROLAYER {layer_number} / LAYER {layer_number}",
        f"; Z OFFSET = {fmt(z_offset, 3)} mm",
        "; LAYER TYPE = FULL PETG",
        "; ============================================================",
        f";MACROLAYER:{layer_number}",
        f";LAYER:{layer_number}",
        f"M532 L{layer_number}",
        "M400",
        f"M106 P1 S{PETG_FAN_PWM}",
        "",
        "; ==================== FULL PETG LAYER / T0 ====================",
    ]


def make_ccf_section_header(reset_u: bool = False) -> list[str]:
    """Start the CCF boundary after switching from outer PETG to T1."""
    lines = [
        "M400",
        f"M106 P1 S{CCF_FAN_PWM}",
    ]
    if reset_u:
        lines += [
            "; Reference-style U coordinate reset after large absolute U",
            "G92 U0 ; reset continuous-fiber absolute coordinate",
        ]
    lines += [
        "",
        "; ================= CCF OUTER REINFORCEMENT / T1 ================",
    ]
    return lines

def make_ccf_to_thermoplastic_transition(
    last_ccf_z: float,
    deposited_v: float,
    current_v: float,
    target_v: float,
    next_section_label: str,
) -> list[str]:
    """
    Switch from CCF/T1 to PETG/T0 using global absolute extrusion coordinates.

    T0 remains parked at its absolute retracted E coordinate after selection.
    Recovery is deliberately deferred until the next PETG deposition starts;
    no G92 E0 is used.
    """
    z_lift = last_ccf_z + TOOLCHANGE_Z_LIFT
    already = deposited_v - current_v
    additional = current_v - target_v

    lines = [
        "",
        "; ============== CCF -> THERMOPLASTIC ===================",
        "; Start change extruder",
        "M400",
        f"; CCF matrix V deposited value = {fmt(deposited_v)}",
        f"; Retract already present in CCF file = {fmt(already)} mm",
        f"; Additional retract for {fmt(TOOLCHANGE_RETRACT)} mm total = {fmt(additional)} mm",
        f"G1 F{RETRACT_FEED} V{fmt(target_v)}",
        f"M104 S{T0_PREHEAT_TEMP} T0",
        f"G0 Z{fmt(z_lift, 5)} F{TRAVEL_Z_FEED}",
        "M1013",
        f"M104 S{T1_WAIT_TEMP} T1",
        "M400",
        "M106 P1 S0",
        f"M109 S{T0_PRINT_TEMP} T0",
        "T0 ; switch extruder",
        "M1013 R",
        "; End change extruder",
        "M400",
        f"M106 P1 S{PETG_FAN_PWM}",
    ]

    lines += [
        "",
        f"; ================= {next_section_label} / T0 =================",
    ]
    return lines

def make_thermoplastic_to_ccf_transition(
    last_thermo_z: float,
    deposited_e: float,
    current_e: float,
    target_e: float,
) -> list[str]:
    """Park PETG/T0 with a 12 mm absolute-E retract and switch to CCF/T1."""
    z_lift = last_thermo_z + TOOLCHANGE_Z_LIFT
    already = deposited_e - current_e
    additional = current_e - target_e

    return [
        "",
        "; ============== THERMOPLASTIC -> CCF ===================",
        "; Start change extruder",
        "M400",
        f"; T0 E deposited value = {fmt(deposited_e)}",
        f"; Retract already present in PETG file = {fmt(already)} mm",
        f"; Additional retract for {fmt(TOOLCHANGE_RETRACT)} mm total = {fmt(additional)} mm",
        f"G1 F{RETRACT_FEED} E{fmt(target_e)}",
        f"M104 S{T1_PREHEAT_TEMP} T1",
        f"G0 Z{fmt(z_lift, 5)} F{TRAVEL_Z_FEED}",
        "M1013",
        f"M104 S{T0_WAIT_TEMP} T0",
        "M400",
        "M106 P1 S0",
        f"M109 S{T1_PRINT_TEMP} T1",
        "T1 ; switch extruder",
        "M1013 R",
        "; End change extruder",
    ]


def make_petg_interlayer_transition(
    last_petg_z: float,
    deposited_e: float,
    current_e: float,
    current_layer_type: str,
    next_layer_type: str,
) -> list[str]:
    """Keep T0 active, retract for travel and Z-lift before the next layer."""
    target_e = deposited_e - PETG_INTERLAYER_RETRACT
    lines = [
        "",
        (
            f"; ============== {current_layer_type} -> "
            f"{next_layer_type} =============="
        ),
        "M400",
        f"; PETG deposited E = {fmt(deposited_e)}",
        f"; Existing PETG retract = {fmt(deposited_e - current_e)} mm",
    ]

    if current_e > target_e + 1e-9:
        lines += [
            f"; Complete inter-layer retract to {fmt(PETG_INTERLAYER_RETRACT)} mm",
            f"G1 F{RETRACT_FEED} E{fmt(target_e)}",
        ]
    else:
        lines.append("; Existing source retract is sufficient for inter-layer travel")

    lines += [
        f"G0 Z{fmt(last_petg_z + TOOLCHANGE_Z_LIFT, 5)} F{TRAVEL_Z_FEED}",
        "; T0 remains selected; recovery occurs at next PETG deposition",
    ]
    return lines


def make_shutdown(deposited_e: float, current_e: float, target_e: float) -> list[str]:
    """Final shutdown after PETG of the last layer."""
    already = deposited_e - current_e
    additional = current_e - target_e

    return [
        "",
        "; ==================== ANISOPRINT END ===================",
        f"; T0 deposited E = {fmt(deposited_e)}",
        f"; Existing final PETG retract = {fmt(already)} mm",
        f"; Additional final retract = {fmt(additional)} mm",
        f"G1 F{RETRACT_FEED} E{fmt(target_e)} ; {fmt(TOOLCHANGE_RETRACT)} mm total final T0 retract",
        "M400",
        "M104 S0 T0",
        "M104 S0 T1",
        "M106 P1 S0",
        "M140 S0",
        "G91",
        "G1 Z20 F900",
        "G90",
        "G28",
        "M530 S0",
        "; ============================================================",
    ]


def main() -> None:
    if NUMBER_OF_LAYERS < 1:
        raise ValueError("NUMBER_OF_LAYERS must be at least 1.")
    if PETG_ONLY_END_LAYERS < 1:
        raise ValueError("PETG_ONLY_END_LAYERS must be at least 1.")
    minimum_layers = 2 * PETG_ONLY_END_LAYERS + 1
    if NUMBER_OF_LAYERS < minimum_layers:
        raise ValueError(
            f"NUMBER_OF_LAYERS must be at least {minimum_layers} so the "
            "PETG-only bottom/top skins leave at least one mixed layer."
        )
    if LAYER_HEIGHT <= 0:
        raise ValueError("LAYER_HEIGHT must be greater than 0.")
    if not 0 < PRINT_FEED_SCALE <= 1:
        raise ValueError("PRINT_FEED_SCALE must be greater than 0 and at most 1.")
    if not 0 < PETG_INTERLAYER_RETRACT <= TOOLCHANGE_RETRACT:
        raise ValueError(
            "PETG_INTERLAYER_RETRACT must be greater than 0 and no greater "
            "than TOOLCHANGE_RETRACT."
        )
    if not (
        BED_X_MIN <= CCF_PREP_X <= BED_X_MAX
        and BED_Y_MIN <= CCF_PREP_Y <= BED_Y_MAX
        and BED_X_MIN <= 233.5 <= BED_X_MAX
    ):
        raise ValueError("The sacrificial CCF preparation line is outside the bed.")

    full_petg_raw = read_lines(FULL_PETG_GCODE)
    petg_outer_raw = read_lines(PETG_OUTER_GCODE)
    ccf_raw = read_lines(CCF_GCODE)
    petg_inner_raw = read_lines(PETG_INNER_GCODE)

    # Keep every source corner exactly as supplied. No radius/fillet geometry
    # is generated for either CCF or PETG.
    ccf_raw = arrange_ccf_start_sequence(ccf_raw)

    # Reduce only material-deposition feeds by 30%. Travel, retract/prime and
    # tool-change feeds remain unchanged.
    full_petg_raw, full_petg_scaled_moves = scale_deposition_feed_rates(
        full_petg_raw,
        extrusion_axes={"E"},
        scale=PRINT_FEED_SCALE,
    )
    petg_outer_raw, petg_outer_scaled_moves = scale_deposition_feed_rates(
        petg_outer_raw,
        extrusion_axes={"E"},
        scale=PRINT_FEED_SCALE,
    )
    ccf_raw, ccf_scaled_moves = scale_deposition_feed_rates(
        ccf_raw,
        extrusion_axes={"U", "V"},
        scale=PRINT_FEED_SCALE,
    )
    petg_inner_raw, petg_inner_scaled_moves = scale_deposition_feed_rates(
        petg_inner_raw,
        extrusion_axes={"E"},
        scale=PRINT_FEED_SCALE,
    )

    # Analyze the original one-layer source coordinates once.
    (
        base_last_ccf_z,
        first_v_prime,
        local_deposited_v,
        local_current_v,
        local_target_v,
        local_final_u,
    ) = analyze_ccf(ccf_raw)
    if abs(first_v_prime - 14.0) > 0.01:
        raise ValueError(
            "The sacrificial preparation line is calibrated for the reference "
            f"V14 prime, but the CCF source starts at V{first_v_prime:.5f}. "
            "Use the matching CFC PETG profile before integration."
        )
    (
        base_last_full_petg_z,
        local_full_deposited_e,
        local_full_current_e,
        local_full_target_e,
    ) = analyze_thermoplastic(full_petg_raw)
    (
        base_last_petg_outer_z,
        local_outer_deposited_e,
        local_outer_current_e,
        local_outer_target_e,
    ) = analyze_thermoplastic(petg_outer_raw)
    (
        base_last_petg_inner_z,
        local_inner_deposited_e,
        local_inner_current_e,
        local_inner_target_e,
    ) = analyze_thermoplastic(petg_inner_raw)

    # Remove source-local positioning/extrusion resets. The integrated output
    # establishes G90/M82 and G92 E/V/U once at startup.
    full_petg_body = strip_source_mode_setup(full_petg_raw, {"E"})
    petg_outer_body = strip_source_mode_setup(petg_outer_raw, {"E"})
    ccf_body = strip_source_mode_setup(ccf_raw, {"U", "V", "E"})
    petg_inner_body = strip_source_mode_setup(petg_inner_raw, {"E"})

    output_lines: list[str] = []
    output_lines += make_startup(NUMBER_OF_LAYERS)
    ccf_preparation, prep_scaled_moves = scale_deposition_feed_rates(
        make_ccf_preparation_line(),
        extrusion_axes={"U", "V"},
        scale=PRINT_FEED_SCALE,
    )
    output_lines += ccf_preparation
    output_lines += make_ccf_to_thermoplastic_transition(
        last_ccf_z=CCF_PREP_Z,
        deposited_v=CCF_PREP_DEPOSITED_V,
        current_v=CCF_PREP_CURRENT_V,
        target_v=CCF_PREP_DEPOSITED_V - TOOLCHANGE_RETRACT,
        next_section_label="READY FOR FULL PETG LAYER",
    )

    # Global absolute-coordinate states, matching the Anisoprint reference.
    previous_petg_deposited_e = 0.0
    previous_ccf_deposited_v: float | None = CCF_PREP_DEPOSITED_V
    current_u_base = CCF_PREP_FINAL_U

    final_abs_deposited_e = 0.0

    for layer_idx in range(NUMBER_OF_LAYERS):
        layer_number = layer_idx + 1
        z_offset = layer_idx * LAYER_HEIGHT
        full_petg_layer = is_full_petg_layer(layer_number, NUMBER_OF_LAYERS)

        # ========================================================
        # FULL-PETG BOTTOM/TOP SKIN LAYER
        # ========================================================
        if full_petg_layer:
            full_e_offset = previous_petg_deposited_e
            full_petg_layer_lines = transform_layer(
                full_petg_body,
                z_offset=z_offset,
                e_offset=full_e_offset,
            )
            if layer_idx > 0:
                full_petg_layer_lines = insert_petg_restore_before_deposition(
                    full_petg_layer_lines,
                    restore_e_to=previous_petg_deposited_e,
                )

            abs_full_deposited_e = local_full_deposited_e + full_e_offset
            abs_full_current_e = local_full_current_e + full_e_offset
            abs_full_target_e = local_full_target_e + full_e_offset
            shifted_last_full_petg_z = base_last_full_petg_z + z_offset

            output_lines += make_full_petg_layer_header(layer_number, z_offset)
            output_lines += full_petg_layer_lines

            if layer_idx < NUMBER_OF_LAYERS - 1:
                next_layer_number = layer_number + 1
                next_layer_type = (
                    "FULL PETG LAYER"
                    if is_full_petg_layer(next_layer_number, NUMBER_OF_LAYERS)
                    else "MIXED PETG OUTER BOUNDARY"
                )
                output_lines += make_petg_interlayer_transition(
                    last_petg_z=shifted_last_full_petg_z,
                    deposited_e=abs_full_deposited_e,
                    current_e=abs_full_current_e,
                    current_layer_type="FULL PETG LAYER",
                    next_layer_type=next_layer_type,
                )
            else:
                output_lines += make_shutdown(
                    deposited_e=abs_full_deposited_e,
                    current_e=abs_full_current_e,
                    target_e=abs_full_target_e,
                )

            previous_petg_deposited_e = abs_full_deposited_e
            final_abs_deposited_e = abs_full_deposited_e
            continue

        # ========================================================
        # MIXED LAYER: PETG OUTER -> CCF -> PETG INNER
        # ========================================================
        reset_u = False
        if U_RESET_THRESHOLD is not None and current_u_base >= U_RESET_THRESHOLD:
            current_u_base = 0.0
            reset_u = True
        u_offset = current_u_base

        # 1) Exact outer PETG boundary (T0)
        outer_e_offset = previous_petg_deposited_e
        petg_outer_layer = transform_layer(
            petg_outer_body,
            z_offset=z_offset,
            e_offset=outer_e_offset,
        )
        petg_outer_layer = insert_petg_restore_before_deposition(
            petg_outer_layer,
            restore_e_to=previous_petg_deposited_e,
        )
        abs_outer_deposited_e = local_outer_deposited_e + outer_e_offset
        abs_outer_current_e = local_outer_current_e + outer_e_offset
        abs_outer_target_e = local_outer_target_e + outer_e_offset

        # 2) Adjacent CCF boundary (T1)
        assert previous_ccf_deposited_v is not None
        v_offset = previous_ccf_deposited_v - first_v_prime
        ccf_layer = transform_layer(
            ccf_body,
            z_offset=z_offset,
            u_offset=u_offset,
            v_offset=v_offset,
        )
        abs_deposited_v = local_deposited_v + v_offset
        abs_current_v = local_current_v + v_offset
        abs_target_v = local_target_v + v_offset

        # 3) Existing inner PETG boundary (T0)
        inner_e_offset = abs_outer_deposited_e
        petg_inner_layer = transform_layer(
            petg_inner_body,
            z_offset=z_offset,
            e_offset=inner_e_offset,
        )
        petg_inner_layer = insert_petg_restore_before_deposition(
            petg_inner_layer,
            restore_e_to=abs_outer_deposited_e,
        )
        abs_inner_deposited_e = local_inner_deposited_e + inner_e_offset
        abs_inner_current_e = local_inner_current_e + inner_e_offset
        abs_inner_target_e = local_inner_target_e + inner_e_offset

        shifted_last_petg_outer_z = base_last_petg_outer_z + z_offset
        shifted_last_ccf_z = base_last_ccf_z + z_offset
        shifted_last_petg_inner_z = base_last_petg_inner_z + z_offset

        output_lines += make_layer_header(layer_number, z_offset)
        output_lines += petg_outer_layer
        output_lines += make_thermoplastic_to_ccf_transition(
            last_thermo_z=shifted_last_petg_outer_z,
            deposited_e=abs_outer_deposited_e,
            current_e=abs_outer_current_e,
            target_e=abs_outer_target_e,
        )
        output_lines += make_ccf_section_header(reset_u=reset_u)
        output_lines += ccf_layer
        output_lines += make_ccf_to_thermoplastic_transition(
            last_ccf_z=shifted_last_ccf_z,
            deposited_v=abs_deposited_v,
            current_v=abs_current_v,
            target_v=abs_target_v,
            next_section_label="EXISTING INNER PETG BOUNDARY",
        )
        output_lines += petg_inner_layer

        if layer_idx < NUMBER_OF_LAYERS - 1:
            next_layer_number = layer_number + 1
            next_layer_type = (
                "FULL PETG LAYER"
                if is_full_petg_layer(next_layer_number, NUMBER_OF_LAYERS)
                else "MIXED PETG OUTER BOUNDARY"
            )
            output_lines += make_petg_interlayer_transition(
                last_petg_z=shifted_last_petg_inner_z,
                deposited_e=abs_inner_deposited_e,
                current_e=abs_inner_current_e,
                current_layer_type="MIXED PETG INNER BOUNDARY",
                next_layer_type=next_layer_type,
            )
        else:
            output_lines += make_shutdown(
                deposited_e=abs_inner_deposited_e,
                current_e=abs_inner_current_e,
                target_e=abs_inner_target_e,
            )

        previous_petg_deposited_e = abs_inner_deposited_e
        previous_ccf_deposited_v = abs_deposited_v
        current_u_base = u_offset + local_final_u
        final_abs_deposited_e = abs_inner_deposited_e

    full_layer_count = 2 * PETG_ONLY_END_LAYERS
    mixed_layer_count = NUMBER_OF_LAYERS - full_layer_count
    expected_final_e = (
        full_layer_count * local_full_deposited_e
        + mixed_layer_count
        * (local_outer_deposited_e + local_inner_deposited_e)
    )
    if not math.isclose(final_abs_deposited_e, expected_final_e, abs_tol=1e-5):
        raise ValueError(
            "Internal PETG extrusion-state error: final absolute E is "
            f"{final_abs_deposited_e:.5f} mm; expected "
            f"{expected_final_e:.5f} mm from the parametric layer schedule."
        )

    OUTPUT_GCODE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_GCODE.write_text("\n".join(output_lines) + "\n", encoding="utf-8")

    xmin, xmax, ymin, ymax = xy_range(
        full_petg_raw,
        petg_outer_raw,
        ccf_raw,
        petg_inner_raw,
    )
    zmin, zmax = z_range(
        full_petg_raw,
        petg_outer_raw,
        ccf_raw,
        petg_inner_raw,
    )

    print("Multilayer integrated G-code created successfully:")
    print(f"  {OUTPUT_GCODE}")
    print()
    print(f"Number of layers             : {NUMBER_OF_LAYERS}")
    print(f"Layer-to-layer Z translation : {LAYER_HEIGHT:.5f} mm")
    print(f"Top-layer Z translation      : {(NUMBER_OF_LAYERS - 1) * LAYER_HEIGHT:.5f} mm")
    print(
        "Full-PETG layers              : "
        f"1-{PETG_ONLY_END_LAYERS} and "
        f"{NUMBER_OF_LAYERS - PETG_ONLY_END_LAYERS + 1}-{NUMBER_OF_LAYERS}"
    )
    print(
        "Mixed PETG/CCF layers         : "
        f"{PETG_ONLY_END_LAYERS + 1}-"
        f"{NUMBER_OF_LAYERS - PETG_ONLY_END_LAYERS}"
    )
    print(f"Full PETG deposited E/layer   : {local_full_deposited_e:.5f} mm")
    print(f"Outer PETG deposited E/layer  : {local_outer_deposited_e:.5f} mm")
    print(f"Inner PETG deposited E/layer  : {local_inner_deposited_e:.5f} mm")
    print(
        "Mixed-layer PETG E total      : "
        f"{local_outer_deposited_e + local_inner_deposited_e:.5f} mm"
    )
    print(f"Final global PETG E           : {final_abs_deposited_e:.5f} mm")
    print(f"CCF first V prime             : {first_v_prime:.5f} mm")
    print(f"CCF local deposited V         : {local_deposited_v:.5f} mm")
    print(f"Final global CCF V            : {previous_ccf_deposited_v:.5f} mm")
    print(f"Final U coordinate            : {current_u_base:.5f} mm")
    print("Corner-radius generation      : disabled for CCF and PETG")
    print(f"Deposition feed scale         : {PRINT_FEED_SCALE:.2f} (30% reduction)")
    print(f"CCF source moves scaled/layer : {ccf_scaled_moves}")
    print(f"Full PETG moves scaled/layer  : {full_petg_scaled_moves}")
    print(f"Outer PETG moves scaled/layer : {petg_outer_scaled_moves}")
    print(f"Inner PETG moves scaled/layer : {petg_inner_scaled_moves}")
    print(f"CCF preparation moves scaled  : {prep_scaled_moves}")
    print("Sacrificial CCF prep line      : enabled at Y=0")

    if None not in (xmin, xmax, ymin, ymax):
        print()
        print(f"Source XY range: X={xmin:.3f}..{xmax:.3f}, Y={ymin:.3f}..{ymax:.3f} mm")
        outside = (
            xmin < BED_X_MIN or xmax > BED_X_MAX or
            ymin < BED_Y_MIN or ymax > BED_Y_MAX
        )
        if outside:
            print("WARNING: The source path extends outside the nominal Composer A4 XY range.")
            print("         This script does NOT modify or offset X/Y coordinates.")

    if zmin is not None and zmax is not None:
        final_zmax = zmax + (NUMBER_OF_LAYERS - 1) * LAYER_HEIGHT
        print()
        print(f"Source Z range               : {zmin:.5f}..{zmax:.5f} mm")
        print(f"Final repeated geometry Z max: {final_zmax:.5f} mm")

    print()
    print("Absolute extrusion logic:")
    print("  E: one G92 E0 at startup; PETG continues across layers")
    print("  V: one G92 V0 at startup; CCF matrix continues across layers")
    print("  U: continuous across layers, with optional ~10000 mm reset")

if __name__ == "__main__":
    main()
