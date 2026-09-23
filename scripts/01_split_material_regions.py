"""Step 01 - clean an NX G-code file and split its closed deposition loops.

The raw non-planar G-code exported from NX is reduced to pure G0/G1 XYZ motion,
then its concentric closed tracks are allocated to the three material regions:

    loops 1..PETG_OUTER_LOOP_COUNT            -> PETG outer boundary
    the next CCF_LOOP_COUNT loops             -> continuous carbon fiber
    everything remaining                      -> PETG inner boundary

The full cleaned file is also written, and is reused later as the solid
bottom/top PETG surfaces.

Every sample listed in SAMPLES below is processed independently and receives
its own subfolder under data/01_split/.
"""

from dataclasses import dataclass
from pathlib import Path
import re

from paths import INPUT_DIR, SPLIT_DIR


# ---------------------------------------------------------------------------
# Samples to process
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Sample:
    """One NX export and the loop allocation that belongs to it."""

    name: str
    input_file: Path
    petg_outer_loop_count: int
    ccf_loop_count: int

    @property
    def output_dir(self) -> Path:
        return SPLIT_DIR / self.name


# The loop counts are geometry-specific: they depend on how many concentric
# tracks NX generated for this part. Inspect the loop listing printed by a
# first run, then set the counts to match.
SAMPLES = (
    Sample(
        name="petg_source",
        input_file=INPUT_DIR / "nx_nonplanar_petg_source.gcode",
        petg_outer_loop_count=2,
        ccf_loop_count=8,
    ),
    Sample(
        name="ccf_source",
        input_file=INPUT_DIR / "nx_nonplanar_ccf_source.gcode",
        petg_outer_loop_count=1,
        ccf_loop_count=9,
    ),
)

# Output file names are identical inside every sample subfolder.
FULL_SURFACES_NAME = "petg_full_surfaces.gcode"
PETG_OUTER_NAME = "petg_outer_boundary.gcode"
CCF_NAME = "ccf_near_outer_boundary.gcode"
PETG_INNER_NAME = "petg_inner_boundary.gcode"


# ---------------------------------------------------------------------------
# Editable splitting and motion parameters
# ---------------------------------------------------------------------------

DEPOSITION_LOOP_COUNT = 15

# Two XY positions are considered equal when their difference is within this
# tolerance. This allows the loop detector to handle small rounding changes.
LOOP_POSITION_TOLERANCE = 0.001
MIN_LOOP_LINE_COUNT = 20
MAX_LOOP_LINE_COUNT = 2000

# Each split file is made independently executable using motion commands only:
# safe-Z -> XY positioning -> rapid approach -> G1 engagement -> track -> safe-Z.
SAFE_Z = 17.4
RAPID_APPROACH_CLEARANCE = 3.0


NUMBER_PATTERN = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?"
WORD_RE = re.compile(
    rf"(?<![A-Za-z])([A-Za-z])\s*({NUMBER_PATTERN})",
    re.IGNORECASE,
)
MOTION_AXES = {"X", "Y", "Z"}


@dataclass(frozen=True)
class MotionRecord:
    """One cleaned motion line and the modal XYZ position after that line."""

    text: str
    motion: str
    explicit_axes: frozenset[str]
    x: float | None
    y: float | None
    z: float | None


@dataclass(frozen=True)
class TrackLoop:
    """Inclusive line-index range of one closed deposition loop."""

    start_index: int
    end_index: int
    start_x: float
    start_y: float


def remove_comments(line: str, comment_depth: int) -> tuple[str, int]:
    """Remove semicolon comments and possibly multiline parenthesis comments."""
    cleaned: list[str] = []

    for character in line:
        if comment_depth > 0:
            if character == "(":
                comment_depth += 1
            elif character == ")":
                comment_depth -= 1
            continue

        if character == "(":
            comment_depth = 1
        elif character == ";":
            break
        else:
            cleaned.append(character)

    return "".join(cleaned), comment_depth


def normalize_motion_code(value: str) -> str | None:
    """Return canonical G0/G1 text, or None for every other G-code."""
    try:
        numeric_value = float(value)
    except ValueError:
        return None

    if numeric_value == 0.0:
        return "G0"
    if numeric_value == 1.0:
        return "G1"
    return None


def clean_gcode_lines(input_file: Path) -> tuple[int, list[str]]:
    """Return motion-only lines with an explicit G0 or G1 on every line."""
    source_lines = input_file.read_text(encoding="utf-8-sig").splitlines()

    output_lines: list[str] = []
    active_motion: str | None = None
    comment_depth = 0

    for line_number, source_line in enumerate(source_lines, start=1):
        uncommented, comment_depth = remove_comments(source_line, comment_depth)
        words = WORD_RE.findall(uncommented)

        # G0 and G1 are modal. Lines containing only coordinates inherit the
        # latest motion type, but it is written explicitly in the output.
        for letter, value in words:
            if letter.upper() == "G":
                motion = normalize_motion_code(value)
                if motion is not None:
                    active_motion = motion

        coordinates = [
            f"{letter.upper()}{value}"
            for letter, value in words
            if letter.upper() in MOTION_AXES
        ]

        # This removes metadata, N numbers, comments, G17/G21/G43/G54/G90/G94,
        # T/S/M/H/F words, blank lines, and every other non-motion command.
        if not coordinates:
            continue

        if active_motion is None:
            raise ValueError(
                f"Coordinates were found on input line {line_number}, but no "
                "earlier G0 or G1 motion mode was found."
            )

        output_lines.append(" ".join([active_motion, *coordinates]))

    if comment_depth != 0:
        raise ValueError("The input contains an unclosed parenthesis comment.")
    if not output_lines:
        raise ValueError("No G0/G1 coordinate movements were found in the input file.")

    return len(source_lines), output_lines


def build_motion_records(cleaned_lines: list[str]) -> list[MotionRecord]:
    """Calculate modal XYZ positions for all cleaned lines."""
    records: list[MotionRecord] = []
    position: dict[str, float | None] = {"X": None, "Y": None, "Z": None}

    for line in cleaned_lines:
        motion = line.split(maxsplit=1)[0]
        explicit_coordinates: dict[str, float] = {}

        for letter, value in WORD_RE.findall(line):
            axis = letter.upper()
            if axis in MOTION_AXES:
                explicit_coordinates[axis] = float(value)

        position.update(explicit_coordinates)
        records.append(
            MotionRecord(
                text=line,
                motion=motion,
                explicit_axes=frozenset(explicit_coordinates),
                x=position["X"],
                y=position["Y"],
                z=position["Z"],
            )
        )

    return records


def same_xy(first: MotionRecord, second: MotionRecord) -> bool:
    """Return True when two records end at the same XY position."""
    if first.x is None or first.y is None or second.x is None or second.y is None:
        return False

    return (
        abs(first.x - second.x) <= LOOP_POSITION_TOLERANCE
        and abs(first.y - second.y) <= LOOP_POSITION_TOLERANCE
    )


def identify_closed_loops(
    records: list[MotionRecord], required_loop_count: int
) -> list[TrackLoop]:
    """Identify consecutive closed G1 tracks by their repeated start/end XY."""
    loops: list[TrackLoop] = []
    search_index = 0

    while len(loops) < required_loop_count:
        found_loop: TrackLoop | None = None

        for start_index in range(search_index, len(records)):
            start = records[start_index]
            if start.motion != "G1" or not {"X", "Y"}.issubset(start.explicit_axes):
                continue

            first_possible_end = start_index + MIN_LOOP_LINE_COUNT - 1
            last_possible_end = min(
                start_index + MAX_LOOP_LINE_COUNT - 1,
                len(records) - 1,
            )

            for end_index in range(first_possible_end, last_possible_end + 1):
                end = records[end_index]
                if end.motion != "G1" or not {"X", "Y"}.issubset(end.explicit_axes):
                    continue
                if same_xy(start, end):
                    if start.x is None or start.y is None:
                        raise RuntimeError("Unexpected missing loop-start coordinates.")
                    found_loop = TrackLoop(
                        start_index=start_index,
                        end_index=end_index,
                        start_x=start.x,
                        start_y=start.y,
                    )
                    break

            if found_loop is not None:
                break

        if found_loop is None:
            raise ValueError(
                f"Only {len(loops)} closed deposition loops were found; "
                f"{required_loop_count} were requested."
            )

        loops.append(found_loop)
        search_index = found_loop.end_index + 1

    return loops


def format_number(value: float) -> str:
    """Format generated coordinates without unnecessary zeroes or spaces."""
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")


def build_independent_segment(
    records: list[MotionRecord], start_index: int, end_index: int
) -> list[str]:
    """Wrap a source segment with motion-only safe approach and retraction."""
    if not 0 <= start_index <= end_index < len(records):
        raise ValueError("Invalid split-segment line range.")

    entry = records[start_index]
    if entry.x is None or entry.y is None or entry.z is None:
        raise ValueError(
            f"Cannot safely engage at cleaned line {start_index + 1}: "
            "the modal X, Y, or Z position is unknown."
        )
    if SAFE_Z <= entry.z:
        raise ValueError(
            f"SAFE_Z ({SAFE_Z}) must be higher than the segment entry Z ({entry.z})."
        )

    approach_z = min(SAFE_Z, entry.z + RAPID_APPROACH_CLEARANCE)
    segment_lines = [
        f"G0 Z{format_number(SAFE_Z)}",
        f"G0 X{format_number(entry.x)} Y{format_number(entry.y)}",
    ]

    if approach_z < SAFE_Z - LOOP_POSITION_TOLERANCE:
        segment_lines.append(f"G0 Z{format_number(approach_z)}")

    # This G1 move is the motion-only engagement. The original first track
    # point is intentionally retained after it, even if it is a zero-length
    # move, so the detected loop remains complete and auditable.
    segment_lines.append(f"G1 Z{format_number(entry.z)}")
    segment_lines.extend(record.text for record in records[start_index : end_index + 1])

    exit_z = records[end_index].z
    if exit_z is None or abs(exit_z - SAFE_Z) > LOOP_POSITION_TOLERANCE:
        # G0 marks a non-deposition exit in this motion-only representation.
        # Actual polymer/fiber retraction must be handled by the later machine-
        # specific integration code because E/T/M commands are excluded here.
        segment_lines.append(f"G0 Z{format_number(SAFE_Z)}")

    return segment_lines


def validate_split_settings(petg_outer_loop_count: int, ccf_loop_count: int) -> None:
    """Reject loop-count combinations that cannot be split safely."""
    if DEPOSITION_LOOP_COUNT < 1:
        raise ValueError("DEPOSITION_LOOP_COUNT must be at least 1.")
    if petg_outer_loop_count < 1:
        raise ValueError("petg_outer_loop_count must be at least 1.")
    if ccf_loop_count < 1:
        raise ValueError("ccf_loop_count must be at least 1.")
    if petg_outer_loop_count + ccf_loop_count > DEPOSITION_LOOP_COUNT:
        raise ValueError(
            "petg_outer_loop_count + ccf_loop_count cannot exceed "
            "DEPOSITION_LOOP_COUNT."
        )
    if RAPID_APPROACH_CLEARANCE <= 0:
        raise ValueError("RAPID_APPROACH_CLEARANCE must be greater than zero.")


def write_gcode(output_file: Path, lines: list[str]) -> None:
    """Write one clean G-code file, creating its output directory if needed."""
    if not lines:
        raise ValueError(f"Refusing to write an empty G-code file: {output_file}")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_outputs(
    input_file: Path,
    cleaned_file: Path,
    petg_outer_file: Path,
    ccf_file: Path,
    petg_inner_file: Path,
    petg_outer_loop_count: int,
    ccf_loop_count: int,
) -> tuple[int, list[TrackLoop], dict[str, int]]:
    """Clean, identify loops, split the path, and write all four output files."""
    validate_split_settings(petg_outer_loop_count, ccf_loop_count)
    source_line_count, cleaned_lines = clean_gcode_lines(input_file)
    records = build_motion_records(cleaned_lines)
    loops = identify_closed_loops(records, DEPOSITION_LOOP_COUNT)

    outer_start = loops[0].start_index
    outer_end = loops[petg_outer_loop_count - 1].end_index

    ccf_first_loop = petg_outer_loop_count
    ccf_last_loop = petg_outer_loop_count + ccf_loop_count - 1
    ccf_start = loops[ccf_first_loop].start_index
    ccf_end = loops[ccf_last_loop].end_index

    next_loop_index = petg_outer_loop_count + ccf_loop_count
    if next_loop_index < DEPOSITION_LOOP_COUNT:
        inner_start = loops[next_loop_index].start_index
    else:
        inner_start = loops[-1].end_index + 1

    if inner_start >= len(records):
        raise ValueError("No G-code remains for the PETG inner-boundary output.")

    petg_outer_lines = build_independent_segment(records, outer_start, outer_end)
    ccf_lines = build_independent_segment(records, ccf_start, ccf_end)
    petg_inner_lines = build_independent_segment(
        records,
        inner_start,
        len(records) - 1,
    )

    write_gcode(cleaned_file, cleaned_lines)
    write_gcode(petg_outer_file, petg_outer_lines)
    write_gcode(ccf_file, ccf_lines)
    write_gcode(petg_inner_file, petg_inner_lines)

    output_counts = {
        "cleaned": len(cleaned_lines),
        "petg_outer": len(petg_outer_lines),
        "ccf": len(ccf_lines),
        "petg_inner": len(petg_inner_lines),
    }
    return source_line_count, loops, output_counts


def process_sample(sample: Sample) -> None:
    """Clean and split one sample, printing the loop allocation it used."""
    if not sample.input_file.is_file():
        raise FileNotFoundError(f"Input G-code file not found: {sample.input_file}")

    output_dir = sample.output_dir
    cleaned_file = output_dir / FULL_SURFACES_NAME
    petg_outer_file = output_dir / PETG_OUTER_NAME
    ccf_file = output_dir / CCF_NAME
    petg_inner_file = output_dir / PETG_INNER_NAME

    source_count, loops, counts = generate_outputs(
        input_file=sample.input_file,
        cleaned_file=cleaned_file,
        petg_outer_file=petg_outer_file,
        ccf_file=ccf_file,
        petg_inner_file=petg_inner_file,
        petg_outer_loop_count=sample.petg_outer_loop_count,
        ccf_loop_count=sample.ccf_loop_count,
    )

    outer_count = sample.petg_outer_loop_count
    ccf_count = sample.ccf_loop_count

    print(f"\n=== {sample.name} ===")
    print(f"Input lines: {source_count}")
    print(f"Closed deposition loops identified: {len(loops)}")
    for loop_number, loop in enumerate(loops, start=1):
        print(
            f"  Loop {loop_number:02d}: cleaned lines "
            f"{loop.start_index + 1}-{loop.end_index + 1}, "
            f"start/end X{format_number(loop.start_x)} "
            f"Y{format_number(loop.start_y)}"
        )

    print("\nLoop allocation:")
    print(f"  PETG outer boundary: loops 1-{outer_count}")
    print(f"  CCF near boundary: loops {outer_count + 1}-{outer_count + ccf_count}")
    print(
        f"  PETG inner boundary: remaining G-code beginning with loop "
        f"{outer_count + ccf_count + 1}"
    )

    print("\nOutput files:")
    print(f"  Cleaned ({counts['cleaned']} lines): {cleaned_file}")
    print(f"  PETG outer ({counts['petg_outer']} lines): {petg_outer_file}")
    print(f"  CCF ({counts['ccf']} lines): {ccf_file}")
    print(f"  PETG inner ({counts['petg_inner']} lines): {petg_inner_file}")


def main() -> None:
    for sample in SAMPLES:
        process_sample(sample)


if __name__ == "__main__":
    main()
