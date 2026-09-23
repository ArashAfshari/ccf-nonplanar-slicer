"""Step 02 - round selected corners in motion-only G0/G1 files.

Runs on the output of step 01, before any extrusion or process commands exist.
The requested radius is an XY circle radius, approximated with G1 chords.
Z is interpolated between each fillet's endpoints: this is an approximation of
the non-planar surface, not a reconstruction from CAD. Adaptive mode tries
smaller radii where necessary and reports them. Corners that still cannot fit
remain unchanged and are reported.

Travel moves, vertical moves, and steep approach/lift moves are protected.
The crossing guard checks XY centreline crossings, NOT bead width or coverage.
Preview the result: step 06 regenerates extrusion from the changed 3D lengths.

IMPORTANT: the default REGIONS below are the sharp corners of the reference
part geometry, in its ORIGINAL coordinates, before step 03 translation and
step 04 rotation. They will not match a different part. Set REGIONS = None to
consider every corner in the file, or measure your own windows from the
step 01 output.
"""

from pathlib import Path
import bisect
import math
import re

from paths import SPLIT_DIR

# -------------------------- EDIT THESE SETTINGS --------------------------
# Only this sample's files have hand-measured corner regions. Filleting is not
# applied to the other samples.
BASE_DIRECTORY = SPLIT_DIR / "petg_source"
FILE_PAIRS = [
    (BASE_DIRECTORY / "petg_full_surfaces.gcode",
     BASE_DIRECTORY / "petg_full_surfaces_filleted.gcode"),
    (BASE_DIRECTORY / "petg_inner_boundary.gcode",
     BASE_DIRECTORY / "petg_inner_boundary_filleted.gcode"),
]

FILLET_RADIUS_MM = 0.4       # Preferred XY radius; 0 copies unchanged.
ADAPT_RADIUS = True         # False requires exactly FILLET_RADIUS_MM.
MIN_FILLET_RADIUS_MM = 0.05 # Lower bound for adaptive attempts.
RADIUS_STEP_MM = 0.025      # Try progressively smaller radii at this interval.
TIP_CLUSTER_MM = 0.05       # Treat adjacent same-turn tip vertices as one corner.
MIN_TURN_DEG = 60.0         # Direction change: straight=0, reversal=180.
MAX_SEARCH_DISTANCE_MM = 5.0  # Search along each side across short segments.
MAX_CHORD_MM = 0.08         # Maximum length of generated G1 arc chords.
ARC_ERROR_MM = 0.002        # Maximum ideal-circle-to-chord error.
DECIMALS = 4               # Coordinate rounding is additional to arc error.
MAX_Z_SLOPE = 0.30         # Protect moves with abs(dZ)/XY length above this.
CHECK_XY_CROSSINGS = True

# (xmin, xmax, ymin, ymax), applied to the original corner coordinates.
REGIONS = [
    (149.675, 149.875, 115.0, 142.0),
    (149.675, 149.875, 67.0, 90.0),
]
# ------------------------------------------------------------------------

EPS = 1e-9
TOKEN = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))")


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def length(v):
    return math.hypot(*v)


def mix(a, b, t):
    return tuple(x + t * (y - x) for x, y in zip(a, b))


def direction(a, b):
    v = sub(b, a)
    d = length(v)
    return (v[0] / d, v[1] / d), d


def signed_turn(u, v):
    return math.atan2(cross(u, v), u[0] * v[0] + u[1] * v[1])


def selected(p):
    return REGIONS is None or any(a <= p[0] <= b and c <= p[1] <= d
                                  for a, b, c, d in REGIONS)


def parse(lines):
    """Reject unsupported input rather than corrupt modal or extrusion data."""
    pos = [None, None, None]
    records = []
    for number, raw in enumerate(lines, 1):
        code = re.sub(r"\([^)]*\)", "", raw.split(";", 1)[0]).strip()
        if not code:
            records.append((raw, None, tuple(pos), False))
            continue
        tokens = list(TOKEN.finditer(code))
        remainder = TOKEN.sub("", code).strip()
        if remainder or not tokens:
            raise ValueError(f"Unsupported syntax at line {number}: {raw}")
        words = [(m[1].upper(), float(m[2])) for m in tokens]
        if (words[0][0] != "G" or words[0][1] not in (0, 1)
                or any(k not in "XYZ" for k, _ in words[1:])
                or len({k for k, _ in words}) != len(words)):
            raise ValueError(f"Line {number}: only explicit G0/G1 XYZ motion is supported. "
                             "Use the cleaned absolute-mm file BEFORE extrusion conversion.")
        before = tuple(pos)
        for key, value in words[1:]:
            if not math.isfinite(value):
                raise ValueError(f"Non-finite coordinate at line {number}")
            pos["XYZ".index(key)] = value
        p = tuple(pos)
        eligible = False
        if words[0][1] == 1 and None not in before and None not in p:
            d = length(sub(p, before))
            eligible = d > EPS and abs(p[2] - before[2]) <= MAX_Z_SLOPE * d
        records.append((raw, int(words[0][1]), p, eligible))
    return records


def proper_cross(a, b, c, d):
    # Contacts within 1e-8 mm of collinearity are not counted as crossings.
    if (max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0])
            or max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1])):
        return False
    u, v = sub(b, a), sub(d, c)
    return (cross(u, sub(c, a)) * cross(u, sub(d, a)) < -1e-16
            and cross(v, sub(a, c)) * cross(v, sub(b, c)) < -1e-16)


def candidates(points, distances, k, radius, last_end):
    """Intersect offset lines of surrounding segments to find tangent circles.

    Both tangent points must lie on their actual source segments. Searching
    multiple segments avoids limiting the radius to the tiny tip segments.
    """
    u0, _ = direction(points[k - 1], points[k])
    v0, _ = direction(points[k], points[k + 1])
    sign = 1 if cross(u0, v0) > 0 else -1
    left = max(0, bisect.bisect_left(distances, distances[k] - MAX_SEARCH_DISTANCE_MM) - 1)
    right = min(len(points) - 2, bisect.bisect_right(distances, distances[k] + MAX_SEARCH_DISTANCE_MM))
    options = []
    for i in range(k - 1, left - 1, -1):
        a = points[i]
        u, lu = direction(a, points[i + 1])
        for j in range(k, right + 1):
            b = points[j]
            v, lv = direction(b, points[j + 1])
            turn = signed_turn(u, v)
            if sign * turn < math.radians(15) or abs(turn) > math.radians(175):
                continue
            n1, n2 = (-u[1] * sign, u[0] * sign), (-v[1] * sign, v[0] * sign)
            aa = (a[0] + radius * n1[0], a[1] + radius * n1[1])
            bb = (b[0] + radius * n2[0], b[1] + radius * n2[1])
            den = cross(u, v)
            delta = sub(bb, aa)
            t, q = cross(delta, v) / den, cross(delta, u) / den
            if not (0 <= t <= lu and 0 <= q <= lv):
                continue
            start, end = distances[i] + t, distances[j] + q
            if (start <= last_end + EPS or start >= distances[k] - EPS
                    or end <= distances[k] + EPS
                    or max(distances[k] - start, end - distances[k]) > MAX_SEARCH_DISTANCE_MM):
                continue
            # A tiny flat at a tip can split one turn into two sharp vertices.
            # Allow those to share a fillet, but protect other nearby corners.
            conflict = False
            for h in range(i + 1, j + 1):
                if h == k:
                    continue
                other_turn = signed_turn(direction(points[h - 1], points[h])[0],
                                         direction(points[h], points[h + 1])[0])
                if abs(other_turn) > math.radians(45):
                    clustered = (abs(distances[h] - distances[k]) <= TIP_CLUSTER_MM
                                 and selected(points[h]) and sign * other_turn > 0)
                    if not clustered:
                        conflict = True
                        break
            if conflict:
                continue
            center = (aa[0] + t * u[0], aa[1] + t * u[1])
            p = mix(a, points[i + 1], t / lu)
            finish = mix(b, points[j + 1], q / lv)
            angle = math.atan2(p[1] - center[1], p[0] - center[0])
            step = min(MAX_CHORD_MM / radius,
                       2 * math.acos(max(-1.0, 1 - min(ARC_ERROR_MM / radius, 1.0))))
            count = max(2, math.ceil(abs(turn) / step))
            arc = [p]
            for h in range(1, count):
                f = h / count
                theta = angle + turn * f
                arc.append((center[0] + radius * math.cos(theta),
                            center[1] + radius * math.sin(theta),
                            p[2] + f * (finish[2] - p[2])))
            arc.append(finish)
            options.append((end - start, start, end, i, j, arc))
    return sorted(options, key=lambda x: x[0])


def process_run(points, line_numbers, all_segments, radius, stats):
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + length(sub(b, a)))
    replacements = []
    last_end = -1.0
    for k in range(1, len(points) - 1):
        if not selected(points[k]):
            continue
        turn = signed_turn(direction(points[k - 1], points[k])[0],
                           direction(points[k], points[k + 1])[0])
        if abs(turn) < math.radians(MIN_TURN_DEG):
            continue
        stats['candidates'] += 1
        if distances[k] <= last_end + EPS:
            stats['covered'] += 1
            continue
        accepted = None
        trial_radii = [radius]
        if ADAPT_RADIUS and radius > MIN_FILLET_RADIUS_MM:
            trial = radius - RADIUS_STEP_MM
            while trial > MIN_FILLET_RADIUS_MM + EPS:
                trial_radii.append(trial)
                trial -= RADIUS_STEP_MM
            trial_radii.append(MIN_FILLET_RADIUS_MM)
        for trial_radius in trial_radii:
            for option in candidates(points, distances, k, trial_radius, last_end):
                _, start, end, i, j, arc = option
                excluded = set(line_numbers[i + 1:j + 2])
                if CHECK_XY_CROSSINGS:
                    # Compare with unaffected original segments plus already fitted arcs.
                    other = [(a, b) for n, a, b in all_segments if n not in excluded]
                    for previous in replacements:
                        other.extend(zip(previous[5], previous[5][1:]))
                    xmin, xmax = min(p[0] for p in arc), max(p[0] for p in arc)
                    ymin, ymax = min(p[1] for p in arc), max(p[1] for p in arc)
                    other = [(a, b) for a, b in other
                             if max(a[0], b[0]) >= xmin and min(a[0], b[0]) <= xmax
                             and max(a[1], b[1]) >= ymin and min(a[1], b[1]) <= ymax]
                    if any(proper_cross(a, b, c, d) for a, b in zip(arc, arc[1:]) for c, d in other):
                        continue
                accepted = option
                break
            if accepted is not None:
                stats['radii'].append((line_numbers[k], trial_radius))
                break
        if accepted is None:
            stats['skipped'].append(line_numbers[k])
        else:
            replacements.append(accepted)
            last_end = accepted[2]
            stats['fitted'] += 1
    if not replacements:
        return None
    result = []
    index = 1  # Starting point was already emitted by the caller.
    for _, start, end, i, j, arc in replacements:
        while index < len(points) and distances[index] < start - EPS:
            result.append(points[index])
            index += 1
        result.extend(arc)
        while index < len(points) and distances[index] <= end + EPS:
            index += 1
    result.extend(points[index:])
    return result


def transform(lines, radius=FILLET_RADIUS_MM):
    if not math.isfinite(radius) or radius < 0:
        raise ValueError('FILLET_RADIUS_MM must be finite and >= 0.')
    if min(MAX_SEARCH_DISTANCE_MM, MAX_CHORD_MM, ARC_ERROR_MM) <= 0:
        raise ValueError('Search distance, chord length and arc error must be positive.')
    if (not math.isfinite(MIN_FILLET_RADIUS_MM) or MIN_FILLET_RADIUS_MM <= 0
            or not math.isfinite(RADIUS_STEP_MM) or RADIUS_STEP_MM <= 0):
        raise ValueError('Minimum radius and radius step must be finite and positive.')
    records = parse(lines)
    stats = {'candidates': 0, 'fitted': 0, 'skipped': [], 'radii': [], 'covered': 0}
    if radius == 0:
        return list(lines), stats
    all_segments = [(i + 1, records[i - 1][2], r[2])
                    for i, r in enumerate(records) if i and r[3]]
    output = []
    i = 0
    while i < len(records):
        if not records[i][3] or i == 0:
            output.append(records[i][0])
            i += 1
            continue
        end = i
        while end < len(records) and records[end][3]:
            end += 1
        points = [records[i - 1][2]] + [r[2] for r in records[i:end]]
        numbers = list(range(i, end + 1))  # One-based source line numbers.
        fitted = process_run(points, numbers, all_segments, radius, stats)
        if fitted is None:
            output.extend(r[0] for r in records[i:end])
        else:
            # Include new geometry when checking later motion runs. Keeping
            # original geometry too makes this guard deliberately conservative.
            previous_point = points[0]
            for point in fitted:
                all_segments.append((-1, previous_point, point))
                previous_point = point
            # Retain comments from rewritten source lines, without motion duplication.
            for r in records[i:end]:
                comments = re.findall(r'\([^)]*\)', r[0])
                if ';' in r[0]:
                    comments.append(r[0].split(';', 1)[1])
                if comments:
                    output.append('; Source comment: ' + ' '.join(comments))
            for p in fitted:
                output.append('G1 ' + ' '.join(f'{axis}{value:.{DECIMALS}f}'
                                               for axis, value in zip('XYZ', p)))
        i = end
    return output, stats


def main():
    # Validate every path before producing either output.
    inputs = [source.resolve() for source, _ in FILE_PAIRS]
    outputs = [target.resolve() for _, target in FILE_PAIRS]
    if any(target in inputs for target in outputs) or len(set(outputs)) != len(outputs):
        raise ValueError('Outputs must be distinct and must not overwrite either input.')
    missing = [str(source) for source, _ in FILE_PAIRS if not source.is_file()]
    if missing:
        raise FileNotFoundError('Missing input file(s):\n  ' + '\n  '.join(missing)
                                + '\nCheck BASE_DIRECTORY and FILE_PAIRS at the top of this script.')
    # Parse and process both successfully before writing the results.
    prepared = []
    for source, target in FILE_PAIRS:
        lines = source.read_text(encoding='utf-8-sig').splitlines()
        result, stats = transform(lines, FILLET_RADIUS_MM)
        prepared.append((source, target, result, stats))
    for source, target, result, stats in prepared:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('\n'.join(result) + '\n', encoding='utf-8')
        print(f'Input: {source}')
        print(f'Output: {target}')
        print(f'Preferred XY radius: {FILLET_RADIUS_MM:g} mm; adaptive: {ADAPT_RADIUS}')
        reduced = [(line, r) for line, r in stats['radii'] if r < FILLET_RADIUS_MM - EPS]
        if reduced:
            print('Smaller fillets used (source line: radius in mm):')
            print(', '.join(f'{line}: {r:.3f}' for line, r in reduced))
        print(f"Selected sharp corners: {stats['candidates']}; fillet arcs: {stats['fitted']}; extra vertices covered: {stats['covered']}; unchanged: {len(stats['skipped'])}")
        if stats['skipped']:
            print('Unchanged source line numbers (radius/space/crossing constraints):')
            print(', '.join(map(str, stats['skipped'])))
        print()
    print('Preview track spacing and coverage. Z is interpolated; regenerate extrusion from the new path.')


if __name__ == '__main__':
    main()
