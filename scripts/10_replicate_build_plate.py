"""Step 10 - replicate a finished support-first Anisoprint job across the bed.

Takes the complete single-part job from step 09 and prints several copies of it
in one build, each at its own XY offset.

Supports are repeated by their explicit ;LAYER_CHANGE markers. Part phases
are repeated by ;MACROLAYER and tool-change markers, NEVER by Z height.
The original startup, sacrificial preparation and shutdown run once.
Original absolute extrusion values are replayed using G92 baselines, with
physical retract-state reconciliation before every block. G92 itself does
not retract or recover filament. U fiber advances and cutter commands are
preserved. The original source's M82 E/V/U convention is required.

This is a postprocessor for this annotated generator, not arbitrary G-code.
Preview the result and check nozzle/carriage clearance on the actual printer.
The optional bounds check concerns commanded coordinates, not carriage size.
"""
from pathlib import Path
from dataclasses import dataclass, field
import math
import re

from paths import JOBS_DIR

# ======================= USER SETTINGS =======================
# All .gcode files directly inside INPUT_FOLDER are processed independently.
# The output folder is created automatically. Subfolders are not scanned.
INPUT_FOLDER = JOBS_DIR / "single_part"
OUTPUT_FOLDER = JOBS_DIR / "multiple_parts"
# Output example: composite_part.gcode -> composite_part_3_samples.gcode
NUMBER_OF_SAMPLES = 3
# Translation from the ORIGINAL part center; (0, 0) keeps its original location.
# Number of entries must equal NUMBER_OF_SAMPLES. Add entries for more copies.
SAMPLE_OFFSETS_XY = [(0.0, 0.0), (0.0, 50.0), (0.0, -50.0)]
# Optional new center for the whole arrangement. None keeps the source center.
# Center is the XY bounding-box midpoint of part deposition (excluding support).
BUILD_CENTER_XY = None  # Example: (150.0, 150.0)
TRANSFER_CLEARANCE_MM = 5.0
TRANSFER_RETRACT_MM = 9.0  # Source uses 9 mm travel and 12 mm parked retract.
TRAVEL_FEED = 7200
Z_FEED = 900
RETRACT_FEED = 2100
MIN_SAMPLE_GAP_MM = 2.0
# Set to YOUR printer's usable limits to enforce travel/build bounds.
MACHINE_LIMITS = None  # Example format: ((xmin,xmax), (ymin,ymax), (zmin,zmax))
# =============================================================
WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))")


def parse(line):
    code = line.split(';', 1)[0].strip()
    return (code.split()[0] if code else ''), {k: float(v) for k, v in WORD.findall(code)}


@dataclass
class State:
    pos: dict = field(default_factory=lambda: dict(X=None, Y=None, Z=None))
    ext: dict = field(default_factory=lambda: dict(E=0., V=0., U=0.))
    depth: dict = field(default_factory=lambda: dict(E=0., V=0.))
    tool: int = 0
    feed: float = 1200.
    relative: bool = False

    def apply(self, line):
        cmd, w = parse(line)
        if cmd in ('T0', 'T1'):
            self.tool = int(cmd[1])
        if cmd == 'G91': self.relative = True
        if cmd == 'G90': self.relative = False
        if cmd == 'G92':
            for a in self.ext:
                if a in w: self.ext[a] = w[a]
        if cmd in ('G0', 'G1'):
            for a in self.pos:
                if a in w:
                    self.pos[a] = w[a] + ((self.pos[a] or 0.) if self.relative else 0.)
            for a in self.ext:
                if a in w:
                    delta = w[a] - self.ext[a]
                    if a in self.depth: self.depth[a] = max(0., self.depth[a] - delta)
                    self.ext[a] = w[a]
            if 'F' in w: self.feed = w['F']


def snapshot(s):
    return State(s.pos.copy(), s.ext.copy(), s.depth.copy(), s.tool, s.feed, s.relative)


def index_of(lines, text, start=0):
    return next(i for i in range(start, len(lines)) if text in lines[i])


def shifted(line, dx, dy):
    cmd, _ = parse(line)
    if cmd not in ('G0', 'G1'): return line
    code, sep, comment = line.partition(';')
    def change(m):
        a, v = m.groups()
        if a in ('X', 'Y'):
            return f'{a}{float(v) + (dx if a == "X" else dy):.5f}'
        return m.group(0)
    return WORD.sub(change, code) + sep + comment


def convert(input_file, output_file, offsets=None):
    offsets = SAMPLE_OFFSETS_XY if offsets is None else offsets
    if not offsets or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in offsets):
        raise ValueError('Provide finite XY offsets for at least one sample.')
    if TRANSFER_CLEARANCE_MM <= 0 or TRANSFER_RETRACT_MM < 0:
        raise ValueError('Invalid transfer settings.')
    input_file, output_file = Path(input_file), Path(output_file)
    if input_file.resolve() == output_file.resolve():
        raise ValueError('Input and output must be different files.')
    lines = input_file.read_text(encoding='utf-8-sig').splitlines()
    if not any('ANISOPRINT SUPPORT-FIRST MULTILAYER CUSTOM' in x for x in lines[:10]):
        raise ValueError('Expected the annotated Anisoprint support-first generator format.')
    support = index_of(lines, 'COMPLETE PETG SUPPORT / T0')
    bridge = index_of(lines, 'SUPPORT -> CCF PART')
    end = index_of(lines, 'ANISOPRINT END')
    macros = [i for i, l in enumerate(lines) if l.startswith(';MACROLAYER:')]
    layers = [i for i in range(support, bridge) if lines[i].strip() == ';LAYER_CHANGE']
    if not layers or not macros: raise ValueError('Missing explicit layer markers.')
    states, state = [], State()
    part_points, all_points, zmax = [], [], 0.
    commands = set()
    for i, line in enumerate(lines):
        states.append(snapshot(state))
        cmd, w = parse(line)
        commands.add(cmd)
        if i < end and (cmd in ('G91','M83','G2','G3') or (cmd == 'G92' and any(a in w for a in 'XYZ'))):
            raise ValueError(f'Unsupported coordinate/extrusion command at line {i+1}: {line}')
        if cmd and cmd not in ('G0','G1','G4','G21','G28','G90','G91','G92','M82','M104','M109','M140','M190','M400','M530','M532','M106','M1001','M1002','M1013','M280','T0','T1'):
            raise ValueError(f'Unrecognized command at line {i+1}: {cmd}')
        before = snapshot(state)
        state.apply(line)
        if support <= i < end and cmd in ('G0','G1') and all(v is not None for v in state.pos.values()):
            if not bridge <= i < macros[0]:
                all_points.append(tuple(state.pos[a] for a in 'XYZ'))
            deposition = any(w.get(a, before.ext[a]) > before.ext[a] + 1e-8 for a in 'EVU') and any(a in w for a in 'XY')
            if deposition:
                zmax = max(zmax, state.pos['Z'])
                if i >= macros[0]: part_points.append(tuple(state.pos[a] for a in 'XYZ'))
    if not part_points: raise ValueError('No part deposition detected.')
    center = tuple((min(p[a] for p in part_points)+max(p[a] for p in part_points))/2 for a in (0,1))
    origin_shift = (0., 0.) if BUILD_CENTER_XY is None else tuple(BUILD_CENTER_XY[a]-center[a] for a in (0,1))
    offsets = [(x+origin_shift[0], y+origin_shift[1]) for x,y in offsets]
    bounds = [(min(p[a] for p in all_points), max(p[a] for p in all_points)) for a in range(3)]
    boxes = [(bounds[0][0]+x,bounds[0][1]+x,bounds[1][0]+y,bounds[1][1]+y) for x,y in offsets]
    for i, b in enumerate(boxes):
        for c in boxes[:i]:
            gap = MIN_SAMPLE_GAP_MM
            if not (b[1]+gap <= c[0] or c[1]+gap <= b[0] or b[3]+gap <= c[2] or c[3]+gap <= b[2]):
                raise ValueError('Sample support/travel bounding boxes overlap or lack the configured gap.')
    safe_z = zmax + TRANSFER_CLEARANCE_MM
    out, actual = [], State()
    def emit(line):
        actual.apply(line)
        cmd, w = parse(line)
        if MACHINE_LIMITS and cmd in ('G0','G1'):
            for a, limits in zip('XYZ', MACHINE_LIMITS):
                if a in w and actual.pos[a] is not None and not limits[0] <= actual.pos[a] <= limits[1]:
                    raise ValueError(f'{a} outside configured limits: {line}')
        out.append(line)
    def reconcile(axis, target):
        # Physical move, separate from the subsequent logical G92 reset.
        delta = actual.depth[axis] - target
        if abs(delta) > 1e-7:
            emit(f'G1 {axis}{actual.ext[axis]+delta:.5f} F{RETRACT_FEED} ; match source retract depth')
    def block(a, b, offset=None, label=''):
        source = states[a]
        if actual.tool != source.tool:
            raise ValueError(f'Tool mismatch before source line {a+1}.')
        axis = 'E' if source.tool == 0 else 'V'
        emit(f'; MULTICOPY BEGIN {label} SOURCE {a+1}:{b}')
        if offset is not None:
            dx, dy = offset
            # Planar layer changes may begin with a wipe at the PREVIOUS endpoint.
            # Part/tool phases instead begin with a full, non-extruding approach.
            first = next((parse(l) for l in lines[a:b] if parse(l)[0] in ('G0','G1') and any(k in parse(l)[1] for k in 'XYZ')), None)
            pos = source.pos.copy()
            if a == layers[0] or a >= macros[0]:
                if first is None or first[0] != 'G0' or not all(k in first[1] for k in 'XYZ'):
                    raise ValueError(f'Expected full XYZ G0 approach at source line {a+1}.')
                pos = {k:first[1][k] for k in 'XYZ'}
            if any(v is None for v in pos.values()): raise ValueError('Unknown entry position.')
            reconcile(axis, max(TRANSFER_RETRACT_MM, source.depth[axis], actual.depth[axis]))
            lift = max(safe_z, actual.pos['Z'] or 0., pos['Z'])
            emit(f'G0 Z{lift:.5f} F{Z_FEED} ; transfer clearance')
            emit(f'G0 X{pos["X"]+dx:.5f} Y{pos["Y"]+dy:.5f} F{TRAVEL_FEED}')
            emit(f'G0 Z{pos["Z"]:.5f} F{Z_FEED}')
        reconcile(axis, source.depth[axis])
        emit('G92 ' + ' '.join(f'{k}{source.ext[k]:.5f}' for k in 'EVU') + ' ; logical source baseline only')
        emit(f'G1 F{source.feed:.5f} ; restore modal feedrate')
        for line in lines[a:b]: emit(shifted(line, *offset) if offset is not None else line)
        emit('; MULTICOPY END')
    for line in lines[:layers[0]]: emit(line)
    for k, (a,b) in enumerate(zip(layers, layers[1:]+[bridge]), 1):
        for n, off in enumerate(offsets, 1): block(a,b,off,f'SUPPORT {k} SAMPLE {n}')
    # Fixed machine-space preparation, purge and tool macros are not translated.
    block(bridge, macros[0], label='ONE-TIME CCF PREPARATION')
    phases = 0
    for layer_no, (a,b) in enumerate(zip(macros, macros[1:]+[end]), 1):
        changes = [j for j in range(a,b) if lines[j].strip() == '; Start change extruder']
        cursor = a
        for c in changes + [b]:
            if c > cursor:
                for n,off in enumerate(offsets,1): block(cursor,c,off,f'PART {layer_no} PHASE {phases+1} SAMPLE {n}')
                phases += 1
            if c != b:
                finish = index_of(lines, '; End change extruder', c) + 1
                block(c,finish,label=f'TOOL CHANGE LAYER {layer_no}')
                cursor = finish
    block(end,len(lines),label='ONE-TIME SHUTDOWN')
    tool_count = sum(parse(l)[0] in ('T0','T1') for l in out)
    header = ['; MULTI-SAMPLE BUILD: offsets from source center', f'; Source center XY: {center}',
              f'; Effective XY translations: {offsets}', f'; Transfer plane: {safe_z:.3f} mm',
              '; Source comments about filament totals apply to ONE sample only.']
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text('\n'.join(header+out)+'\n', encoding='utf-8')
    print(f'Created: {output_file}\nSamples: {len(offsets)}; support layers: {len(layers)}; macro-layers: {len(macros)}')
    print(f'Tool selections: {tool_count} (including startup); actual changes: {tool_count-1}')
    print(f'Original part center: {center}; transfer plane: {safe_z:.3f} mm')
    for n,b in enumerate(boxes,1): print(f'Sample {n}: X {b[0]:.3f}..{b[1]:.3f}, Y {b[2]:.3f}..{b[3]:.3f}')
    if MACHINE_LIMITS is None: print('Machine limits not configured: check bed limits and carriage clearance in preview.')
    return out


def main():
    if NUMBER_OF_SAMPLES != len(SAMPLE_OFFSETS_XY):
        raise ValueError('NUMBER_OF_SAMPLES must equal the number of SAMPLE_OFFSETS_XY entries.')
    if NUMBER_OF_SAMPLES < 1:
        raise ValueError('NUMBER_OF_SAMPLES must be at least 1.')
    if INPUT_FOLDER.resolve() == OUTPUT_FOLDER.resolve():
        raise ValueError('INPUT_FOLDER and OUTPUT_FOLDER must be different folders.')
    if not INPUT_FOLDER.is_dir():
        raise FileNotFoundError(f'Input folder does not exist: {INPUT_FOLDER}')
    inputs = sorted((p for p in INPUT_FOLDER.iterdir()
                     if p.is_file() and p.suffix.lower() == '.gcode'),
                    key=lambda p: p.name.lower())
    if not inputs:
        raise FileNotFoundError(f'No .gcode files found in: {INPUT_FOLDER}')
    outputs = [OUTPUT_FOLDER / f'{p.stem}_{NUMBER_OF_SAMPLES}_samples.gcode'
               for p in inputs]
    if len({str(p).lower() for p in outputs}) != len(outputs):
        raise ValueError('Input filenames would produce duplicate output names.')
    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    failures = []
    for input_file, output_file in zip(inputs, outputs):
        print(f'\nProcessing {input_file.name}')
        try:
            convert(input_file, output_file)
        except (ValueError, OSError, StopIteration) as exc:
            reason = str(exc) or 'Missing a required source G-code section marker.'
            failures.append((input_file.name, reason))
            print(f'FAILED: {input_file.name}: {reason}')
    print(f'\nBatch complete: {len(inputs)-len(failures)} succeeded; {len(failures)} failed.')
    if failures:
        print('Failed inputs (any older outputs for these files are not updated):')
        for name, reason in failures:
            print(f'  {name}: {reason}')
        raise SystemExit(1)


if __name__ == '__main__':
    main()
