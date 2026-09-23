# ccf-nonplanar-slicer

Non-planar G-code generation for continuous carbon fiber (CCF) and thermoplastic
matrix on an [Anisoprint](https://anisoprint.com/) composite printer.

The toolchain starts from a curved, non-planar toolpath exported as pure motion
G-code from Siemens NX, splits it into fiber and polymer regions, converts each
region into Anisoprint extrusion commands, and assembles the result into a
complete multilayer printer job — optionally preceded by a PETG support
structure sliced in PrusaSlicer, and optionally replicated across the bed.

Ten scripts, run in order. Pure Python standard library, no dependencies.

> **Safety.** The output drives a real printer with a composite nozzle and a
> fiber cutter. Always preview the generated G-code and check nozzle and
> carriage clearance before printing. The built-in bounds and overlap checks
> concern commanded coordinates, not the physical size of the carriage.

---

## Pipeline

| Step | Script | What it does |
|-----:|--------|--------------|
| 01 | [`01_split_material_regions.py`](scripts/01_split_material_regions.py) | Strips the NX export down to pure G0/G1 XYZ motion, detects the closed deposition loops, and splits them into PETG outer boundary, CCF region, and PETG inner boundary. Also writes the full cleaned file, reused as the solid bottom/top surfaces. |
| 02 | [`02_fillet_sharp_edges.py`](scripts/02_fillet_sharp_edges.py) | Rounds sharp corners into G1 chord arcs so the fiber is not asked to turn faster than it can bend. Adaptive: falls back to smaller radii and reports what it used. |
| 03 | [`03_translate_paths.py`](scripts/03_translate_paths.py) | Translates every source in XY onto the print bed. |
| 04 | [`04_rotate_paths.py`](scripts/04_rotate_paths.py) | Rotates all files about one shared XY center, so the fiber and polymer paths stay aligned with each other and across sources. |
| 05 | [`05_generate_ccf_extrusion.py`](scripts/05_generate_ccf_extrusion.py) | Converts the fiber motion into Anisoprint CCF commands: fiber feed `U`, matrix feed `V`, and the cutter trigger placed ahead of each strand end. |
| 06 | [`06_generate_petg_extrusion.py`](scripts/06_generate_petg_extrusion.py) | Converts the three PETG motion files into absolute-`E` extrusion, with travel retraction and recovery. |
| 07 | [`07_assemble_multilayer_job.py`](scripts/07_assemble_multilayer_job.py) | Stacks the single-layer modules into a complete job: solid PETG end layers, CCF-reinforced layers between, tool changes, temperatures, fans, start/end code. |
| 08 | [`08_convert_support_toolpath.py`](scripts/08_convert_support_toolpath.py) | Converts a PrusaSlicer PETG support job into an Anisoprint PETG toolpath module (arcs linearized, Prusa-specific commands removed). |
| 09 | [`09_assemble_support_job.py`](scripts/09_assemble_support_job.py) | Prints the support first, then the composite part, in one job — carrying extrusion state across the join and checking for nozzle/part overlap. |
| 10 | [`10_replicate_build_plate.py`](scripts/10_replicate_build_plate.py) | Replicates a finished job into several copies at different XY offsets. |

Extrusion math, in brief:

```
PETG     dE = L * extrusion_width * layer_height / filament_area * flow
CCF      dU = L * fiber_per_mm
         dV = L * matrix_per_mm
```

where `L = sqrt(dx² + dy² + dz²)` — the **full 3D** segment length, which is what
makes the result correct on non-planar paths where `dz` is not zero.

---

## Requirements

- **Python 3.10 or newer.** No third-party packages.
- Siemens NX (or any source producing absolute, millimetre, motion-only G-code)
  for the non-planar toolpath.
- PrusaSlicer, only if you want the PETG support in steps 08–09.

---

## Quick start

```bash
git clone https://github.com/ArashAfshari/ccf-nonplanar-slicer.git
cd ccf-nonplanar-slicer
cp examples/*.gcode data/00_input/
```

On Windows PowerShell, use `Copy-Item examples\*.gcode data\00_input\` for the last line.

Then run the steps in order:

```bash
python scripts/01_split_material_regions.py
python scripts/02_fillet_sharp_edges.py
python scripts/03_translate_paths.py
python scripts/04_rotate_paths.py
python scripts/05_generate_ccf_extrusion.py
python scripts/06_generate_petg_extrusion.py
python scripts/07_assemble_multilayer_job.py
```

That produces `data/04_jobs/ccf_petg_multilayer.gcode`.

For the support-first job, additionally:

```bash
python scripts/08_convert_support_toolpath.py
python scripts/09_assemble_support_job.py
```

which produces `data/04_jobs/support_then_ccf_petg_multilayer.gcode`.

To print several copies, put the finished job(s) in `data/04_jobs/single_part/`
and run:

```bash
python scripts/10_replicate_build_plate.py
```

---

## Repository layout

```
scripts/           the ten pipeline steps, plus paths.py
examples/          reference inputs, enough to run the whole pipeline
data/
  00_input/          where you put your own inputs
  01_split/          steps 01-03, one subfolder per source
  02_aligned/        step 04, translated and rotated onto the bed
  03_modules/        steps 05, 06 and 08, single-layer toolpath modules
  04_jobs/           steps 07, 09 and 10, complete printer jobs
```

Everything under `data/` is generated and is not tracked by git.

---

## Configuration

Every script resolves its paths from the repository root through
[`scripts/paths.py`](scripts/paths.py), so the pipeline runs unchanged after a
clone. There is no config file: each step has a clearly marked settings block at
the top of the file, which is where all process parameters live.

The settings you are most likely to change:

| Where | Setting | Meaning |
|-------|---------|---------|
| step 01 | `SAMPLES` | Which NX exports to process, and how many loops go to the outer boundary and to the fiber |
| step 02 | `FILLET_RADIUS_MM`, `REGIONS` | Corner radius, and which corners to round |
| step 03 | `DEFAULT_X_OFFSET`, `DEFAULT_Y_OFFSET` | Position on the bed |
| step 05 | `CCF_WIDTH_MM`, `CCF_HEIGHT_MM`, `FIBER_DIAMETER_MM`, `EXTRUSION_MULTIPLIER` | Composite cross-section and flow |
| step 06 | `EXTRUSION_WIDTH_MM`, `DEPOSITION_HEIGHT_MM`, `FLOW_MULTIPLIER` | PETG bead and flow |
| step 07 | `NUMBER_OF_LAYERS`, `PETG_ONLY_END_LAYERS`, `LAYER_HEIGHT` | Layer structure, temperatures, fans, feed scaling |
| step 10 | `NUMBER_OF_SAMPLES`, `SAMPLE_OFFSETS_XY`, `MACHINE_LIMITS` | Bed arrangement |

---

## Notes and limitations

**The reference geometry is baked into some settings.** Two places hold values
measured from the reference part and will not transfer to different geometry:

- Step 02's `REGIONS` are XY windows around the sharp corners of the reference
  part, in its pre-translation, pre-rotation coordinates. Set `REGIONS = None`
  to consider every corner, or measure your own windows from the step 01 output.
- Step 01's loop counts depend on how many concentric tracks NX generated. Run
  it once, read the loop listing it prints, then set the counts.

**Two sources, different roles.** The reference pipeline takes the *fiber* track
from `nx_nonplanar_ccf_source.gcode` and the *PETG* geometry from
`nx_nonplanar_petg_source.gcode`, because their loop allocations differ. This is
why both files are in `examples/`. The source used is named explicitly at the
top of steps 05 (`CCF_SAMPLE`) and 06 (`PETG_SAMPLE`).

**Engage and retract classification.** Step 06 handles two path types with
different rules, selected per job: `VERTICAL_ONLY` (only a purely vertical move
next to a travel is material-free — for open inner and surface paths) and
`ANY_ADJACENT` (any G1 next to a travel is material-free — for the closed outer
boundary). Using the wrong mode either drops a bead or lays one down during
positioning.

**Z interpolation in filleting.** Step 02 interpolates Z linearly across each
fillet. This approximates the non-planar surface rather than reconstructing it
from CAD. Extrusion is regenerated afterwards from the changed 3D lengths, so
flow stays correct, but the surface deviates slightly inside each arc.

**Step 09's overlap check is point-based.** It compares deposition endpoints
against nearby support endpoints. It is a guard, not a proof of clearance.

---

## Author

**Arash Afshari** — Institute of Mechanical Engineering

## Citation

If you use this software in your research, please cite it. See
[`CITATION.cff`](CITATION.cff) — the DOI and publication details will be filled
in once the accompanying paper is available.

## License

[MIT](LICENSE)
