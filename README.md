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
- 
---

## Author

**Arash Afshari** — Institute of Mechanical Engineering

## Citation

If you use this software in your research, please cite it. See
[`CITATION.cff`](CITATION.cff) — the DOI and publication details will be filled
in once the accompanying paper is available.

## License

[MIT](LICENSE)
