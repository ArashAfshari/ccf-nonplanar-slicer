# Example inputs

Reference inputs for the non-planar composite sample. Together they are enough
to run the entire pipeline, steps 01 through 10.

Copy them into the working folder before the first run:

```bash
cp examples/*.gcode data/00_input/
```

```powershell
Copy-Item examples\*.gcode data\00_input\
```

| File | Feeds | Description |
|------|-------|-------------|
| `nx_nonplanar_petg_source.gcode` | step 01 | Non-planar toolpath exported from Siemens NX as pure motion G-code. Supplies the **PETG** geometry: outer boundary, inner boundary and the solid bottom/top surfaces. Step 01 writes its results to `data/01_split/petg_source/`. |
| `nx_nonplanar_ccf_source.gcode` | step 01 | The same part with a different loop allocation (one outer loop instead of two). Supplies the **CCF** fiber track, into `data/01_split/ccf_source/`. Both files are needed: see "Two sources, different roles" in the main README. |
| `prusaslicer_petg_support.gcode` | step 08 | PETG support structure sliced in PrusaSlicer for an MK4, 0.4 mm nozzle, 0.32 mm layers. Only needed for the support-first job in steps 08–09. |
| `support_body.stl` | — | The support body itself, so the PrusaSlicer job above can be re-sliced with different settings. Not read by any script. |

These files are kept here unchanged. `data/` is a scratch area and is not
tracked by git, so you can delete and regenerate it freely without losing them.
