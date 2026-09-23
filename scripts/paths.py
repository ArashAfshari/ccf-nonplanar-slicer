"""Repo-relative folder locations shared by every pipeline step.

Every step resolves its inputs and outputs from the repository root instead of
an absolute path on one machine, so the pipeline runs unchanged after a clone.

The layout mirrors the pipeline:

    data/00_input/    raw entry points (NX motion G-code, PrusaSlicer support job)
    data/01_split/    steps 01-03, one subfolder per source
    data/02_aligned/  step 04, translated and rotated onto the bed
    data/03_modules/  steps 05, 06 and 08, single-layer toolpath modules
    data/04_jobs/     steps 07, 09 and 10, complete printer jobs

Override any of these by editing the constants below, or point an individual
step somewhere else by editing the settings block at the top of that step.
"""

from pathlib import Path

# scripts/paths.py -> scripts/ -> repository root
REPO_ROOT = Path(__file__).resolve().parent.parent

DATA_DIR = REPO_ROOT / "data"
EXAMPLES_DIR = REPO_ROOT / "examples"

INPUT_DIR = DATA_DIR / "00_input"
SPLIT_DIR = DATA_DIR / "01_split"
ALIGNED_DIR = DATA_DIR / "02_aligned"
MODULES_DIR = DATA_DIR / "03_modules"
JOBS_DIR = DATA_DIR / "04_jobs"


def ensure_dirs(*directories: Path) -> None:
    """Create the given folders if they do not exist yet."""
    for directory in directories:
        directory.mkdir(parents=True, exist_ok=True)
