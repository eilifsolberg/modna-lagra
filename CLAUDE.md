# CLAUDE.md

## Project

Modna Lagra ("ripe storage"): a DIY-prototype machine that scans avocados with a
multispectral camera, stores them on sloped shelves in temperature zones, and
moves them with an X/Z elevator at the back to customer-facing sections on top.
Should generalise to other fruit (mango, tomato, kiwi, pear). Full requirements
are in `README.md` (Norwegian).

## Layout

- `src/modna/lagra/avocado_manager.py` — parametric concept model of the whole
  machine in build123d. One flat script: PARAMETERS → DERIVED QUANTITIES →
  geometry sections (housing, rod-bed shelves with software-chosen lanes and
  a loading drawer, top section, elevator with rigid-chain fork carriage,
  an imaging drawer under the loading drawer with a
  swappable imaging module: spin rollers on a load cell, side camera on a
  distance slide)
  → STEP export + capacity report. Per-level temperature zones are left out
  for now.
- `src/modna/lagra/rigid_chain.py` — the fork's 3D-printable rigid chain
  (rigid downwards, curls upwards, with a keel that runs between the shelf
  rods): link geometry, joint check, load/droop
  model. The machine model imports it (`import rigid_chain as rc`) for the
  chain, sprocket, vertical magazine and the report. Run on its own it writes
  printable STLs (link, sprocket, bench clamp) and a bench-test STEP to
  `./rigid_chain_print/`.
- `camera-holder/` — Blender/STL/3MF/G-code files for a 3D-printed Raspberry Pi
  camera + LED holder. Versions are separate files (`_v2` … `_v8`); the
  changelog and print workflow (Blender → PrusaSlicer → Prusa printer) are in
  `camera-holder/README.md`. Binary files — don't edit, and don't commit
  unless asked.

## Commands

Uses `uv` with Python 3.14.

- Run the model: `uv run python src/modna/lagra/avocado_manager.py`
  — writes `avocado_manager.step` to the current directory, prints the
  capacity report, and shows the model in the VS Code OCP CAD Viewer if its
  panel is open (otherwise prints a note and continues).
- Chain parts + bench test: `uv run python src/modna/lagra/rigid_chain.py`
  — after changing link geometry the joint check must report 0 overlap
  curled up and a positive overlap bent down.
- Format/lint: `uvx ruff format` and `uvx ruff check` (ruff is used via the
  VS Code extension; format on save is enabled).

There are no tests. To verify a change to the model, run the script and check
that it exits cleanly and the capacity report numbers make sense.

## Conventions

- Code, comments and part labels are in English; `README.md` is Norwegian.
- Units are millimetres. Axes: X = width (left→right), Y = depth
  (0 = front/customer, D = back/elevator), Z = height (0 = floor).
- All dimensions go in the UPPER_CASE parameters block at the top of the
  script; derived values are computed from them — don't hard-code numbers
  that should follow a parameter.
- Add geometry via `add(shape, label, color)` so every part gets a name and
  colour in the viewer tree. `box(x, y, z, dx, dy, dz)` builds a box from its
  min corner.
- Keep the capacity/mechanism report at the end in sync when adding
  parameters that affect fit or capacity.
