# Session handover — resume here

Snapshot for picking up in a fresh Claude Code window (started 2026-07-03, Typhornis/Racemus).
Read this + the auto-loaded memory files, and you can continue seamlessly.

## TL;DR — what we're doing RIGHT NOW

Building a render script so we can visually SEE the pan/tilt turret move. Working file:
**`sim/scenes/phase1_view.py`** (a copy of the working `phase1_step1_pan.py`, being extended to
render). We are **mid "Piece 1" of the render build** — adding a sky dome. The user types the
code; Claude teaches/navigates verbosely.

### The render-build plan (5 pieces, user writes each)
1. **Sky dome** (DomeLight)  ← *IN PROGRESS.* Add `UsdLux` to the pxr import; add 3 dome lines
   (`Define` / `CreateIntensityAttr(1000.0)` / `CreateColorAttr(Gf.Vec3f(0.35,0.55,0.90))`) right
   after the `UsdPhysics.Scene.Define(...)` line. Pattern is identical to `phase0_sky_camera.py`.
2. **Camera aimed AT the turret** — the one new concept: `rep.create.camera(position=..., look_at=...)`.
   Turret center is ~`(0, 0.2, 0.35)`; a good camera pose is `position=(2.5, -2.0, 1.5)`,
   `look_at=(0.0, 0.2, 0.35)`. Then `render_product` + `rgb` annotator (same as Phase 0 capture).
3. **Render + save** — settle physics, `rep.orchestrator.step(...)`, `rgb_annot.get_data()`, save
   PNG with Pillow (same as `phase0_sky_camera.py`).
4. **Fix the settling bug** — in the earlier attempt the pose was captured too early (commanded
   60/45, only reached 9.2/6.9). Needs MORE `world.step(render=False)` iterations before capture,
   OR ensure physics keeps advancing. ~150 steps was not enough for a 60° move; try 300+.
5. **Two poses** — render pose A (0,0) and pose B (e.g. 60,45), save both, compare → "it moved."

Note: the turret-build code is currently duplicated (it's in `phase1_step1_pan.py` and would be
copied into `phase1_view.py`). Good later cleanup: extract `build_turret(stage)` into a reusable
module `sim/turret_rig.py` and import it in both. (Deferred — teaches Python functions/modules.)

## Project state

- **Phase 0 COMPLETE** — Isaac Sim 5.1.0 installed (pip, `.venv`, Python 3.11), PyTorch cu128 sees
  the RTX 3090, first sky+camera render works (`sim/scenes/_output/rgb_0000.png`).
- **Phase 1 IN PROGRESS** — 2-DOF pan/tilt turret WORKS (`sim/scenes/phase1_step1_pan.py`):
  base + pan joint (axis Z) + tilt joint (axis X, mounted on pan), driven to target angles, range
  limits from `sim/turret_config.py` (placeholder specs). Verified pan=30/tilt=45 → reached exactly.
- **Committed & pushed** through `72f3eef Adding Tilt` (confirm `git status` is clean/synced).
- Still TODO in Phase 1 after the render: wire max-SPEED limits (drive respects range, not
  `PAN/TILT_MAX_SPEED_DPS`); mount a camera on the tilt link; drone asset + scripted flight paths.

## How to run things (Windows)

- **Activate venv first** (per shell): PowerShell `.\.venv\Scripts\Activate.ps1` — prompt shows `(.venv)`.
- **Run a scene:** `python .\sim\scenes\<file>.py` (needs `(.venv)` active).
- **Headless needs** `OMNI_KIT_ACCEPT_EULA=YES` in the env (else it hangs on the license prompt).
- Full venv python path (for non-activated shells): `D:\source\Racemus\.venv\Scripts\python.exe`.

## Gotchas already learned (don't re-discover)

- **Kit swallows Python `print()`** — write results to a sidecar file (see the `log()` +
  `_output/*_status.txt` pattern in the scene scripts). This is how we read results.
- **Headless capture:** drive rendering with `rep.orchestrator.step()`, NOT `sim_app.update()`.
  First 1-2 steps return an empty array while RTX warms up — retry until `get_data()` is non-empty.
- **Isaac deprecation warnings** (`omni.isaac.* -> isaacsim.*`) are harmless noise from Isaac's own
  extensions, not our code.
- **IntelliSense:** works for numpy/math/stdlib; will NOT complete `isaacsim`/`pxr` calls (compiled
  libs, thin type stubs) — that's universal to Isaac, not broken. `docs/isaac-notes.md` is the
  "autocomplete" for Isaac calls.
- **Editor:** `.vscode/settings.json` set for 80-col ruler, IntelliSense on, NO auto-format
  (comments are hand-aligned into columns), no AI ghost-text.

## Working style (also in memory — READ IT)

Senior-dev/junior-dev mentorship. The user is a computer engineer, rusty, **weak on Python**, new
to Isaac/robotics. They learn by **writing code themselves** — the user types, Claude navigates.
**Be VERBOSE**: teach the concept, where it comes from, why it goes where it goes, what each line
does — THEN have them type it. Do NOT write whole files for them (has been a repeated mis-step).
Keep code <=80 cols with hand-aligned inline comments. See memory: `mentorship-working-style`,
`user-background`, `racemus-phase-progress`, and `docs/isaac-notes.md` (glossary).

## To resume in the new window

1. Open a fresh terminal (PowerShell or Git Bash), `cd D:\source\Racemus`, activate the venv.
2. Start Claude Code (`claude`). Your memory files load automatically.
3. Tell it: "Read docs/session-handover.md and let's continue." It'll pick up at Piece 1 above.
   (Use a NEW session for speed; `claude --continue` would reload this whole long conversation.)
