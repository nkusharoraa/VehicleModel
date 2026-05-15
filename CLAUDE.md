# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Vehicle dynamics modeling library (steering geometry, suspension kinematics, wheel loads, steering effort). Pure-Python numerical code built on `numpy`, `scipy`, `scikit-learn`, `matplotlib`, `pandas`/`openpyxl`. No build system, no test suite — analysis is driven from `Playground.ipynb` and ad-hoc scripts.

Install deps with `pip install -r requirements.txt`. Always run from the repo root so that `Instances.<Quad|Twin>.<file>` imports resolve correctly (e.g. `python -c "from Instances.Quad.QUTE_CNG35_Kerb import instance"`).

## Three Vehicle classes — they are NOT interchangeable

The repo defines three top-level modules, each containing its own `Vehicle` class with overlapping but **different `__init__` signatures**. Pick the one matching the suspension/steering topology being modeled; every instance file in `Instances/` already imports the correct one.

| Module | Use for | Distinguishing constructor args |
|---|---|---|
| `VehicleModel4Wh.py` | Standard 4-wheeler with double-wishbone or MacPherson strut + rack-and-pinion | `r_A, r_B, r_C, r_O, r_K`, optional `r_La/Lb/Ua/Ub/strut`, `twf`/`twr` |
| `VehicleModelModular.py` | Linkage-driven steering (bellcrank + intermediate column) | adds `pivot1`, `pivot2`, `cibj`, `cobj`, `s1`, `s2`; **no `r_C`** |
| `VehicleModelTwin.py` | Twin/tandem-axle layout with additional tie-rod nodes | adds `r_lowermount`, `r_uppermount`, `r_newtierodobj`, `r_newtierodibj`; uses single `tw` (not twf/twr) |

When refactoring shared logic, edit all three — they were forked, not abstracted, so changes don't propagate.

## Where coordinates and parameters live

- **Vehicle-spec helpers** (wheelbase, track, tire stiffness, pinion, etc.): `Instances/Quad/base.py` and `Instances/Twin/base.py`. Classes: `QUTE`, `R129`, `Golf`, `Modular`, `ModularRE`, etc.
- **Tire models** (Pacejka coefficients, cornering stiffness vs. load, pneumatic trail): also in those `base.py` files — `MRF13570R1269S`, `CONTINENTAL12580R13`.
- **Concrete instances** (hard-point coordinates in mm + GVW + CG): `Instances/Quad/*.py` and `Instances/Twin/*.py`. Each file builds one `instance = Vehicle(...)` by importing the relevant `base` helpers and one of the three `Vehicle` classes. Import these from notebooks/scripts as `from Instances.Quad.QUTE_CNG35_Kerb import instance`.
- `Instances.txt` is a legacy/scratch dump of the same classes — not imported anywhere; don't edit it expecting it to take effect.

## Architecture of the `Vehicle` class

Same shape in all three modules:

1. **Static vs. dynamic states.** `__init__` builds two lightweight `VehicleState` objects via the `create_object` classmethod — one with static loaded radius `slr`, one with dynamic loaded radius `dlr`. `self.reference()` returns whichever is active, selected by `self.dynamic_analysis` (0 = static, 1 = dynamic). Most geometry methods read from `self.reference()`, so toggling `dynamic_analysis` reuses the same method bodies for both states.
2. **KPA-angle-indexed memoization.** Each `VehicleState` carries large arrays (`dpK`, `dpA`, `dpB`, `dpC`, `dpO`, `dpT`, `dpW`, `dpdz`, `dpfvsa`, `dpsvsa`) sized `int(mindp/step + maxdp/step + 1)` (default 50°+50°, step 0.1°) with `zeropos` as the origin. Every `curr_X(curr_KPA_angle)` method writes/reads its slot; calls are cheap on a hit and incremental on a miss (it walks 0.1° at a time from the nearest computed neighbor). **Don't call `curr_*` with arbitrary jumps** — the inductive scheme described in `README.md` requires neighbors to already be filled.
3. **Inverse mappings via regression + fsolve.** `regression_model()` fits polynomial+linear regressions over the memoized samples to invert road-steer↔KPA and rack↔KPA. `KPA_rotation_angle(road_steer)`, `rack_vs_road_steer`, `KPA_rotation_angle_vs_rack`, `road_steer_vs_rack` use those regressions as initial guesses, then refine with `fsolve`.
4. **`_safe_fsolve` wrapper.** All non-trivial fsolve calls should go through `self._safe_fsolve(func, x0, memo_key=...)`. It memoizes successful solutions in `self._fsolve_memo`, perturbs the guess and retries up to 3 times on non-convergence, and warns rather than raising on final failure. Use a `memo_key` whenever the same equation is solved repeatedly at varying KPA angles.
5. **Ground contact toggle.** `on_ground=False` (default) treats the vehicle as lifted (no `delta_z` feedback); `on_ground=True` engages the bump/droop coupling. The Twin model accepts the flag but ignores it.
6. **Dynamic load & effort calculation.** `staticsolve`/`dynamicsolve` solve the per-corner force balance; `static_steering_effort` / `dynamic_steering_effort` build on top via the mechanical-advantage chain (`tierod_force_*`, `rack_force_*`, `mechanical_advantage_*`). `trainslipangles()` runs in `__init__` to pre-warm slip-angle guesses across the steering range; if it slows iteration, narrow `mindp`/`maxdp` rather than skipping it.

## Auxiliary scripts

- **`UJPlots.py`** — Standalone CLI that prompts for upper/lower U-joint angles and plots θ and ω relationships. Built as a Windows EXE via PyInstaller (`UJPlots.spec`, output in `dist/UJPlots.exe`). Rebuild: `pyinstaller UJPlots.spec`.
- **`TireData.py`** — Fits Pacejka-style `CFprime` and `SATprime` curves from `tiretrainingdata.xlsx` using `scipy.optimize.curve_fit`. Prints optimal `B, C, D, E` parameters; paste those into a tire class in the relevant `base.py`.
- **`Playground.ipynb`** — Primary analysis surface (~7.7 MB). Not run by CI; manual exploration only. The `.gitignore` lists `*.ipynb`, but this file is already tracked, so commits will pick up output changes — clear outputs before committing if the goal is a code-only diff.

## Conventions worth knowing

- Coordinates are in **mm**, masses in **kg**, angles in **degrees** at the interface (radians inside trig). Speed enters in **km/h** and is converted to m/s in `__init__`.
- KPA angle convention: positive = steer one direction, negative = the other; the per-vehicle lock range comes from `dila` / `assumed_rack_stroke`.
- `r_strut == [0,0,0]` is a sentinel meaning "no strut, use upper A-arm (`r_Ua`/`r_Ub`)" inside `fvsa_equations` / `svsa_equations`. Don't change that sentinel without auditing every FVSA/SVSA branch.
- `.vscode/settings.json` points `python.analysis.extraPaths` at a Windows-only Adams plugin path — safe to ignore on Linux/macOS.
