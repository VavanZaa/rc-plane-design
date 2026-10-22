# V-201 RC Plane

V-201 is a 1.4 m, single-motor, high-wing sport trainer derived from the lessons learned on V-101. The design target is stable manual flight, a gentle stall, straightforward maneuvering, basic aerobatics, and replaceable printed structure.

## Current status

Phase 4 (multipoint NeuralFoil/Kulfan optimization) is implemented and passes its acceptance gates. The accepted aircraft includes:

- an approximately elliptical 1.4 m main wing with a flat 1,000 mm spar panel and raised outer tips;
- a conventional horizontal and vertical tail;
- a simplified aerodynamic fuselage;
- the initial center of gravity;
- two ailerons, elevator, and rudder;
- the MG90S wing-servo thickness requirement;
- two parallel, 1,000 mm long, 6x4 mm carbon wing spars;
- a four-bolt, spar-supported removable high-wing saddle;
- an 800 g all-up design case;
- automated cruise trim and static-stability checks;
- independent large-NeuralFoil stall-order verification;
- coarse/medium/high VLM convergence checks; and
- a conditional 4-5 m asphalt takeoff-run estimate.

The mass allocation is 400 g for components and 300-400 g for the printed airframe. The current design point uses the upper limit: 800 g total.

## Run Phase 4

From this directory:

```powershell
py -3.12 aerodynamic\v201_phase4_optimize.py
```

The script writes:

- `aerodynamic/results/v201_phase4_summary.json`
- `aerodynamic/results/v201_phase4_optimized_design.json`
- optimized station/Kulfan files under `optimization_results/`
- `cad_reference/v201_phase4_optimized_wing.html`

Use `--verify-only` to rerun the independent checks and exports without repeating the MA97 solve.

See [PHASE_4.md](PHASE_4.md) for accepted dimensions and results, [PHASE_3.md](PHASE_3.md) for the baseline, and [WING_FUSELAGE_INTERFACE.md](WING_FUSELAGE_INTERFACE.md) for the Fusion 360 mounting geometry.
