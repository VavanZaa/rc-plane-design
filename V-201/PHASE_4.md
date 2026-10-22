# Phase 4 - Multipoint Wing Optimization

## Outcome

Phase 4 passes all aerodynamic, stability, packaging, convergence, and provisional spar-strength gates. The accepted design is a smooth, cubic-Bernstein planform with spanwise Kulfan airfoils optimized by IPOPT/MA97. NeuralFoil supplies viscous section aerodynamics and VLM supplies three-dimensional lift and induced drag.

The Phase 3 root and mounting geometry is locked. The optimizer did not move the 250 mm root chord, X = 230 mm root leading edge, Z = 55 mm high-wing reference plane, two carbon-spar centerlines, saddle, or four M3 fasteners.

## Accepted wing geometry

| Semispan y | Chord | Twist |
|---:|---:|---:|
| 0 mm | 250.000 mm | 0.000 deg |
| 175 mm | 246.172 mm | -0.072 deg |
| 350 mm | 230.625 mm | -0.650 deg |
| 500 mm | 203.353 mm | -1.000 deg |
| 600 mm | 176.007 mm | -1.645 deg |
| 650 mm | 159.156 mm | -2.100 deg |
| 700 mm | 140.000 mm | -2.500 deg |

The chord values are samples of one cubic Bernstein field, not independent polygon corners. This keeps the leading and trailing edges smooth when reconstructed in Fusion 360.

| Property | Accepted value |
|---|---:|
| Span | 1,400 mm |
| Wing area | 30.502 dm2 |
| Mean aerodynamic chord | 222.425 mm |
| Aspect ratio | 6.426 |
| Center-panel dihedral | 0 deg through y = 500 mm |
| Outer-tip dihedral | 12.031 deg from y = 500-700 mm |
| Approximate tip rise | 42.63 mm |
| Aileron span | y = 350-650 mm each side |
| Aileron hinge | 75% local chord |
| Physical trailing-edge thickness | 0.8 mm |

## Optimization method

- Three Kulfan/CST control airfoils are blended quadratically across the semispan.
- The chord field uses four cubic Bernstein control values, following the smooth-span method from V-101.
- Twist is constrained to become progressively more negative toward the tip.
- Both intact carbon tubes force the central 1,000 mm panel to remain flat. Only the outer 200 mm tip panels use the optimized dihedral angle.
- IPOPT uses the MA97 sparse linear solver.
- The objective combines takeoff at 6.8 m/s, endurance at 9.5 m/s, cruise at 13 m/s, and fast/gust flight at 17 m/s. The endurance condition has the highest weight.
- NeuralFoil optimization uses the `small` model; acceptance uses the independent `large` model with Ncrit = 5 for a conservative outdoor/printed-surface transition assumption.
- The accepted result is rerun with coarse, medium, and high VLM meshes.

## High-resolution wing performance

These values describe the wing plus the NeuralFoil profile-drag integration. The complete-aircraft values are lower because the tail and fuselage add drag.

| Point | Speed | Alpha | Drag | L/D | Change from Phase 3 drag |
|---|---:|---:|---:|---:|---:|
| Takeoff | 6.8 m/s | 9.980 deg | 0.508 N | 15.46 | -0.58% |
| Endurance | 9.5 m/s | 4.221 deg | 0.383 N | 20.47 | -2.59% |
| Cruise | 13.0 m/s | 1.447 deg | 0.421 N | 18.62 | +0.45% |
| Fast/gust | 17.0 m/s | 0.132 deg | 0.568 N | 13.80 | -1.63% |

The four-point weighted drag improves by approximately 1.29%. Cruise alone is essentially unchanged because Phase 4 prioritizes endurance, straight continuous spars, smooth geometry, servo clearance, structural depth, and gentle stall behavior over a single-speed drag minimum.

The medium-to-high VLM change is 0.565% for total drag and 2.10% for induced drag, within the respective 3% and 7% convergence gates.

## Complete-aircraft trim and stability

| Point | Alpha | Elevator | Complete-aircraft L/D |
|---|---:|---:|---:|
| Takeoff | 9.560 deg | -14.243 deg | 10.78 |
| Endurance | 3.148 deg | -5.645 deg | 12.88 |
| Cruise | 0.915 deg | -2.339 deg | 10.66 |
| Fast/gust | -0.346 deg | -0.708 deg | 7.85 |

The elevator should be designed for at least +/-20 degrees mechanical travel. The larger takeoff deflection is rotation authority; the +/-8 degree trim gate remains applicable to endurance, cruise, and fast flight.

Cruise static-stability derivatives remain correctly signed:

- `Cm_alpha = -3.1034 / rad`
- `Cl_beta = -0.1620 / rad`
- `Cn_beta = +0.1184 / rad`

## Gentle-stall result

The large NeuralFoil sweep predicts the root section reaches its maximum lift first. The earliest aileron/tip-region section stalls 1.50 degrees of aircraft incidence later than the center section, exceeding the 1.0 degree requirement. The stronger outer washout is intentional and must be preserved in Fusion 360.

Using 90% of the lowest predicted section CLmax as a conservative three-dimensional value gives:

- estimated stall speed: 6.27 m/s;
- target rotation speed: 6.80 m/s, or 1.084 times estimated stall speed;
- recommended initial approach speed: 7.83 m/s.

## Servo and spar constraints

At y = 350 mm, the optimized section provides 32.00 mm depth at 30% chord. This preserves 3.5 mm over the 28.5 mm MG90S body height and 0.5 mm over the earlier Phase 3 gross-envelope requirement.

Both 6 mm OD / 4 mm ID, 1,000 mm carbon tubes remain on the parallel X = 292.5 mm and X = 380.0 mm centerlines. The local chord fraction of the rear tube changes smoothly from 60.0% at the root to approximately 68.0% at y = 500 mm as the wing tapers. Both tubes retain more than the required 10 mm airfoil depth through their y = 500 mm endpoints, and the rear tube remains approximately 14.2 mm ahead of the 75%-chord aileron hinge there. The structural estimate at the 4 g design case gives:

The forward tube uses constant Z = 55 mm and the rear tube uses constant Z = 57 mm. With the sections twisted about the forward-spar reference, the worst tube-to-nearest-skin clearance is 2.69 mm at the rear spar endpoint. The Phase 4 minimum is 2.0 mm beyond the 3 mm tube radius.

| Check | Value |
|---|---:|
| Elliptical-load root bending moment | 4.662 N m |
| Forward spar stress at assumed 65% load share | 178.1 MPa |
| Rear spar stress at assumed 35% load share | 95.9 MPa |
| Required carbon bending strength including 1.5 safety factor | 267.1 MPa |
| Provisional allowable used by Phase 4 | 300 MPa |

The purchased tube must have a supplier bending-strength value of at least 267 MPa. If it does not, the provisional structural pass is invalid. The printed shell, lattice, glue joints, spar sleeves, and saddle are not represented by this beam calculation and still require the 3 g and 4 g proof tests in `WING_FUSELAGE_INTERFACE.md`.

## Run and reproduce

Run a new optimization:

```powershell
py -3.12 aerodynamic\v201_phase4_optimize.py
```

Rerun verification and exports from the saved MA97 checkpoint:

```powershell
py -3.12 aerodynamic\v201_phase4_optimize.py --verify-only
```

Primary outputs:

- `aerodynamic/results/v201_phase4_summary.json`
- `aerodynamic/results/v201_phase4_optimized_design.json`
- `optimization_results/v201_phase4_wing_stations.csv`
- `optimization_results/v201_phase4_spar_rails.csv`
- `optimization_results/v201_phase4_kulfan_coefficients.json`
- `optimization_results/airfoils/*.dat`
- `optimization_results/v201_phase4_geometry.png`
- `optimization_results/v201_phase4_stall_order.png`
- `cad_reference/v201_phase4_optimized_wing.html`
- `cad_reference/v201_phase4_optimized_wing_three_view.png`

The `.dat` files are normalized section references. They must not be imported directly into Fusion as final physical sketches. The later Fusion import phase must apply each station's chord, twist, dihedral, leading-edge position, and millimetre units from the station CSV/JSON.

## Takeoff-run condition

The conservative ground-roll estimate is 4.28 m at 600 g static thrust and 3.59 m at 700 g static thrust. It assumes 85% average thrust during the roll, asphalt rolling coefficient 0.04, and half of the rotation-point aircraft drag as average aerodynamic drag.

This result is conditional. Before relying on a 4-5 m runway, measure the installed motor, ESC, battery, and propeller on a thrust stand and confirm at least 600 g static thrust without exceeding the 30 A ESC or motor limits.
