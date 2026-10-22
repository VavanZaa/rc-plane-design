# Phase 3 - Aerodynamic Baseline

## Objective

Create a reproducible, full-aircraft starting point before any NeuralFoil/Kulfan optimization. This phase establishes the geometry, mass case, controls, center of gravity, and conventional-tail stability model that the optimization must preserve or improve.

## Baseline configuration

| Item | Baseline |
|---|---:|
| Configuration | High-wing tractor taildragger |
| Motor | Single 2212, 1000 KV |
| Battery | 3S 2200 mAh |
| ESC | 30 A |
| Component estimate | 400 g |
| Printed-airframe target | 300-400 g |
| Design all-up mass | 800 g |
| Wingspan | 1,400 mm |
| Wing area | approximately 30.36 dm2 |
| Mean aerodynamic chord | approximately 221 mm |
| Initial CG | 63 mm aft of the root leading edge |
| Cruise speed | 13 m/s |
| Controls | Two ailerons, elevator, rudder |

## Main-wing stations

The sixth station terminates the aileron while preserving the five-point aerodynamic planform.

| Semispan station | Chord | Twist | Airfoil |
|---:|---:|---:|---|
| 0 mm | 250 mm | 0.0 deg | NACA 2414 |
| 175 mm | 245 mm | -0.2 deg | NACA 2414 |
| 350 mm | 230 mm | -0.7 deg | NACA 2414 |
| 525 mm | 195 mm | -1.3 deg | NACA 2413 |
| 650 mm | 159.3 mm | -1.8 deg | NACA 2412 |
| 700 mm | 145 mm | -2.0 deg | NACA 2412 |

The quarter-chord line is straight. To keep both 1,000 mm carbon tubes intact and unbent, the central panel is flat through y = 500 mm. Only the unsupported outer 200 mm tip panel rises, using 12 degrees baseline dihedral. This gives approximately the same total tip rise as 3.5 degrees across the complete semispan while preserving two straight, continuous spars. Phase 4 may vary the outer-panel angle between 10 and 14 degrees.

The wing root reference plane is 55 mm above the fuselage centerline. This high-wing position restores the stronger pendulum contribution to roll stability and provides more ground clearance during hard landings.

The MG90S servo station is at 350 mm semispan. Its gross section depth is 32.2 mm from a 230 mm chord and 14% airfoil, exceeding the 31.5 mm Phase 3 requirement. The forward spar sits ahead of the servo at 25% chord.

## Wing spars

The wing uses exactly two identical 6x4 mm carbon members. Both are 1,000 mm long, run parallel to the span axis, and are centered on the aircraft:

- forward spar on the constant X/Z = 292.5/55.0 mm centerline (25% root chord);
- rear spar on the constant X/Z = 380.0/57.0 mm centerline (60% root chord).

The two constant-X centerlines make the straight 1 m tubes parallel to one another. The spars cover the central 500 mm of each 700 mm semispan. The outer 200 mm of each tip has no carbon member, so the printed diamond lattice, shell, and segment joints in that region must be verified in the structural phase. The battery bay should end ahead of the forward carry-through; the receiver can sit behind the rear carry-through.

## Wing-to-fuselage connection

The removable wing is clamped to a conformal high-wing saddle by four M3 bolts at X/Y = (260/+/-28) mm and (445/+/-28) mm. Each bolt passes through an 8 mm OD compression sleeve and a reinforced 10 mm OD hardpoint; it must never clamp directly through an unsupported LW-PLA shell. Two fuselage bulkheads sit directly below the two spar centerlines so lift and landing loads pass from the carbon tubes into the fuselage sides.

The saddle has a minimum 34 mm contact half-width, with raised shoulders or fitted pockets to resist fore-aft and side motion. The screws provide clamp force and retention rather than being the only shear-carrying feature. A clear 46 mm central corridor remains between the hardpoint bosses for the 44 mm-wide equipment envelope.

Full Fusion 360 dimensions, print-material requirements, tolerances, and assembly order are in `WING_FUSELAGE_INTERFACE.md`.

## Conventional tail

- Horizontal-tail span: 650 mm
- Horizontal-tail root/tip chord: 120/85 mm
- Elevator hinge: 70% chord
- Vertical-tail height: 275 mm
- Vertical-tail root/tip chord: 125/75 mm
- Rudder hinge: 65% chord
- Nominal aerodynamic tail arm: approximately 550 mm

## Phase 3 acceptance gates

The generated summary must show:

1. Lift equal to the 800 g design weight at 13 m/s.
2. Cruise pitch trim with elevator magnitude below 8 degrees.
3. Negative pitch-stability derivative `Cm_alpha`.
4. Negative roll derivative `Cl_beta`.
5. Positive yaw derivative `Cn_beta`.
6. Wing-servo gross depth of at least 31.5 mm.
7. Horizontal and vertical tail-volume coefficients reported from the actual geometry.
8. Two spar-aligned fuselage bulkheads and four M3 mounting hardpoints are defined.
9. Fastener bosses remain inside the fuselage and at least 20 mm clear of spars and wing edges.
10. A minimum 44 mm clear equipment corridor remains between the hardpoints.

These are baseline checks, not final certification. Phase 4 now adds the multipoint optimization, structural constraints, root-first stall constraints, and high-resolution convergence checks; see `PHASE_4.md` for the accepted result.
