# V-201 Wing-to-Fuselage Interface

## Load path

The complete 1.4 m wing is a removable assembly. Both 6 mm OD / 4 mm ID carbon tubes remain continuous and centered inside the wing. The forward line is X/Z = 292.5/55.0 mm and the rear line is X/Z = 380.0/57.0 mm through y = +/-500 mm. Both intact 1 m tubes are straight and parallel; the 2 mm rear vertical offset centers it safely inside the washed-out sections. The central 1,000 mm wing panel is flat; dihedral begins only in the unsupported outer 200 mm tips. The wing sits on a conformal saddle on top of the fuselage. A reinforced fuselage bulkhead is located directly below each carbon tube. This makes the primary load path:

`wing skin/lattice -> carbon tubes -> center-section spar sleeves -> saddle -> spar-aligned bulkheads -> fuselage sides`

The M3 bolts clamp and retain the assembly. They are not intended to carry the full wing bending and fore-aft shear loads by themselves.

## Fusion 360 master dimensions

Coordinates use X aft, Y right, and Z up. The wing root leading edge is X = 230 mm and its chord reference plane is Z = 55 mm.

| Feature | Dimension or position |
|---|---:|
| Wing center-section half-width | 45 mm |
| Minimum saddle contact half-width | 34 mm |
| Forward carbon-tube centerline | Constant X = 292.5 mm; 25% root chord |
| Rear carbon-tube centerline | Constant X = 380.0 mm; 60% root chord |
| Forward carbon-tube Z centerline | Constant Z = 55 mm through y = +/-500 mm |
| Rear carbon-tube Z centerline | Constant Z = 57 mm through y = +/-500 mm |
| Outer-tip dihedral break | y = +/-500 mm; carbon tubes end at the break |
| Front M3 bolt pair | X = 260 mm, Y = +/-28 mm |
| Rear M3 bolt pair | X = 445 mm, Y = +/-28 mm |
| Bolt clearance hole | 3.3 mm |
| Compression sleeve | 8 mm OD, 3.3 mm ID |
| Reinforced hardpoint boss | minimum 10 mm OD |
| Washer | minimum 9 mm OD |
| Central equipment corridor | 46 mm available; preserve at least 44 mm |

Create the saddle surface from the actual optimized root-airfoil underside rather than a rectangular notch. Keep the Phase 3 root incidence at 0 degrees unless the aerodynamic optimization changes it. Model 0.25 mm clearance on each side of fitted printed parts, then correct that value using a small printer-specific fit coupon.

## Construction requirements

- Print the center-section spar sleeves, four compression sleeves, saddle shoulders, bolt bosses, and two fuselage bulkheads in PETG or standard PLA. Do not use LW-PLA for these hardpoints.
- Tie each spar sleeve into both wing skins with local ribs or diamond-lattice webs. Do not end the sleeve in a single wall.
- Tie both spar-aligned bulkheads into the left and right fuselage skins with fillets and at least two adjacent longitudinal rails.
- Use M3 brass heat-set inserts or captive nuts in the fuselage hardpoints. Size insert pockets from the actual hardware and validate them with a coupon.
- Use nylon M3 bolts for initial flights so an abnormal impact can release the wing before the center fuselage is destroyed. Metal M3 bolts remain an option only after the mount passes load testing.
- Add shallow saddle shoulders or fitted pockets around the center section. Leave enough clearance for removal without letting the wing slide fore-aft or sideways when clamped.
- Keep the battery and wiring out of the compression-sleeve columns. The 46 mm corridor between bosses preserves the 44 mm equipment envelope with 1 mm clearance on each side.

## Assembly sequence

1. Bond each carbon tube continuously into its printed wing sleeve through the center section.
2. Bond the hardpoint and compression-sleeve inserts into the wing center section.
3. Install both spar-aligned bulkheads and four insert bosses in the fuselage.
4. Fit the wing to the saddle and verify full contact without bolt force.
5. Match-drill the four 3.3 mm holes using the assembled parts so accumulated print tolerance does not preload the wing.
6. Install washers and tighten the M3 bolts only enough to eliminate movement.

## Required tests before flight

1. Install the complete wing and ballast the fuselage to the 800 g design mass.
2. Support the aircraft beneath the fuselage and apply distributed upward load to both wings, not point loads at the tips.
3. Hold 2.4 kg total distributed load for 30 seconds for an initial 3 g proof test. Inspect the saddle, bolt bosses, spar sleeves, and fuselage bulkheads.
4. Repeat at 3.2 kg total for the 4 g design proof test only after the 3 g test shows no cracking, permanent movement, or insert pullout.
5. Do not fly if either wing shifts, the bolts loosen, the saddle gaps, or any printed layer turns white or cracks.

The static proof test validates the built print and adhesive joints; the dimensional checks in the aerodynamic script only verify that the attachment components fit the current baseline geometry.
