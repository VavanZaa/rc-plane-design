import adsk.core, adsk.fusion, traceback
import os
import csv
import math

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        if not design:
            ui.messageBox('No active Fusion design.')
            return

        rootComp = design.rootComponent

        # Find data directory (script is in fusion_addin/V201_Wing_Importer)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        fusion_addin_dir = os.path.dirname(script_dir)
        project_dir = os.path.dirname(fusion_addin_dir)
        csv_path = os.path.join(project_dir, 'optimization_results', 'v201_phase4_wing_stations.csv')
        airfoils_dir = os.path.join(project_dir, 'optimization_results', 'airfoils')

        if not os.path.exists(csv_path):
            ui.messageBox(f'CSV not found: {csv_path}')
            return

        # Read stations
        stations = []
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                stations.append({
                    'y_mm': float(row['y_mm']),
                    'chord_mm': float(row['chord_mm']),
                    'twist_deg': float(row['twist_deg']),
                    'le_x_mm': float(row['leading_edge_x_mm']),
                    'le_z_mm': float(row['leading_edge_z_mm'])
                })

        # Create new component
        occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        wingComp = occ.component
        wingComp.name = "Optimized_Wing_Master"

        sketches = wingComp.sketches
        planes = wingComp.constructionPlanes

        le_points = adsk.core.ObjectCollection.create()
        te_points = adsk.core.ObjectCollection.create()

        profiles_for_loft = []

        for i, st in enumerate(stations):
            # Parse the .dat file
            y_str = f"{int(st['y_mm']):04d}"
            dat_filename = f"v201_phase4_station_{i:02d}_y{y_str}.dat"
            dat_path = os.path.join(airfoils_dir, dat_filename)
            
            pts = []
            with open(dat_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:] # skip header
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        pts.append((float(parts[0]), float(parts[1])))
            
            # Find LE index (min x)
            le_idx = 0
            min_x = 999.0
            for idx, p in enumerate(pts):
                if p[0] < min_x:
                    min_x = p[0]
                    le_idx = idx

            # Split into upper (TE to LE) and lower (LE to TE)
            upper_raw = pts[:le_idx+1]
            lower_raw = pts[le_idx:]
            
            # Reverse upper so it goes LE to TE
            upper_raw.reverse()

            chord_cm = st['chord_mm'] / 10.0
            theta = math.radians(st['twist_deg'])
            le_x_cm = st['le_x_mm'] / 10.0
            le_z_cm = st['le_z_mm'] / 10.0
            y_cm = st['y_mm'] / 10.0

            def transform(p):
                x = p[0] * chord_cm
                z = p[1] * chord_cm
                # Twist pitch rotation around Y axis
                x_rot = x * math.cos(theta) + z * math.sin(theta)
                z_rot = -x * math.sin(theta) + z * math.cos(theta)
                # Translate
                return (x_rot + le_x_cm, y_cm, z_rot + le_z_cm)

            upper_transformed = [transform(p) for p in upper_raw]
            lower_transformed = [transform(p) for p in lower_raw]

            # Collect guide rail points
            le_pt = upper_transformed[0]
            te_upper = upper_transformed[-1]
            te_lower = lower_transformed[-1]
            te_mid = (
                (te_upper[0] + te_lower[0]) / 2.0,
                (te_upper[1] + te_lower[1]) / 2.0,
                (te_upper[2] + te_lower[2]) / 2.0
            )

            le_points.add(adsk.core.Point3D.create(*le_pt))
            te_points.add(adsk.core.Point3D.create(*te_mid))

            # Create construction plane at Y offset
            planeInput = planes.createInput()
            offsetValue = adsk.core.ValueInput.createByReal(y_cm)
            planeInput.setByOffset(wingComp.xZConstructionPlane, offsetValue)
            stationPlane = planes.add(planeInput)
            stationPlane.name = f"Station_{i}_y{st['y_mm']}"

            # Create sketch
            sketch = sketches.add(stationPlane)
            sketch.name = f"Profile_{i}"

            # Draw splines
            up_coll = adsk.core.ObjectCollection.create()
            for p in upper_transformed:
                up_coll.add(adsk.core.Point3D.create(p[0], 0, p[2])) # Y in sketch is mapped from Z in world, X from X. The offset plane handles world Y.
                # Actually, fusion maps X to sketch X and Z to sketch Y when creating a sketch on XZ plane offset.
                # Let's use sketch.modelToSketchSpace() if needed, or directly draw in world coords.
                # In a sketch, adding by Point3D uses sketch space coordinates by default unless we use modelToSketchSpace.
                pass
            
            # Better to use 3D sketches to avoid coordinate mapping confusion, or map to sketch space.
            # If sketch is on a plane parallel to XZ, Sketch X is World X, Sketch Y is World Z (usually).
            # To be absolutely safe, let's use modelToSketchSpace for all points.
            up_sk_pts = adsk.core.ObjectCollection.create()
            for p in upper_transformed:
                sk_pt = sketch.modelToSketchSpace(adsk.core.Point3D.create(*p))
                up_sk_pts.add(sk_pt)
                
            lo_sk_pts = adsk.core.ObjectCollection.create()
            for p in lower_transformed:
                sk_pt = sketch.modelToSketchSpace(adsk.core.Point3D.create(*p))
                lo_sk_pts.add(sk_pt)

            splines = sketch.sketchCurves.sketchFittedSplines
            up_spline = splines.add(up_sk_pts)
            lo_spline = splines.add(lo_sk_pts)

            # Close trailing edge
            lines = sketch.sketchCurves.sketchLines
            lines.addByTwoPoints(up_spline.endSketchPoint, lo_spline.endSketchPoint)
            
            profiles_for_loft.append(sketch.profiles.item(0))

        # Create guide rails in a 3D sketch
        rail_sketch = sketches.add(wingComp.xYConstructionPlane)
        rail_sketch.name = "Guide_Rails"
        rail_sketch.is3D = True
        
        le_rail_spline = rail_sketch.sketchCurves.sketchFittedSplines.add(le_points)
        te_rail_spline = rail_sketch.sketchCurves.sketchFittedSplines.add(te_points)

        # Create Loft
        loftFeats = wingComp.features.loftFeatures
        loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        
        for prof in profiles_for_loft:
            loftInput.loftSections.add(prof)
            
        loftInput.isSolid = True
        loftInput.centerLineOrRails.addRail(le_rail_spline)
        loftInput.centerLineOrRails.addRail(te_rail_spline)

        loftFeat = loftFeats.add(loftInput)

        # Mirror the body to create the left half-wing
        mirrorFeats = wingComp.features.mirrorFeatures
        mirrorInput = mirrorFeats.createInput(
            adsk.core.ObjectCollection.createWithArray([loftFeat.bodies.item(0)]),
            wingComp.xZConstructionPlane
        )
        mirrorFeats.add(mirrorInput)

        ui.messageBox('Wing imported, lofted, and mirrored successfully!')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
