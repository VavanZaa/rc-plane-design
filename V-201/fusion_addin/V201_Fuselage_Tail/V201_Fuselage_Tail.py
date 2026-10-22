import adsk.core, adsk.fusion, traceback
import math

def generate_naca0010(chord_cm, num_pts=50):
    # Generates upper surface points for NACA 0010.
    # Returns a list of (x, z) relative to LE at (0,0)
    pts = []
    t = 0.10
    for i in range(num_pts + 1):
        # Cosine spacing for better resolution at LE
        beta = math.pi * i / num_pts
        x_norm = 0.5 * (1.0 - math.cos(beta))
        
        yt_norm = 5 * t * (0.2969 * math.sqrt(x_norm) - 
                           0.1260 * x_norm - 
                           0.3516 * (x_norm**2) + 
                           0.2843 * (x_norm**3) - 
                           0.1015 * (x_norm**4))
        pts.append((x_norm * chord_cm, yt_norm * chord_cm))
    return pts

def create_tail_sketch(sketches, plane, chord_cm, offset_x_cm, offset_y_cm, offset_z_cm, is_vertical=False):
    sketch = sketches.add(plane)
    
    # Generate upper and lower points
    upper_raw = generate_naca0010(chord_cm)
    
    up_pts = adsk.core.ObjectCollection.create()
    lo_pts = adsk.core.ObjectCollection.create()
    
    # In a sketch on a standard plane, we need to map to world properly, or just use world coords with 3D sketch.
    # To keep it simple, we use a 3D sketch.
    sketch.is3D = True
    
    for px, pz in upper_raw:
        if is_vertical:
            # Vertical tail: thickness is in Y direction, chord is in X direction
            up_pts.add(adsk.core.Point3D.create(offset_x_cm + px, offset_y_cm + pz, offset_z_cm))
            lo_pts.add(adsk.core.Point3D.create(offset_x_cm + px, offset_y_cm - pz, offset_z_cm))
        else:
            # Horizontal tail: thickness is in Z direction, chord is in X direction
            up_pts.add(adsk.core.Point3D.create(offset_x_cm + px, offset_y_cm, offset_z_cm + pz))
            lo_pts.add(adsk.core.Point3D.create(offset_x_cm + px, offset_y_cm, offset_z_cm - pz))
            
    # Draw splines
    splines = sketch.sketchCurves.sketchFittedSplines
    up_spline = splines.add(up_pts)
    lo_spline = splines.add(lo_pts)
    
    # Close trailing edge
    lines = sketch.sketchCurves.sketchLines
    lines.addByTwoPoints(up_spline.endSketchPoint, lo_spline.endSketchPoint)
    
    return sketch

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

        # Create new component
        occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = "V201_Fuselage_Tail"

        sketches = comp.sketches
        planes = comp.constructionPlanes
        loftFeats = comp.features.loftFeatures
        mirrorFeats = comp.features.mirrorFeatures
        
        # -----------------------------------------------------
        # 1. HORIZONTAL STABILIZER
        # -----------------------------------------------------
        htail_root_chord = 12.0 # cm
        htail_tip_chord = 8.5   # cm
        htail_span = 32.5       # cm (half span)
        htail_le_x = 81.0       # cm
        htail_z = 4.5           # cm
        
        h_root_sketch = create_tail_sketch(sketches, comp.xYConstructionPlane, htail_root_chord, htail_le_x, 0.0, htail_z)
        h_root_sketch.name = "HTail_Root"
        
        h_tip_sketch = create_tail_sketch(sketches, comp.xYConstructionPlane, htail_tip_chord, htail_le_x + (htail_root_chord - htail_tip_chord)/2, htail_span, htail_z)
        h_tip_sketch.name = "HTail_Tip"
        
        # Loft HTail
        loftInput_h = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput_h.loftSections.add(h_root_sketch.profiles.item(0))
        loftInput_h.loftSections.add(h_tip_sketch.profiles.item(0))
        loftInput_h.isSolid = True
        htail_body = loftFeats.add(loftInput_h)
        
        # Mirror HTail
        mirrorInput_h = mirrorFeats.createInput(
            adsk.core.ObjectCollection.createWithArray([htail_body.bodies.item(0)]),
            comp.xZConstructionPlane
        )
        mirrorFeats.add(mirrorInput_h)

        # -----------------------------------------------------
        # 2. VERTICAL FIN
        # -----------------------------------------------------
        vtail_root_chord = 12.5 # cm
        vtail_tip_chord = 7.5   # cm
        vtail_height = 27.5     # cm
        vtail_le_x = 81.0       # cm
        
        v_root_sketch = create_tail_sketch(sketches, comp.xZConstructionPlane, vtail_root_chord, vtail_le_x, 0.0, htail_z, is_vertical=True)
        v_root_sketch.name = "VTail_Root"
        
        v_tip_sketch = create_tail_sketch(sketches, comp.xZConstructionPlane, vtail_tip_chord, vtail_le_x + 2.5, 0.0, htail_z + vtail_height, is_vertical=True)
        v_tip_sketch.name = "VTail_Tip"
        
        # Loft VTail
        loftInput_v = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftInput_v.loftSections.add(v_root_sketch.profiles.item(0))
        loftInput_v.loftSections.add(v_tip_sketch.profiles.item(0))
        loftInput_v.isSolid = True
        loftFeats.add(loftInput_v)
        
        # -----------------------------------------------------
        # 3. FUSELAGE
        # -----------------------------------------------------
        # Stations: X, Z_center, Radius/Dimensions
        fuse_stations = [
            {"x": 5.0, "z": 4.5, "r": 2.0},              # Firewall
            {"x": 15.0, "z": 4.5, "r": 4.0},             # Battery Bay
            {"x": 23.0, "z": 4.5, "r": 4.5},             # Wing Saddle front
            {"x": 48.0, "z": 4.5, "r": 4.0},             # Wing Saddle rear
            {"x": 90.0, "z": 4.5, "r": 1.5}              # Tail boom end
        ]
        
        fuse_profiles = []
        for i, fs in enumerate(fuse_stations):
            # Create offset plane along X (offset from YZ plane)
            planeInput = planes.createInput()
            offsetValue = adsk.core.ValueInput.createByReal(fs["x"])
            planeInput.setByOffset(comp.yZConstructionPlane, offsetValue)
            stationPlane = planes.add(planeInput)
            stationPlane.name = f"Fuse_Station_{i}"
            
            sketch = sketches.add(stationPlane)
            sketch.name = f"Fuse_Profile_{i}"
            
            # Draw circle (in YZ sketch space, origin maps to X=offset)
            # Center of the circle should be at world Y=0, Z=fs["z"].
            # To be foolproof against sketch-coordinate mapping, use 3D sketch points
            sketch.is3D = True
            center_pt = sketch.modelToSketchSpace(adsk.core.Point3D.create(fs["x"], 0.0, fs["z"]))
            
            sketch.sketchCurves.sketchCircles.addByCenterRadius(center_pt, fs["r"])
            fuse_profiles.append(sketch.profiles.item(0))
            
        # Loft Fuselage
        loftInput_f = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for prof in fuse_profiles:
            loftInput_f.loftSections.add(prof)
        loftInput_f.isSolid = True
        loftFeats.add(loftInput_f)
        
        ui.messageBox('Fuselage and Tail generated successfully!')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
