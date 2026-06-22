import adsk.core, adsk.fusion, traceback
import math
import os
from typing import Tuple, Optional

HEIGHT = 0.6  # Height of the bearing in cm (6mm)
dimention_text_height = HEIGHT - 0.2
ball_diameter = 0.45  # Ball diameter in cm (4.5mm)
offset_towards_z = 0.1
outer_housing_outer_dia = 3.0 # default value
outer_housing_thickness = 0.3
outer_housing_inner_dia = 0.0
offset_for_revolte_cut = 0.1
inner_housing_thickness = 0.2
inner_housing_outer_dia = 0.0
inner_housing_inner_dia = 0.0
min_gap_between_separater_holes = 0.5
separater_hole_diameter = ball_diameter - 0.21
cork_hole_diameter = ball_diameter+0.07
cork_cap_diameter = cork_hole_diameter-0.01

def extrude_ring(comp: adsk.fusion.Component, sketch: adsk.fusion.Sketch, height: float) -> Tuple[adsk.fusion.ExtrudeFeature, adsk.fusion.BRepFace]:
    # Find the ring profile (which has 2 loops)
    prof = None
    for p in sketch.profiles:
        if p.profileLoops.count == 2:
            prof = p
            break
    if not prof:
        prof = sketch.profiles.item(0)
        
    # Extrude the ring profile
    ext_in = comp.features.extrudeFeatures.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext_in.setDistanceExtent(False, adsk.core.ValueInput.createByReal(height))
    ext_feat = comp.features.extrudeFeatures.add(ext_in)
    
    # Find outer cylindrical face
    outer_face = None
    max_radius = 0.0
    for face in ext_feat.sideFaces:
        geom = face.geometry
        if geom.classType() == adsk.core.Cylinder.classType():
            cylinder = adsk.core.Cylinder.cast(geom)
            if cylinder and cylinder.radius > max_radius:
                max_radius = cylinder.radius
                outer_face = face
    if not outer_face:
        raise Exception("No outer cylindrical face found")      
    return ext_feat, outer_face

def create_common_outer_housing(design: adsk.fusion.Design, name: str, outer_dia: Optional[float] = None) -> Tuple[adsk.fusion.Component, adsk.fusion.BRepFace]:
    if outer_dia is None:
        outer_dia = outer_housing_outer_dia
    rootComp = design.rootComponent
    
    # Create new component
    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    
    # Add sketch on XY plane
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name + " Sketch"
    
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0,0), outer_dia / 2.0)
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0,0), outer_housing_inner_dia / 2.0)
    
    # Extrude the ring to HEIGHT
    _, outer_face = extrude_ring(comp, sk, HEIGHT)
    
    # Sketch on XZ plane for the revolute cut
    sk_cut = comp.sketches.add(comp.xZConstructionPlane)
    model_pt = adsk.core.Point3D.create(outer_housing_inner_dia / 2.0 - offset_towards_z, 0, HEIGHT / 2.0)
    sketch_pt = sk_cut.modelToSketchSpace(model_pt)
    sk_cut.sketchCurves.sketchCircles.addByCenterRadius(sketch_pt, ball_diameter / 2.0)
    
    # Revolve Cut around Z axis (restricted to local housing body)
    prof_cut = sk_cut.profiles.item(0)
    rev_in = comp.features.revolveFeatures.createInput(prof_cut, comp.zConstructionAxis, adsk.fusion.FeatureOperations.CutFeatureOperation)
    rev_in.setAngleExtent(False, adsk.core.ValueInput.createByReal(math.pi * 2))
    
    # Restrict cut to only this component's body
    body = comp.bRepBodies.item(0)
    rev_in.participantBodies = [body]
    
    comp.features.revolveFeatures.add(rev_in)
    
    return comp, outer_face

def create_outer_housing(design: adsk.fusion.Design) -> adsk.fusion.Component:
    comp, outer_housing_outer_face = create_common_outer_housing(design, "Housing")
    body = comp.bRepBodies.item(0)

    # Create a tangent plane to cut the cork hole.
    planes = comp.constructionPlanes
    planeInput = planes.createInput()
    planeInput.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(outer_housing_outer_dia / 2.0))
    tangent_plane = planes.add(planeInput)
    
    # On this tanget plane create a sketch which is a circle of the size cork_hole_diameter
    sk_tangent = comp.sketches.add(tangent_plane)
    sk_tangent.name = "Tangent Sketch"
    
    
    model_pt_tangent = adsk.core.Point3D.create(outer_housing_outer_dia / 2.0, 0, HEIGHT / 2.0)
    sketch_pt_tangent = sk_tangent.modelToSketchSpace(model_pt_tangent)
    sk_tangent.sketchCurves.sketchCircles.addByCenterRadius(sketch_pt_tangent, cork_hole_diameter / 2.0)
    
    # Extrude cut the cork hole
    prof_tangent = sk_tangent.profiles.item(0)
    ext_cut_in = comp.features.extrudeFeatures.createInput(prof_tangent, adsk.fusion.FeatureOperations.CutFeatureOperation)
    distance_def = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(outer_housing_thickness * 1.5))
    ext_cut_in.setOneSideExtent(distance_def, adsk.fusion.ExtentDirections.NegativeExtentDirection)
    
    # Restrict cut to only outer housing body
    ext_cut_in.participantBodies = [body]
    
    comp.features.extrudeFeatures.add(ext_cut_in)
    
    engrave_dimentions(comp, outer_housing_outer_face)
        
    return comp

def engrave_dimentions(comp: adsk.fusion.Component, face: adsk.fusion.BRepFace) -> None:
    planes = comp.constructionPlanes
    
    # Cast face to Cylinder to get its radius
    cylinder = adsk.core.Cylinder.cast(face.geometry)
    cylinder_radius = cylinder.radius
    
    # 1. Outer diameter tangent plane (at Y = cylinder_radius, Z = HEIGHT/2)
    planeInput_outer = planes.createInput()
    planeInput_outer.setByOffset(comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(cylinder_radius))
    outer_diameter_tangent_plane = planes.add(planeInput_outer)
    
    # 2. Inner diameter tangent plane (at Y = -1 * cylinder_radius, Z = HEIGHT/2)
    planeInput_inner = planes.createInput()
    planeInput_inner.setByOffset(comp.xZConstructionPlane, adsk.core.ValueInput.createByReal(-cylinder_radius))
    inner_diameter_tangent_plane = planes.add(planeInput_inner)
    
    # Calculate text in mm and round to 2 decimals
    outer_val = round(outer_housing_outer_dia * 10, 2)
    inner_val = round(inner_housing_inner_dia * 10, 2)
    outer_str = f"{outer_val:g}"
    inner_str = f"{inner_val:g}"
    
    # Create sketch and text on outer tangent plane
    sk_outer = comp.sketches.add(outer_diameter_tangent_plane)
    sk_outer.name = "Outer Diameter Text Sketch"
    model_point_outer = adsk.core.Point3D.create(0.0, cylinder_radius, HEIGHT / 2.0)
    sketch_point_outer = sk_outer.modelToSketchSpace(model_point_outer)
    approx_width_outer = len(outer_str) * 0.6 * dimention_text_height
    pos_x_outer = sketch_point_outer.x - approx_width_outer / 2.0
    pos_y_outer = sketch_point_outer.y - dimention_text_height / 2.0
    point_outer = adsk.core.Point3D.create(pos_x_outer, pos_y_outer, 0)
    txt_input_outer = sk_outer.sketchTexts.createInput(outer_str, dimention_text_height, point_outer)
    sketch_text_outer = sk_outer.sketchTexts.add(txt_input_outer)
    
    # Create sketch and text on inner tangent plane
    sk_inner = comp.sketches.add(inner_diameter_tangent_plane)
    sk_inner.name = "Inner Diameter Text Sketch"
    model_point_inner = adsk.core.Point3D.create(0.0, -cylinder_radius, HEIGHT / 2.0)
    sketch_point_inner = sk_inner.modelToSketchSpace(model_point_inner)
    approx_width_inner = len(inner_str) * 0.6 * dimention_text_height
    pos_x_inner = sketch_point_inner.x - approx_width_inner / 2.0
    pos_y_inner = sketch_point_inner.y - dimention_text_height / 2.0
    point_inner = adsk.core.Point3D.create(pos_x_inner, pos_y_inner, 0)
    txt_input_inner = sk_inner.sketchTexts.createInput(inner_str, dimention_text_height, point_inner)
    sketch_text_inner = sk_inner.sketchTexts.add(txt_input_inner)
    
    # Extrude cut the text profiles starting from their sketch planes (which are tangent)
    extrudes = comp.features.extrudeFeatures
    body = comp.bRepBodies.item(0)
    
    # 1.5 mm depth (0.15 cm)
    distance_def = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(0.15))
    
    # Extrude outer text (Negative direction goes inwards)
    extInput_outer = extrudes.createInput(sketch_text_outer, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_outer.setOneSideExtent(distance_def, adsk.fusion.ExtentDirections.NegativeExtentDirection)
    extInput_outer.participantBodies = [body]
    extrudes.add(extInput_outer)
        
    # Extrude inner text (Positive direction goes inwards since plane is at Y = -R)
    extInput_inner = extrudes.createInput(sketch_text_inner, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput_inner.setOneSideExtent(distance_def, adsk.fusion.ExtentDirections.PositiveExtentDirection)
    extInput_inner.participantBodies = [body]
    extrudes.add(extInput_inner)

def create_inner_housing(design: adsk.fusion.Design) -> adsk.fusion.Component:
    rootComp = design.rootComponent
    
    # Create new component
    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Inner Housing"
    
    # Add sketch on XY plane
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Inner Housing Sketch"
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0,0), inner_housing_outer_dia / 2.0)
    
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0,0), inner_housing_inner_dia / 2.0)
    
    # Extrude the ring to HEIGHT
    extrude_ring(comp, sk, HEIGHT)
    
    # Sketch on XZ plane for the revolute cut
    sk_cut = comp.sketches.add(comp.xZConstructionPlane)

    model_pt = adsk.core.Point3D.create(inner_housing_outer_dia / 2.0 + offset_for_revolte_cut, 0, HEIGHT / 2.0)
    sketch_pt = sk_cut.modelToSketchSpace(model_pt)
    sk_cut.sketchCurves.sketchCircles.addByCenterRadius(sketch_pt, ball_diameter / 2.0) # radius is ball_diameter / 2.0
    
    # Revolve Cut around Z axis (restricted to local inner housing body)
    prof_cut = sk_cut.profiles.item(0)
    rev_in = comp.features.revolveFeatures.createInput(prof_cut, comp.zConstructionAxis, adsk.fusion.FeatureOperations.CutFeatureOperation)
    rev_in.setAngleExtent(False, adsk.core.ValueInput.createByReal(math.pi * 2))
    
    # Restrict cut to only this component's body
    body = comp.bRepBodies.item(0)
    rev_in.participantBodies = [body]
    
    comp.features.revolveFeatures.add(rev_in)
    
    return comp

def compute_number_of_holes_in_separater(mid_dia: float) -> int:
    # Constraint: Maximize the number of separator holes such that the centers
    # are spaced so that the gap between any two adjacent holes of size
    # 'ball_diameter' is at least 3mm (0.3cm).
    circumference = math.pi * mid_dia
    num_holes = int(circumference / (separater_hole_diameter + min_gap_between_separater_holes))
    return num_holes

def create_separator(design: adsk.fusion.Design) -> adsk.fusion.Component:
    rootComp = design.rootComponent
    
    # Create new component
    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Separator"
    
    # Add sketch on XY plane
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = "Separator Sketch"
    
    sep_thickness = 0.05  # 0.5mm thickness in cm
    mid_dia = (outer_housing_inner_dia + inner_housing_outer_dia) / 2.0
    
    sep_outer_dia = mid_dia + sep_thickness / 2.0
    sep_inner_dia = mid_dia - sep_thickness / 2.0
    
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0,0), sep_outer_dia / 2.0)
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0,0), sep_inner_dia / 2.0)
    
    # Extrude the ring to HEIGHT
    extrude_ring(comp, sk, HEIGHT)
    
    # Create tangent plane (parallel to Z axis, perpendicular to XY plane, tangent to outer separator face)
    planes = comp.constructionPlanes
    planeInput = planes.createInput()
    planeInput.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(sep_outer_dia / 2.0))
    tangent_plane = planes.add(planeInput)
    
    # Create sketch on tangent plane
    sk_tangent = comp.sketches.add(tangent_plane)
    sk_tangent.name = "Tangent Sketch"
    
    # Circle center at height/2 along Z axis, Y axis as 0
    model_pt = adsk.core.Point3D.create(sep_outer_dia / 2.0, 0, HEIGHT / 2.0)
    sketch_pt = sk_tangent.modelToSketchSpace(model_pt)
    sk_tangent.sketchCurves.sketchCircles.addByCenterRadius(sketch_pt, separater_hole_diameter)
    
    # Extrude cut the circle into the separator (towards the Z-axis / inwards)
    prof_tangent = sk_tangent.profiles.item(0)
    ext_cut_in = comp.features.extrudeFeatures.createInput(prof_tangent, adsk.fusion.FeatureOperations.CutFeatureOperation)
    distance_def = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(sep_thickness * 1.5))
    ext_cut_in.setOneSideExtent(distance_def, adsk.fusion.ExtentDirections.NegativeExtentDirection)
    
    # Restrict cut to only separator body
    body = comp.bRepBodies.item(0)
    ext_cut_in.participantBodies = [body]
    
    ext_cut = comp.features.extrudeFeatures.add(ext_cut_in)
    
    num_holes = compute_number_of_holes_in_separater(mid_dia)
    
    # Circular Pattern to replicate the hole
    objCollection = adsk.core.ObjectCollection.create()
    for face in ext_cut.faces:
        objCollection.add(face)
        
    pattern_in = comp.features.circularPatternFeatures.createInput(objCollection, comp.zConstructionAxis)
    pattern_in.quantity = adsk.core.ValueInput.createByReal(num_holes)
    pattern_in.computeOption = adsk.fusion.PatternComputeOptions.IdenticalPatternCompute
    comp.features.circularPatternFeatures.add(pattern_in)
    
    return comp

def create_cork(design: adsk.fusion.Design) -> adsk.fusion.Component:
    # Cork thickness is 0.5mm (0.05cm) less than the outer housing wall thickness.
    # Therefore, the temporary ring for the cork has outer_dia decreased by 2 * 0.05cm = 0.1cm.
    cork_outer_dia = outer_housing_outer_dia - 0.1
    comp, _ = create_common_outer_housing(design, "Cork", outer_dia=cork_outer_dia)
    body = comp.bRepBodies.item(0)
    
    # Create tangent plane (parallel to Z axis, perpendicular to XY plane, tangent to outer cork face)
    planes = comp.constructionPlanes
    planeInput = planes.createInput()
    planeInput.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(cork_outer_dia / 2.0))
    tangent_plane = planes.add(planeInput)
    
    # Create sketch on tangent plane
    sk_tangent = comp.sketches.add(tangent_plane)
    sk_tangent.name = "Tangent Sketch"
    
    # Circle center at height/2 along Z axis, Y axis as 0
    model_pt_tangent = adsk.core.Point3D.create(cork_outer_dia / 2.0, 0, HEIGHT / 2.0)
    sketch_pt_tangent = sk_tangent.modelToSketchSpace(model_pt_tangent)
    sk_tangent.sketchCurves.sketchCircles.addByCenterRadius(sketch_pt_tangent, cork_cap_diameter / 2.0)
    
    # Extrude intersect the circle into the housing (towards the Z-axis / inwards)
    prof_tangent = sk_tangent.profiles.item(0)
    ext_intersect_in = comp.features.extrudeFeatures.createInput(prof_tangent, adsk.fusion.FeatureOperations.IntersectFeatureOperation)
    distance_def = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(cork_outer_dia / 2.0))
    ext_intersect_in.setOneSideExtent(distance_def, adsk.fusion.ExtentDirections.NegativeExtentDirection)
    
    # Restrict intersect to only cork body
    ext_intersect_in.participantBodies = [body]
    
    comp.features.extrudeFeatures.add(ext_intersect_in)
    
    # Create a horizontal slot cut (screwdriver slot) on the outer face of the cork
    # We create a new sketch on the tangent plane
    sk_slot = comp.sketches.add(tangent_plane)
    sk_slot.name = "Slot Sketch"
    
    slot_width = 0.6  # 4mm horizontal length
    slot_thickness = 0.1  # 0.8mm vertical thickness
    
    model_center = adsk.core.Point3D.create(cork_outer_dia / 2.0, 0, HEIGHT / 2.0)
    sketch_center = sk_slot.modelToSketchSpace(model_center)
    
    x_c = sketch_center.x
    y_c = sketch_center.y
    
    # Define rectangle corners relative to local sketch space
    p1 = adsk.core.Point3D.create(x_c - slot_width/2.0, y_c - slot_thickness/2.0, 0)
    p2 = adsk.core.Point3D.create(x_c + slot_width/2.0, y_c - slot_thickness/2.0, 0)
    p3 = adsk.core.Point3D.create(x_c + slot_width/2.0, y_c + slot_thickness/2.0, 0)
    p4 = adsk.core.Point3D.create(x_c - slot_width/2.0, y_c + slot_thickness/2.0, 0)
    
    sk_slot.sketchCurves.sketchLines.addByTwoPoints(p1, p2)
    sk_slot.sketchCurves.sketchLines.addByTwoPoints(p2, p3)
    sk_slot.sketchCurves.sketchLines.addByTwoPoints(p3, p4)
    sk_slot.sketchCurves.sketchLines.addByTwoPoints(p4, p1)
    
    prof_slot = sk_slot.profiles.item(0)
    
    # Extrude cut the slot inwards by 0.5 mm (0.03 cm)
    ext_slot_in = comp.features.extrudeFeatures.createInput(prof_slot, adsk.fusion.FeatureOperations.CutFeatureOperation)
    distance_def_slot = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(0.05))
    ext_slot_in.setOneSideExtent(distance_def_slot, adsk.fusion.ExtentDirections.NegativeExtentDirection)
    
    # Restrict cut to only cork body (after the intersect, there is only one body representing the cork)
    cork_body = comp.bRepBodies.item(0)
    ext_slot_in.participantBodies = [cork_body]
    
    comp.features.extrudeFeatures.add(ext_slot_in)
    
    return comp
def get_outer_housing_outer_dia(ui: adsk.core.UserInterface) -> Optional[float]:
    # Prompt user for outer housing outer diameter
    val, cancelled = ui.inputBox('Enter outer housing outer diameter (cm):', 'Outer Housing Outer Diameter', str(outer_housing_outer_dia))
    if cancelled:
        return None

    try:
        val_float = float(val)
    except ValueError:
        ui.messageBox('Please enter a valid number.', 'Error')
        return None

    if val_float <= 0:
        ui.messageBox('Diameter must be a positive number.', 'Error')
        return None

    return val_float

def run(context: dict) -> None:
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion design', 'No Design')
            return

        global outer_housing_outer_dia
        global outer_housing_inner_dia
        global inner_housing_outer_dia
        global inner_housing_inner_dia

        outer_dia = get_outer_housing_outer_dia(ui)
        if outer_dia is None:
            return

        # Update globals with validated values
        outer_housing_outer_dia = outer_dia
        outer_housing_inner_dia = outer_housing_outer_dia - 2 * outer_housing_thickness
        inner_housing_outer_dia = outer_housing_inner_dia - 0.22 * 2
        inner_housing_inner_dia = round(inner_housing_outer_dia - 2 * inner_housing_thickness, 2)
        if inner_housing_inner_dia <= 0:
            ui.messageBox('The outer housing outer diameter is too small to fit the inner housing.', 'Error')
            return

        create_outer_housing(design)
        create_inner_housing(design)
        create_separator(design)
        create_cork(design)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
