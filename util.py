import adsk.core
import adsk.fusion
import math
from typing import Optional


def create_sketch_on_offset_plane(comp: adsk.fusion.Component, offset_mm: float) -> adsk.fusion.Sketch:
    """
    Creates a construction plane offset from the XY plane, and returns a new sketch on it.
    
    :param comp: The component to create the plane and sketch in.
    :param offset_mm: The offset distance in mm.
    :return: The created Sketch object.
    """
    sketches = comp.sketches
    constructionPlanes = comp.constructionPlanes
    planeInput = constructionPlanes.createInput()
    # Fusion API uses cm internally, so convert mm to cm
    offsetValue = adsk.core.ValueInput.createByReal(offset_mm / 10.0)
    planeInput.setByOffset(comp.xYConstructionPlane, offsetValue)
    offsetPlane = constructionPlanes.add(planeInput)
    return sketches.add(offsetPlane)


def find_profile_by_outer_curve(sketch: adsk.fusion.Sketch, outer_curve: adsk.fusion.SketchEntity) -> Optional[adsk.fusion.Profile]:
    """
    Finds the profile in a sketch that has the specified outer_curve as its outer loop.
    
    :param sketch: The sketch to search profiles in.
    :param outer_curve: The outer boundary curve of the desired profile.
    :return: The matching Profile object, or None if not found.
    """
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        for j in range(prof.profileLoops.count):
            loop = prof.profileLoops.item(j)
            if loop.isOuter:
                for k in range(loop.profileCurves.count):
                    pc = loop.profileCurves.item(k)
                    if pc.sketchEntity == outer_curve:
                        return prof
    return None


def draw_circular_hole_pattern(
    sketch: adsk.fusion.Sketch,
    center_pt: adsk.core.Point3D,
    hole_radius_cm: float,
    count: int,
    pcd_mm: float,
    angle_offset_rad: float = 0.0,
) -> list:
    """
    Draws a circular pattern of holes on the given sketch.
    
    :param sketch: The sketch to draw circles in.
    :param center_pt: The center point of the pattern.
    :param hole_radius_cm: Radius of each hole in cm.
    :param count: Number of holes.
    :param pcd_mm: Pitch circle diameter in mm.
    :param angle_offset_rad: Initial angular offset in radians.
    :return: List of created SketchCircle objects.
    """
    circles = sketch.sketchCurves.sketchCircles
    drawn_circles = []
    for i in range(count):
        theta = (2.0 * math.pi / count) * i + angle_offset_rad
        cx = center_pt.x + (pcd_mm * math.cos(theta)) / 20.0
        cy = center_pt.y + (pcd_mm * math.sin(theta)) / 20.0
        hole_center = adsk.core.Point3D.create(cx, cy, 0)
        
        circle = circles.addByCenterRadius(hole_center, hole_radius_cm)
        circle.isFixed = True
        drawn_circles.append(circle)
    return drawn_circles


def draw_rod_holder_profile(
    sketch: adsk.fusion.Sketch,
    circle_radius_cm: float,
    rod_holder_dist_mm: float,
    rod_holder_width_mm: float,
    rod_holder_start_dist_mm: float,
) -> tuple:
    """
    Draws the rod holder extension in the -X direction connecting the disk to the rod face.
    Returns the sketch lines created: (connect_upper, connect_lower, end_line).
    Note: mid_line is deliberately omitted so the circle and rod holder share a natural boundary without gaps.
    """
    lines = sketch.sketchCurves.sketchLines
    rod_holder_dist_cm = rod_holder_dist_mm / 10.0
    half_width_cm = (rod_holder_width_mm / 2.0) / 10.0
    start_dist_cm = rod_holder_start_dist_mm / 10.0

    # Ensure start_dist_cm is within circle radius
    if start_dist_cm >= circle_radius_cm:
        start_dist_cm = circle_radius_cm - 0.01

    intersect_y_cm = math.sqrt(circle_radius_cm**2 - start_dist_cm**2)

    # In -X direction:
    pt_end_lower = adsk.core.Point3D.create(-rod_holder_dist_cm, -half_width_cm, 0)
    pt_end_upper = adsk.core.Point3D.create(-rod_holder_dist_cm, half_width_cm, 0)
    end_line = lines.addByTwoPoints(pt_end_lower, pt_end_upper)
    end_line.isFixed = True

    pt_intersect_upper = adsk.core.Point3D.create(-start_dist_cm, intersect_y_cm, 0)
    pt_intersect_lower = adsk.core.Point3D.create(-start_dist_cm, -intersect_y_cm, 0)

    connect_upper = lines.addByTwoPoints(pt_intersect_upper, pt_end_upper)
    connect_upper.isFixed = True

    connect_lower = lines.addByTwoPoints(pt_intersect_lower, pt_end_lower)
    connect_lower.isFixed = True

    return (connect_upper, connect_lower, end_line)


def find_largest_profile(sketch: adsk.fusion.Sketch) -> adsk.fusion.Profile:
    """
    Finds the profile with the largest area in the sketch.
    Useful for selecting the main outer plate/disk body when internal holes are present.
    """
    best_prof = None
    max_area = -1.0
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        try:
            area = prof.areaProperties().area
            if area > max_area:
                max_area = area
                best_prof = prof
        except:
            pass
    if not best_prof and sketch.profiles.count > 0:
        best_prof = sketch.profiles.item(0)
    return best_prof


def apply_vertical_taper_cut(
    comp: adsk.fusion.Component,
    taper_start_x_mm: float,
    taper_end_x_mm: float,
    start_height_mm: float,
    end_height_mm: float,
) -> None:
    """
    Cuts symmetrical slanting tapers (top and bottom) on the rod holder along the -X direction.
    - At taper_start_x_mm (e.g. -45 mm): height is start_height_mm (e.g. 41 mm).
    - At taper_end_x_mm (e.g. -95 mm): height is end_height_mm (e.g. 26 mm).
    Operates on the XZ construction plane with a symmetric cut feature across the Y axis.
    """
    extrudes = comp.features.extrudeFeatures
    sketch = comp.sketches.add(comp.xZConstructionPlane)
    lines = sketch.sketchCurves.sketchLines

    start_x = taper_start_x_mm / 10.0
    end_x = taper_end_x_mm / 10.0
    overshoot_x = (taper_end_x_mm - 15.0) / 10.0  # extend cut past end face

    center_z = (start_height_mm / 2.0) / 10.0
    start_top_z = start_height_mm / 10.0
    start_bottom_z = 0.0

    half_end_h = (end_height_mm / 2.0) / 10.0
    end_top_z = center_z + half_end_h
    end_bottom_z = center_z - half_end_h

    # 1. Top Wedge Cut (removes material above the slanting top line)
    top_3d = [
        adsk.core.Point3D.create(start_x, 0, start_top_z),
        adsk.core.Point3D.create(end_x, 0, end_top_z),
        adsk.core.Point3D.create(overshoot_x, 0, end_top_z),
        adsk.core.Point3D.create(overshoot_x, 0, start_top_z + 1.0),
        adsk.core.Point3D.create(start_x, 0, start_top_z + 1.0),
    ]
    top_2d = [sketch.modelToSketchSpace(p) for p in top_3d]
    for i in range(len(top_2d)):
        l = lines.addByTwoPoints(top_2d[i], top_2d[(i + 1) % len(top_2d)])
        l.isFixed = True

    # 2. Bottom Wedge Cut (removes material below the slanting bottom line)
    bottom_3d = [
        adsk.core.Point3D.create(start_x, 0, start_bottom_z),
        adsk.core.Point3D.create(end_x, 0, end_bottom_z),
        adsk.core.Point3D.create(overshoot_x, 0, end_bottom_z),
        adsk.core.Point3D.create(overshoot_x, 0, start_bottom_z - 1.0),
        adsk.core.Point3D.create(start_x, 0, start_bottom_z - 1.0),
    ]
    bottom_2d = [sketch.modelToSketchSpace(p) for p in bottom_3d]
    for i in range(len(bottom_2d)):
        l = lines.addByTwoPoints(bottom_2d[i], bottom_2d[(i + 1) % len(bottom_2d)])
        l.isFixed = True

    # Cut both wedges symmetrically across Y (50 mm each side covers full width)
    cut_half_width = adsk.core.ValueInput.createByReal(5.0)
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        try:
            cutInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
            cutInput.setSymmetricExtent(cut_half_width, True)
            extrudes.add(cutInput)
        except:
            pass



def draw_hexagon(sketch: adsk.fusion.Sketch, center_pt: adsk.core.Point3D, side_to_side_size_mm: float) -> list:
    """
    Draws a regular hexagon centered at center_pt with the specified side-to-side distance in mm.
    """
    r_cm = (side_to_side_size_mm / 10.0) / math.sqrt(3.0)
    hex_points = []
    for k in range(6):
        angle = k * math.pi / 3.0
        px = center_pt.x + r_cm * math.cos(angle)
        py = center_pt.y + r_cm * math.sin(angle)
        hex_points.append(adsk.core.Point3D.create(px, py, 0))

    lines = sketch.sketchCurves.sketchLines
    hex_lines = []
    for k in range(6):
        p_start = hex_points[k]
        p_end = hex_points[(k + 1) % 6]
        line = lines.addByTwoPoints(p_start, p_end)
        line.isFixed = True
        hex_lines.append(line)
    return hex_lines


def cut_rod_holder_bore_and_clamp(
    comp: adsk.fusion.Component,
    rod_holder_dist_mm: float,
    total_height_mm: float,
    bore_radius_mm: float = 7.25,
    bore_depth_mm: float = 43.0,
    clamp_offset_mm: float = 10.0,
) -> None:
    """
    Cuts the horizontal rod bore into the -X face of the rod holder,
    along with the vertical clamping screw hole and nut pocket matching joint3.
    """
    extrudes = comp.features.extrudeFeatures
    constructionPlanes = comp.constructionPlanes

    # 1. Find the planar face on the body at the -X end (matching joint3.py)
    target_face = None
    for b_idx in range(comp.bRepBodies.count):
        b = comp.bRepBodies.item(b_idx)
        for f_idx in range(b.faces.count):
            f = b.faces.item(f_idx)
            if f.geometry.objectType == adsk.core.Plane.classType():
                p = adsk.core.Plane.cast(f.geometry)
                if abs(p.normal.y) < 1e-4 and abs(p.normal.z) < 1e-4:
                    if target_face is None or p.origin.x < target_face.geometry.origin.x:
                        target_face = f

    if target_face:
        face_sketch = comp.sketches.add(target_face)
    else:
        planeInput = constructionPlanes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(-rod_holder_dist_mm / 10.0)
        planeInput.setByOffset(comp.yZConstructionPlane, offsetValue)
        endPlane = constructionPlanes.add(planeInput)
        face_sketch = comp.sketches.add(endPlane)

    center_z_mm = total_height_mm / 2.0

    # Map 3D point (x = -rod_holder_dist, y = 0, z = center_z) to 2D sketch space
    model_center = adsk.core.Point3D.create(-rod_holder_dist_mm / 10.0, 0.0, center_z_mm / 10.0)
    sketch_center = face_sketch.modelToSketchSpace(model_center)

    bore_circle = face_sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center, bore_radius_mm / 10.0)
    bore_circle.isFixed = True

    bore_prof = find_profile_by_outer_curve(face_sketch, bore_circle)
    if not bore_prof and face_sketch.profiles.count > 0:
        bore_prof = face_sketch.profiles.item(0)

    if bore_prof:
        cut_dist_val = -abs(bore_depth_mm) / 10.0
        cutInput = extrudes.createInput(bore_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        cutInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cut_dist_val))
        try:
            extrudes.add(cutInput)
        except:
            try:
                cutInput2 = extrudes.createInput(bore_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
                cutInput2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-cut_dist_val))
                extrudes.add(cutInput2)
            except:
                pass

    # 2. Clamping Nut Pocket on the -X face (matching joint3 dimensions: 6.5 mm x 2.7 mm, depth 15 mm)
    rect_w_cm = 6.5 / 10.0
    rect_h_cm = 2.7 / 10.0
    model_rect_center = adsk.core.Point3D.create(
        -rod_holder_dist_mm / 10.0,
        0.0,
        (center_z_mm - 10.0) / 10.0
    )
    sketch_rect_center = face_sketch.modelToSketchSpace(model_rect_center)

    p1 = adsk.core.Point3D.create(sketch_rect_center.x - rect_w_cm / 2.0, sketch_rect_center.y - rect_h_cm / 2.0, 0)
    p2 = adsk.core.Point3D.create(sketch_rect_center.x + rect_w_cm / 2.0, sketch_rect_center.y + rect_h_cm / 2.0, 0)
    rect_lines = face_sketch.sketchCurves.sketchLines.addTwoPointRectangle(p1, p2)
    for l in range(rect_lines.count):
        rect_lines.item(l).isFixed = True

    rect_prof = find_profile_by_outer_curve(face_sketch, rect_lines.item(0))
    if rect_prof:
        cut_dist_val = -1.5
        cutInput = extrudes.createInput(rect_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        cutInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cut_dist_val))
        try:
            extrudes.add(cutInput)
        except:
            try:
                cutInput2 = extrudes.createInput(rect_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
                cutInput2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-cut_dist_val))
                extrudes.add(cutInput2)
            except:
                pass

    # 3. Vertical clamping screw hole through the block (matching joint3: 3 mm dia at 10 mm from end face)
    try:
        clamp_x_mm = -rod_holder_dist_mm + clamp_offset_mm
        clamp_sketch = comp.sketches.add(comp.xYConstructionPlane)
        clamp_center = adsk.core.Point3D.create(clamp_x_mm / 10.0, 0.0, 0.0)
        clamp_circle = clamp_sketch.sketchCurves.sketchCircles.addByCenterRadius(clamp_center, 0.15)  # 3 mm dia
        clamp_circle.isFixed = True

        clamp_prof = find_profile_by_outer_curve(clamp_sketch, clamp_circle)
        if clamp_prof:
            vertCutInput = extrudes.createInput(clamp_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
            start_offset = adsk.core.ValueInput.createByReal(-0.2)
            vertCutInput.startExtent = adsk.fusion.OffsetStartDefinition.create(start_offset)
            vertCutDistance = adsk.core.ValueInput.createByReal((total_height_mm / 10.0) + 0.4)
            vertCutInput.setDistanceExtent(False, vertCutDistance)
            extrudes.add(vertCutInput)
    except:
        pass





def create_component(design: adsk.fusion.Design, name: str) -> adsk.fusion.Component:
    """
    Creates a new component under the root component with the given name.
    
    :param design: The active Fusion design.
    :param name: The name for the new component.
    :return: The created Component object.
    """
    rootComp = design.rootComponent
    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp


def get_body_height(comp: adsk.fusion.Component) -> float:
    """
    Returns the maximum height (Z-axis extent) of the solid bodies in the component in mm.
    
    :param comp: The component to inspect.
    :return: The height of the solid bodies in mm, or 0.0 if no solid bodies exist.
    """
    if comp.bRepBodies.count == 0:
        return 0.0
    
    min_z = float('inf')
    max_z = float('-inf')
    
    for i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(i)
        bbox = body.boundingBox
        if bbox.minPoint.z < min_z:
            min_z = bbox.minPoint.z
        if bbox.maxPoint.z > max_z:
            max_z = bbox.maxPoint.z
            
    if min_z == float('inf') or max_z == float('-inf'):
        return 0.0
        
    return (max_z - min_z) * 10.0


def delete_component_by_name(design: adsk.fusion.Design, name: str) -> None:
    """
    Deletes any existing occurrence whose component name or occurrence name contains the given name.
    Useful for re-running scripts in the same design without creating duplicate components.
    """
    root_comp = design.rootComponent
    occurrences_to_delete = []
    target_name = name.strip().lower()
    for i in range(root_comp.occurrences.count):
        occ = root_comp.occurrences.item(i)
        occ_name = occ.name.lower()
        comp_name = occ.component.name.lower() if occ.component else ""
        if target_name in occ_name or target_name in comp_name:
            occurrences_to_delete.append(occ)
            
    for occ in occurrences_to_delete:
        try:
            occ.deleteMe()
        except:
            try:
                # In direct modeling or parametric mode, if deleteMe fails, try removing bodies
                if occ.component:
                    for b in list(occ.component.bRepBodies):
                        b.deleteMe()
            except:
                pass


