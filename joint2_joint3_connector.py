"""Joint 2 to Joint 3 Connector Script for Autodesk Fusion 360."""

import importlib
import math
import os
import sys
import traceback
import adsk.core
import adsk.fusion

# Ensure local modules can be imported
script_dir = os.path.dirname(__file__)
if script_dir in sys.path:
    sys.path.remove(script_dir)
sys.path.insert(0, script_dir)

import util
importlib.reload(util)

# Initialize the global variables for the Application and UserInterface objects.
app = adsk.core.Application.get()
ui = app.userInterface

# Disk & Geometry Constants
outer_circle_pcd = 63.0  # mm (outer circle PCD diameter)
OUTER_CIRCLE_PCD = outer_circle_pcd
EXTRUDE_DISTANCE = 5.0  # mm
MOTOR_SIDE_Z_OFFSET = 0.0  # mm (sketched on 0,0,0 plane)
MOTOR_SQUARE_HOLE_SIZE = 43.0  # mm (43 mm x 43 mm square cutout)
MOTOR_CORNER_CHAMFER_LINE_LENGTH = 5.0  # mm (each 90-degree corner replaced by a 5 mm flat line)

# M3 Screw Hole Constants for Disks (from joint3: diameter 3.1 mm)
M3_SCREW_THREAD_DIAMETER = 3.1  # mm
m3_radius_cm = (M3_SCREW_THREAD_DIAMETER / 2.0) / 10.0
MOTOR_M3_HOLES_COUNT = 4
MOTOR_M3_HOLES_PCD = 52.0 + 3.0  # mm (55.0 mm, increased by 3 mm)
MOTOR_M3_HOLES_ANGLE_DEG = -30.0  # degrees (shifted by -30 degrees)

# Distance between the 2 disks (1 mm clearance + 1 mm clearance/support + 34 mm joint thickness = 36 mm)
DISTANCE_BETWEEN_DISKS = 1.0 + 1.0 + 34.0  # mm
NON_MOTOR_SIDE_Z_OFFSET = EXTRUDE_DISTANCE + DISTANCE_BETWEEN_DISKS  # 41.0 mm

# Rod Holder Constants (oriented in -X direction, matching joint3 geometry)
ROD_HOLDER_DIST = 95.0  # mm (distance from joint center to rod holder face along -X)
ROD_HOLDER_WIDTH = 28.0  # mm (face width in Y, tapered down for a compact rod collar)
ROD_HOLDER_START_DIST = 18.0  # mm (offset from center where flange meets circle)
ROD_BORE_RADIUS = 7.25  # mm (14.5 mm diameter rod bore, matching joint3)
ROD_BORE_DEPTH = 43.0  # mm (depth of bore along +X)

# Attachment screws to fasten non-motor top cap to motor-side rod holder block
ATTACHMENT_SCREW_X = -38.0  # mm (X position along -X arm)
ATTACHMENT_SCREW_Y = 8.0  # mm (+/- Y offset)
ATTACHMENT_SCREW_RADIUS = 3.4 / 2.0  # mm (clearance hole for M3 screw)
ATTACHMENT_NUT_SIZE = 5.7  # mm (M3 nut side-to-side size)
ATTACHMENT_NUT_DEPTH = 3.0  # mm

# Taper & Slanting Constants
TAPER_START_X = -45.0  # mm (where the slanting begins along -X, past the attachment screws)
TAPER_END_HEIGHT = 26.0  # mm (vertical height at rod holder face x = -95 mm)
TOP_CAP_REACH = 45.0  # mm (extent of non-motor side top cap along -X)


def draw_motor_side_connection(
    comp: adsk.fusion.Component,
    total_height_mm: float,
) -> None:
    """
    Constructs the 'motor side connection' component:
    - Base disk at z = 0 (5 mm thick) with 43 mm chamfered cutout and 4 M3 holes.
    - Extruded rod holder structure extending in -X direction.
    - Symmetrical vertical slanting taper (top and bottom slopes) from x = -45 mm to x = -95 mm.
    - Horizontal rod bore (14.5 mm dia) + clamping nut/screw on -X face.
    - 2 M3 attachment holes and nut pockets on the flat mounting bridge (z = total_height_mm).
    """
    extrudes = comp.features.extrudeFeatures

    # -------------------------------------------------------------
    # 1. Base Disk at z = 0
    # -------------------------------------------------------------
    disk_sketch = util.create_sketch_on_offset_plane(comp, 0.0)

    # 1a. Base circle
    center_pt = adsk.core.Point3D.create(0, 0, 0)
    radius_cm = (OUTER_CIRCLE_PCD / 2.0) / 10.0
    circle = disk_sketch.sketchCurves.sketchCircles.addByCenterRadius(center_pt, radius_cm)
    circle.isFixed = True

    # 1b. Centered 43x43 mm motor cutout with 5 mm corner chamfers
    half_size_cm = (MOTOR_SQUARE_HOLE_SIZE / 2.0) / 10.0
    d_cm = ((MOTOR_CORNER_CHAMFER_LINE_LENGTH / math.sqrt(2.0)) / 10.0)
    pts = [
        (half_size_cm, -half_size_cm + d_cm),
        (half_size_cm, half_size_cm - d_cm),
        (half_size_cm - d_cm, half_size_cm),
        (-half_size_cm + d_cm, half_size_cm),
        (-half_size_cm, half_size_cm - d_cm),
        (-half_size_cm, -half_size_cm + d_cm),
        (-half_size_cm + d_cm, -half_size_cm),
        (half_size_cm - d_cm, -half_size_cm),
    ]
    lines = disk_sketch.sketchCurves.sketchLines
    for i in range(8):
        p_start = adsk.core.Point3D.create(pts[i][0], pts[i][1], 0)
        p_end = adsk.core.Point3D.create(pts[(i + 1) % 8][0], pts[(i + 1) % 8][1], 0)
        l = lines.addByTwoPoints(p_start, p_end)
        l.isFixed = True

    # 1c. 4 M3 screw holes (PCD 55 mm, -30 deg)
    util.draw_circular_hole_pattern(
        sketch=disk_sketch,
        center_pt=center_pt,
        hole_radius_cm=m3_radius_cm,
        count=MOTOR_M3_HOLES_COUNT,
        pcd_mm=MOTOR_M3_HOLES_PCD,
        angle_offset_rad=math.radians(MOTOR_M3_HOLES_ANGLE_DEG),
    )

    # 1d. Extrude base disk 5 mm (New Body)
    disk_prof = util.find_profile_by_outer_curve(disk_sketch, circle)
    if not disk_prof:
        disk_prof = util.find_largest_profile(disk_sketch)

    if disk_prof:
        extInput1 = extrudes.createInput(disk_prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extInput1.setDistanceExtent(False, adsk.core.ValueInput.createByReal(EXTRUDE_DISTANCE / 10.0))
        extrudes.add(extInput1)

    # -------------------------------------------------------------
    # 2. Rod Holder Block at z = 0 extending up to total_height_mm (41 mm)
    # -------------------------------------------------------------
    rod_sketch = util.create_sketch_on_offset_plane(comp, 0.0)

    rod_holder_dist_cm = ROD_HOLDER_DIST / 10.0
    half_width_cm = (ROD_HOLDER_WIDTH / 2.0) / 10.0
    start_dist_cm = ROD_HOLDER_START_DIST / 10.0
    intersect_y_cm = math.sqrt(radius_cm**2 - start_dist_cm**2)

    p_intersect_upper = adsk.core.Point3D.create(-start_dist_cm, intersect_y_cm, 0)
    p_intersect_lower = adsk.core.Point3D.create(-start_dist_cm, -intersect_y_cm, 0)
    p_end_upper = adsk.core.Point3D.create(-rod_holder_dist_cm, half_width_cm, 0)
    p_end_lower = adsk.core.Point3D.create(-rod_holder_dist_cm, -half_width_cm, 0)
    p_mid_inner = adsk.core.Point3D.create(-radius_cm, 0, 0)

    rod_lines = rod_sketch.sketchCurves.sketchLines
    l_upper = rod_lines.addByTwoPoints(p_intersect_upper, p_end_upper)
    l_upper.isFixed = True
    l_end = rod_lines.addByTwoPoints(p_end_upper, p_end_lower)
    l_end.isFixed = True
    l_lower = rod_lines.addByTwoPoints(p_end_lower, p_intersect_lower)
    l_lower.isFixed = True
    arc_inner = rod_sketch.sketchCurves.sketchArcs.addByThreePoints(p_intersect_lower, p_mid_inner, p_intersect_upper)
    arc_inner.isFixed = True

    rod_prof = rod_sketch.profiles.item(0) if rod_sketch.profiles.count > 0 else None

    if rod_prof:
        extInput2 = extrudes.createInput(rod_prof, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        extInput2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(total_height_mm / 10.0))
        extrudes.add(extInput2)

    # -------------------------------------------------------------
    # 3. Apply Symmetrical Vertical Taper Cut (slants height from 41 mm down to 26 mm)
    # -------------------------------------------------------------
    util.apply_vertical_taper_cut(
        comp=comp,
        taper_start_x_mm=TAPER_START_X,
        taper_end_x_mm=-ROD_HOLDER_DIST,
        start_height_mm=total_height_mm,
        end_height_mm=TAPER_END_HEIGHT,
    )

    # -------------------------------------------------------------
    # 4. Cut rod bore and clamping features on the -X face
    # -------------------------------------------------------------
    util.cut_rod_holder_bore_and_clamp(
        comp=comp,
        rod_holder_dist_mm=ROD_HOLDER_DIST,
        total_height_mm=total_height_mm,
        bore_radius_mm=ROD_BORE_RADIUS,
        bore_depth_mm=ROD_BORE_DEPTH,
        clamp_offset_mm=10.0,
    )

    # -------------------------------------------------------------
    # 5. Create M3 attachment holes and nut pockets on flat top bridge (z = total_height_mm)
    # -------------------------------------------------------------
    nut_pts = [
        adsk.core.Point3D.create(ATTACHMENT_SCREW_X / 10.0, -ATTACHMENT_SCREW_Y / 10.0, 0),
        adsk.core.Point3D.create(ATTACHMENT_SCREW_X / 10.0, ATTACHMENT_SCREW_Y / 10.0, 0),
    ]

    # Draw M3 screw holes (diameter 3.1 mm) going down 15 mm from top face
    top_screw_sketch = util.create_sketch_on_offset_plane(comp, total_height_mm)
    for pt in nut_pts:
        c = top_screw_sketch.sketchCurves.sketchCircles.addByCenterRadius(pt, m3_radius_cm)
        c.isFixed = True

    for p_idx in range(top_screw_sketch.profiles.count):
        p_c = top_screw_sketch.profiles.item(p_idx)
        try:
            cutIn = extrudes.createInput(p_c, adsk.fusion.FeatureOperations.CutFeatureOperation)
            cutIn.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-1.5))  # -15 mm downwards
            extrudes.add(cutIn)
        except:
            pass

    # Draw hex nut pockets (5.7 mm size, 3 mm deep) to seat the M3 nuts
    hex_sketch = util.create_sketch_on_offset_plane(comp, total_height_mm)
    for pt in nut_pts:
        util.draw_hexagon(hex_sketch, pt, ATTACHMENT_NUT_SIZE)

    for h_idx in range(hex_sketch.profiles.count):
        p_hex = hex_sketch.profiles.item(h_idx)
        try:
            cutIn = extrudes.createInput(p_hex, adsk.fusion.FeatureOperations.CutFeatureOperation)
            cutIn.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-ATTACHMENT_NUT_DEPTH / 10.0))  # -3 mm
            extrudes.add(cutIn)
        except:
            pass


def draw_non_motor_side_connection(
    comp: adsk.fusion.Component,
    offset_mm: float,
) -> None:
    """
    Constructs the 'non motor side connection' component:
    - Sits at offset_mm (41 mm) and extrudes 5 mm to 46 mm.
    - Forms a solid seamless plate combining the top disk and mounting top cap extending to x = -45 mm.
    - 4 M3 screw holes on the disk (PCD 55 mm, -30 deg).
    - 2 M3 clearance through-holes on the mounting cap at x = -38 mm.
    """
    sketch = util.create_sketch_on_offset_plane(comp, offset_mm)

    radius_cm = (OUTER_CIRCLE_PCD / 2.0) / 10.0
    start_dist_cm = ROD_HOLDER_START_DIST / 10.0
    cap_reach_cm = TOP_CAP_REACH / 10.0
    intersect_y_cm = math.sqrt(radius_cm**2 - start_dist_cm**2)

    # Compute cap width at x = -TOP_CAP_REACH matching the tapered motor side block
    rod_holder_dist_cm = ROD_HOLDER_DIST / 10.0
    half_end_w_cm = (ROD_HOLDER_WIDTH / 2.0) / 10.0
    slope = (half_end_w_cm - intersect_y_cm) / (-rod_holder_dist_cm - -start_dist_cm)
    half_cap_w_cm = intersect_y_cm + slope * (-cap_reach_cm - -start_dist_cm)

    # 1. Outer boundary: Major circular arc (+X) + 3 mounting cap boundary lines (-X)
    p_intersect_upper = adsk.core.Point3D.create(-start_dist_cm, intersect_y_cm, 0)
    p_intersect_lower = adsk.core.Point3D.create(-start_dist_cm, -intersect_y_cm, 0)
    p_cap_upper = adsk.core.Point3D.create(-cap_reach_cm, half_cap_w_cm, 0)
    p_cap_lower = adsk.core.Point3D.create(-cap_reach_cm, -half_cap_w_cm, 0)
    p_mid_outer = adsk.core.Point3D.create(radius_cm, 0, 0)

    # Major arc through (+radius_cm, 0)
    arc_major = sketch.sketchCurves.sketchArcs.addByThreePoints(p_intersect_upper, p_mid_outer, p_intersect_lower)
    arc_major.isFixed = True

    # 3 boundary lines of the mounting cap extending to x = -45 mm
    lines = sketch.sketchCurves.sketchLines
    l_lower = lines.addByTwoPoints(p_intersect_lower, p_cap_lower)
    l_lower.isFixed = True
    l_end = lines.addByTwoPoints(p_cap_lower, p_cap_upper)
    l_end.isFixed = True
    l_upper = lines.addByTwoPoints(p_cap_upper, p_intersect_upper)
    l_upper.isFixed = True

    # 2. 4 M3 screw holes on the disk (PCD 55 mm, -30 deg)
    center_pt = adsk.core.Point3D.create(0, 0, 0)
    util.draw_circular_hole_pattern(
        sketch=sketch,
        center_pt=center_pt,
        hole_radius_cm=m3_radius_cm,
        count=MOTOR_M3_HOLES_COUNT,
        pcd_mm=MOTOR_M3_HOLES_PCD,
        angle_offset_rad=math.radians(MOTOR_M3_HOLES_ANGLE_DEG),
    )

    # 3. 2 M3 clearance through-holes (dia 3.4 mm) on the mounting cap
    attach_pts = [
        adsk.core.Point3D.create(ATTACHMENT_SCREW_X / 10.0, -ATTACHMENT_SCREW_Y / 10.0, 0),
        adsk.core.Point3D.create(ATTACHMENT_SCREW_X / 10.0, ATTACHMENT_SCREW_Y / 10.0, 0),
    ]
    for pt in attach_pts:
        c = sketch.sketchCurves.sketchCircles.addByCenterRadius(pt, ATTACHMENT_SCREW_RADIUS / 10.0)
        c.isFixed = True

    # 4. Extrude the single unified solid plate by 5 mm
    plate_prof = util.find_largest_profile(sketch)
    if not plate_prof:
        plate_prof = util.find_profile_by_outer_curve(sketch, l_end)

    extrudes = comp.features.extrudeFeatures
    distance = adsk.core.ValueInput.createByReal(EXTRUDE_DISTANCE / 10.0)

    if plate_prof:
        extInput = extrudes.createInput(plate_prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extInput.setDistanceExtent(False, distance)
        extrudes.add(extInput)


def run(_context: str):
    """This function is called by Fusion when the script is run."""
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design found.", "No Design")
            return

        # Clean up any existing components from a previous run (enables refreshing in the same document)
        util.delete_component_by_name(design, "motor side connection")
        util.delete_component_by_name(design, "non motor side connection")

        # Calculate interface offset for the non-motor side disk:
        # Base disk (5 mm) + gap between disks (1 + 1 + 34 = 36 mm) = 41 mm
        non_motor_offset = EXTRUDE_DISTANCE + DISTANCE_BETWEEN_DISKS

        # 1. Create 'motor side connection' component
        # Extends disk at z = 0 and rod holder block up to z = non_motor_offset (41 mm)
        motor_comp = util.create_component(design, "motor side connection")
        draw_motor_side_connection(
            comp=motor_comp,
            total_height_mm=non_motor_offset,
        )

        # 2. Create 'non motor side connection' component
        # Sits at z = non_motor_offset (41 mm) and extends across the rod holder as a 5 mm top cap
        non_motor_comp = util.create_component(design, "non motor side connection")
        draw_non_motor_side_connection(
            comp=non_motor_comp,
            offset_mm=non_motor_offset,
        )

        # Log success silently without showing a blocking popup dialog
        app.log("Joint 2 - Joint 3 Connector updated successfully.")

    except:  # pylint:disable=bare-except
        if ui:
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")
        app.log(f"Failed:\n{traceback.format_exc()}")
