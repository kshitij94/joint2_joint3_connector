import math
import adsk.core
import adsk.fusion
import util

from constants import (
    ARC_THICKNESS_MM,
    CONE_WIDTH_MM,
    BASE_ARC_RADIUS_MM,
    ROD_SIDE_WIDTH_MM,
    M3_SCREW_THREAD_DIAMETER_MM,
    ROD_SCREW_HOLE_INSET_MM,
)

SMALLER_ARC_ANGLE_DEG = 35.0
LARGER_ARC_ANGLE_DEG = 55.0
BASE_EXTRUDE_DISTANCE_MM = 32.0  # mm

# Rod holder hole (ported from kshitij94/joint3)
ROD_HOLE_DIAMETER_MM = 14.5  # mm
ROD_HOLE_DEPTH_MM = 43.0  # mm
ROD_NUT_SLOT_WIDTH_MM = 6.5  # mm (along Y)
ROD_NUT_SLOT_HEIGHT_MM = 2.7  # mm (along Z)
ROD_NUT_SLOT_DEPTH_MM = 15.0  # mm
ROD_NUT_SLOT_OFFSET_MM = 10.0  # mm below rod hole center
ROD_SCREW_HOLE_DIAMETER_MM = 3.0  # mm

def construct_base(
    comp: adsk.fusion.Component,
    sketch_offset: float,
    baseArcRadiusMM = BASE_ARC_RADIUS_MM ,
    rodSideWidthMM = ROD_SIDE_WIDTH_MM,
    extrudeDistanceMM: float = BASE_EXTRUDE_DISTANCE_MM
) -> None:
    """Constructs the base for the joint2 to joint3 connector."""
    sketch = util.create_sketch_on_offset_plane(comp, sketch_offset)
    lines = sketch.sketchCurves.sketchLines

    circleTowardCenter = util.create_circle(sketch, adsk.core.Point3D.create(0, 0, 0), baseArcRadiusMM)
    circleTowardRodHolder = util.create_circle(sketch, adsk.core.Point3D.create(0, 0, 0), baseArcRadiusMM + ARC_THICKNESS_MM)

    rodHoleSideXCM = (baseArcRadiusMM + ARC_THICKNESS_MM + CONE_WIDTH_MM) / 10.0
    rodHoleSideLow = adsk.core.Point3D.create(rodHoleSideXCM, -rodSideWidthMM / 10.0, 0)
    rodHoleSideHigh = adsk.core.Point3D.create(rodHoleSideXCM, rodSideWidthMM / 10.0, 0)

    smallerArcAngleRad = math.radians(SMALLER_ARC_ANGLE_DEG)
    rodHolderFacingArcRadiusCM = (baseArcRadiusMM + ARC_THICKNESS_MM) / 10.0
    p1 = adsk.core.Point3D.create(
        rodHolderFacingArcRadiusCM * math.cos(smallerArcAngleRad),
        rodHolderFacingArcRadiusCM * math.sin(smallerArcAngleRad),
        0
    )
    p2 = adsk.core.Point3D.create(
        rodHolderFacingArcRadiusCM * math.cos(smallerArcAngleRad),
        -rodHolderFacingArcRadiusCM * math.sin(smallerArcAngleRad),
        0
    )
    largerArcAngleRad = math.radians(LARGER_ARC_ANGLE_DEG)
    p3 = adsk.core.Point3D.create(
        rodHolderFacingArcRadiusCM * math.cos(largerArcAngleRad),
        rodHolderFacingArcRadiusCM * math.sin(largerArcAngleRad),
        0
    )
    p4 = adsk.core.Point3D.create(
        rodHolderFacingArcRadiusCM * math.cos(largerArcAngleRad),
        -rodHolderFacingArcRadiusCM * math.sin(largerArcAngleRad),
        0
    )

    arcRadiusCM = (baseArcRadiusMM) / 10.0
    p5 = adsk.core.Point3D.create(
        arcRadiusCM * math.cos(largerArcAngleRad),
        arcRadiusCM * math.sin(largerArcAngleRad),
        0
    )
    p6 = adsk.core.Point3D.create(
        arcRadiusCM * math.cos(largerArcAngleRad),
        -arcRadiusCM * math.sin(largerArcAngleRad),
        0
    )

    lines.addByTwoPoints(p1, rodHoleSideHigh)
    lines.addByTwoPoints(p2, rodHoleSideLow)
    lines.addByTwoPoints(p6, p4)
    lines.addByTwoPoints(p5, p3)
    lines.addByTwoPoints(rodHoleSideLow, rodHoleSideHigh)

    circleTowardCenter.trim(adsk.core.Point3D.create(-baseArcRadiusMM / 10.0, 0, 0))
    remaining_outer = circleTowardRodHolder.trim(adsk.core.Point3D.create(-(baseArcRadiusMM + ARC_THICKNESS_MM) / 10.0, 0, 0))
    if remaining_outer and remaining_outer.count > 0:
        remaining_outer.item(0).trim(adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM) / 10.0, 0, 0))

    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 7)/10,0,0), M3_SCREW_THREAD_DIAMETER_MM/2)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 25)/10,15/10,0), M3_SCREW_THREAD_DIAMETER_MM/2)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 25)/10,-15/10,0), M3_SCREW_THREAD_DIAMETER_MM/2)

    base_extrude = util.extrudeMaxAreaProfile(sketch, extrudeDistanceMM, comp)

    if base_extrude:
        create_rod_holder_hole(comp, base_extrude, rodHoleSideXCM, sketch_offset, extrudeDistanceMM)


def create_rod_holder_hole(
    comp: adsk.fusion.Component,
    extrude_feature: adsk.fusion.ExtrudeFeature,
    rodSideXCM: float,
    sketch_offset: float,
    extrudeDistanceMM: float,
) -> None:
    """Cuts the rod holder hole, the clamping nut slot and the vertical clamping screw hole
    into the rod-side end face of the base (logic ported from kshitij94/joint3)."""
    extrudes = comp.features.extrudeFeatures

    # Find the planar end face on the rod side (normal parallel to X, furthest along +X)
    target_face = None
    target_x = None
    for f in range(extrude_feature.faces.count):
        face = extrude_feature.faces.item(f)
        if face.geometry.objectType == adsk.core.Plane.classType():
            plane = adsk.core.Plane.cast(face.geometry)
            normal = plane.normal
            if abs(normal.y) < 1e-5 and abs(normal.z) < 1e-5:
                if target_x is None or plane.origin.x > target_x:
                    target_face = face
                    target_x = plane.origin.x

    if not target_face:
        return

    face_sketch = comp.sketches.add(target_face)
    center_z_cm = (sketch_offset + extrudeDistanceMM / 2.0) / 10.0

    # Rod holder hole, centered on the end face
    hole_center = face_sketch.modelToSketchSpace(adsk.core.Point3D.create(rodSideXCM, 0.0, center_z_cm))
    hole_center.z = 0
    rod_hole_circle = face_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        hole_center, (ROD_HOLE_DIAMETER_MM / 2.0) / 10.0
    )
    rod_hole_circle.isFixed = True

    rod_hole_profile = util.find_profile_by_outer_curve(face_sketch, rod_hole_circle)
    if rod_hole_profile:
        cut_input = extrudes.createInput(rod_hole_profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        # Negative value cuts inwards, opposite the face's normal direction
        cut_input.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-ROD_HOLE_DEPTH_MM / 10.0))
        extrudes.add(cut_input)

    # Rectangular slot for the nut used to tighten the rod holder
    slot_z_cm = center_z_cm - ROD_NUT_SLOT_OFFSET_MM / 10.0
    half_w_cm = (ROD_NUT_SLOT_WIDTH_MM / 2.0) / 10.0
    half_h_cm = (ROD_NUT_SLOT_HEIGHT_MM / 2.0) / 10.0
    c1 = face_sketch.modelToSketchSpace(adsk.core.Point3D.create(rodSideXCM, -half_w_cm, slot_z_cm - half_h_cm))
    c2 = face_sketch.modelToSketchSpace(adsk.core.Point3D.create(rodSideXCM, half_w_cm, slot_z_cm + half_h_cm))
    c1.z = 0
    c2.z = 0
    rect_lines = face_sketch.sketchCurves.sketchLines.addTwoPointRectangle(c1, c2)
    for l in range(rect_lines.count):
        rect_lines.item(l).isFixed = True

    rect_profile = util.find_profile_by_outer_curve(face_sketch, rect_lines.item(0))
    if rect_profile:
        rect_cut_input = extrudes.createInput(rect_profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        rect_cut_input.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-ROD_NUT_SLOT_DEPTH_MM / 10.0))
        extrudes.add(rect_cut_input)

    # Vertical screw hole through the base, passing through the nut slot and rod hole
    screw_sketch = comp.sketches.add(comp.xYConstructionPlane)
    screw_center = adsk.core.Point3D.create(rodSideXCM - ROD_SCREW_HOLE_INSET_MM / 10.0, 0.0, 0.0)
    screw_circle = util.create_circle(screw_sketch, screw_center, ROD_SCREW_HOLE_DIAMETER_MM / 2.0)

    screw_profile = util.find_profile_by_outer_curve(screw_sketch, screw_circle)
    if screw_profile:
        screw_cut_input = extrudes.createInput(screw_profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
        # Start slightly below the base bottom and cut fully through to the top
        start_offset = adsk.core.ValueInput.createByReal((sketch_offset - 5.0) / 10.0)
        screw_cut_input.startExtent = adsk.fusion.OffsetStartDefinition.create(start_offset)
        screw_cut_input.setDistanceExtent(False, adsk.core.ValueInput.createByReal((extrudeDistanceMM + 10.0) / 10.0))
        extrudes.add(screw_cut_input)