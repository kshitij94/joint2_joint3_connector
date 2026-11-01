import adsk.core
import adsk.fusion
import util
import math

from constants import (
    ARC_THICKNESS_MM,
    CONE_WIDTH_MM,
    BASE_ARC_RADIUS_MM,
    ROD_SIDE_WIDTH_MM,
    M3_SCREW_THREAD_DIAMETER_MM,
    ROD_SCREW_HOLE_INSET_MM,
)

EXTENDER_OUTER_DIAMETER_MM = 65  # mm
EXTENDER_INNER_DIAMETER_MM = 5.0  # mm
SMALLER_ARC_ANGLE_DEG = 60.0
EXTENDER_EXTRUDE_DISTANCE_MM = 5.0  # mm

MOTOR_BODY_WIDTH_MM = 42.5  # mm
MOTOR_SCREW_PCD_MM = 21.98 * 2.0  # 43.96 mm (matching joint3 output_disk_2)
MAGNETIC_ENCODER_PCD_MM = 22.5  # mm
JOINT_CONNECTION_SCREWS_PCD_MM = 52.0  # mm
MOTOR_SIDE_JOINT_CONNECTION_SCREWS_PCD_MM = 55.0
MOTOR_NOTCH_WIDTH_MM = 17.0  # mm
MOTOR_NOTCH_DEPTH_MM = 6.0  # mm
def construct_motor_side_extender(
    comp: adsk.fusion.Component,
    sketch_offset: float,
    baseArcRadiusMM: float = BASE_ARC_RADIUS_MM,
    rodSideWidthMM: float = ROD_SIDE_WIDTH_MM,
    extrudeDistanceMM: float = EXTENDER_EXTRUDE_DISTANCE_MM,
):
    """Constructs the motor-side extender onto joint 3."""
    sketch = util.create_sketch_on_offset_plane(comp, sketch_offset)
    origin = adsk.core.Point3D.create(0, 0, 0)

    outer_circle = util.create_circle(sketch, origin, EXTENDER_OUTER_DIAMETER_MM / 2.0)

    # Square cutout for the motor centered at origin, rotated by 30 degrees, with a 17mm x 6mm notch
    half_side_cm = (MOTOR_BODY_WIDTH_MM / 2.0) / 10.0
    rot_rad = math.radians(30.0)
    cos_rot = math.cos(rot_rad)
    sin_rot = math.sin(rot_rad)

    def rot_pt(x, y):
        rx = x * cos_rot - y * sin_rot
        ry = x * sin_rot + y * cos_rot
        return adsk.core.Point3D.create(rx, ry, 0.0)

    # 4 corners of motor square
    p_sq0 = rot_pt(-half_side_cm, -half_side_cm)  # negative x, negative y
    p_sq1 = rot_pt(half_side_cm, -half_side_cm)   # positive x, negative y
    p_sq2 = rot_pt(half_side_cm, half_side_cm)    # positive x, positive y
    p_sq3 = rot_pt(-half_side_cm, half_side_cm)   # negative x, positive y

    lines = sketch.sketchCurves.sketchLines
    line01 = lines.addByTwoPoints(p_sq0, p_sq1)
    line01.isFixed = True
    line12 = lines.addByTwoPoints(p_sq1, p_sq2)
    line12.isFixed = True
    line23 = lines.addByTwoPoints(p_sq2, p_sq3)
    line23.isFixed = True

    # 17mm x 6mm notch cut centered on the edge between p_sq0 and p_sq3
    notch_half_w_cm = (MOTOR_NOTCH_WIDTH_MM / 2.0) / 10.0
    notch_d_cm = MOTOR_NOTCH_DEPTH_MM / 10.0

    p_notch_edge_low = rot_pt(-half_side_cm, -notch_half_w_cm)
    p_notch_outer_low = rot_pt(-half_side_cm - notch_d_cm, -notch_half_w_cm)
    p_notch_outer_high = rot_pt(-half_side_cm - notch_d_cm, notch_half_w_cm)
    p_notch_edge_high = rot_pt(-half_side_cm, notch_half_w_cm)

    # Edge segments and notch outline
    l_edge_high = lines.addByTwoPoints(p_sq3, p_notch_edge_high)
    l_edge_high.isFixed = True
    l_notch_out_high = lines.addByTwoPoints(p_notch_edge_high, p_notch_outer_high)
    l_notch_out_high.isFixed = True
    l_notch_back = lines.addByTwoPoints(p_notch_outer_high, p_notch_outer_low)
    l_notch_back.isFixed = True
    l_notch_out_low = lines.addByTwoPoints(p_notch_outer_low, p_notch_edge_low)
    l_notch_out_low.isFixed = True
    l_edge_low = lines.addByTwoPoints(p_notch_edge_low, p_sq0)
    l_edge_low.isFixed = True
    l_notch_edge = lines.addByTwoPoints(p_notch_edge_high, p_notch_edge_low)
    l_notch_edge.isFixed = True

    rodHoleSideXCM = (baseArcRadiusMM + ARC_THICKNESS_MM + CONE_WIDTH_MM) / 10.0
    rodHoleSideLow = adsk.core.Point3D.create(rodHoleSideXCM, -rodSideWidthMM / 10.0, 0)
    rodHoleSideHigh = adsk.core.Point3D.create(rodHoleSideXCM, rodSideWidthMM / 10.0, 0)
    rad = math.radians(SMALLER_ARC_ANGLE_DEG)
    circleRadiusCM = (EXTENDER_OUTER_DIAMETER_MM) / 20.0
    p1 = adsk.core.Point3D.create(
        circleRadiusCM * math.cos(rad),
        circleRadiusCM * math.sin(rad),
        0
    )
    p2 = adsk.core.Point3D.create(
        circleRadiusCM * math.cos(rad),
        -circleRadiusCM * math.sin(rad),
        0
    )

    upper_line = lines.addByTwoPoints(p1, rodHoleSideHigh)
    lower_line = lines.addByTwoPoints(p2, rodHoleSideLow)
    end_line = lines.addByTwoPoints(rodHoleSideLow, rodHoleSideHigh)

    p3 = adsk.core.Point3D.create(circleRadiusCM, 0, 0)
    outer_circle.trim(p3)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 7) / 10.0, 0, 0), M3_SCREW_THREAD_DIAMETER_MM / 2.0)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 25) / 10.0, 15 / 10.0, 0), M3_SCREW_THREAD_DIAMETER_MM / 2.0)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 25) / 10.0, -15 / 10.0, 0), M3_SCREW_THREAD_DIAMETER_MM / 2.0)
    screw_center = adsk.core.Point3D.create(rodHoleSideXCM - ROD_SCREW_HOLE_INSET_MM / 10.0, 0.0, 0.0)
    util.create_circle(sketch, screw_center, 3.1 / 2.0)

    # 4 holes for joint connection screws (PCD 55 mm, rotated by 60 degrees)
    joint_connection_screws_pcd_cm = MOTOR_SIDE_JOINT_CONNECTION_SCREWS_PCD_MM / 20.0
    for i in range(4):
        angle_rad = math.radians(60.0 + i * 90.0)
        hole_center = adsk.core.Point3D.create(
            joint_connection_screws_pcd_cm * math.cos(angle_rad),
            joint_connection_screws_pcd_cm * math.sin(angle_rad),
            0.0
        )
        util.create_circle(sketch, hole_center, M3_SCREW_THREAD_DIAMETER_MM / 2.0)

    util.extrudeMaxAreaProfile(sketch, extrudeDistanceMM, comp)



def construct_non_motor_side_extender(
    comp: adsk.fusion.Component,
    sketch_offset: float,
    baseArcRadiusMM = BASE_ARC_RADIUS_MM ,
    rodSideWidthMM = ROD_SIDE_WIDTH_MM,
    extrudeDistanceMM: float = EXTENDER_EXTRUDE_DISTANCE_MM,


):
    """Constructs the non-motor-side extender onto joint 3."""
    sketch = util.create_sketch_on_offset_plane(comp, sketch_offset)
    origin = adsk.core.Point3D.create(0, 0, 0)

    outer_circle = util.create_circle(sketch, origin, EXTENDER_OUTER_DIAMETER_MM / 2.0)
    inner_circle = util.create_circle(sketch, origin, EXTENDER_INNER_DIAMETER_MM / 2.0)
    rodHoleSideXCM = (baseArcRadiusMM + ARC_THICKNESS_MM + CONE_WIDTH_MM) / 10.0
    rodHoleSideLow = adsk.core.Point3D.create(rodHoleSideXCM, -rodSideWidthMM / 10.0, 0)
    rodHoleSideHigh = adsk.core.Point3D.create(rodHoleSideXCM, rodSideWidthMM / 10.0, 0)
    rad = math.radians(SMALLER_ARC_ANGLE_DEG)
    circleRadiusCM = (EXTENDER_OUTER_DIAMETER_MM) / 20.0
    p1 = adsk.core.Point3D.create(
        circleRadiusCM * math.cos(rad),
        circleRadiusCM * math.sin(rad),
        0
    )
    p2 = adsk.core.Point3D.create(
        circleRadiusCM * math.cos(rad),
        -circleRadiusCM * math.sin(rad),
        0
    )

    lines = sketch.sketchCurves.sketchLines
    upper_line = lines.addByTwoPoints(p1, rodHoleSideHigh)
    lower_line = lines.addByTwoPoints(p2, rodHoleSideLow)
    end_line = lines.addByTwoPoints(rodHoleSideLow, rodHoleSideHigh)

    p3 = adsk.core.Point3D.create(circleRadiusCM,0,0)
    outer_circle.trim(p3)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 7)/10,0,0), M3_SCREW_THREAD_DIAMETER_MM/2)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 25)/10,15/10,0), M3_SCREW_THREAD_DIAMETER_MM/2)
    util.create_circle(sketch, adsk.core.Point3D.create((baseArcRadiusMM + ARC_THICKNESS_MM + 25)/10,-15/10,0), M3_SCREW_THREAD_DIAMETER_MM/2)
    rodHoleSideXCM = (baseArcRadiusMM + ARC_THICKNESS_MM + CONE_WIDTH_MM) / 10.0
    screw_center = adsk.core.Point3D.create(rodHoleSideXCM - ROD_SCREW_HOLE_INSET_MM / 10.0, 0.0, 0.0)
    util.create_circle(sketch, screw_center, 3.1 / 2.0)


    # 4 holes for joint connection screws (PCD 52 mm, rotated by 30 degrees)
    joint_connection_screws_pcd_cm = JOINT_CONNECTION_SCREWS_PCD_MM / 20.0
    for i in range(4):
        angle_rad = math.radians(30.0 + i * 90.0)
        hole_center = adsk.core.Point3D.create(
            joint_connection_screws_pcd_cm * math.cos(angle_rad),
            joint_connection_screws_pcd_cm * math.sin(angle_rad),
            0.0
        )
        util.create_circle(sketch, hole_center, M3_SCREW_THREAD_DIAMETER_MM / 2.0)

    # 4 holes for magnetic encoder (M3 screw, PCD 22.5 mm, aligned with axes at 0 degrees)
    encoder_pcd_cm = MAGNETIC_ENCODER_PCD_MM / 20.0
    for i in range(4):
        angle_rad = math.radians(i * 90.0)
        hole_center = adsk.core.Point3D.create(
            encoder_pcd_cm * math.cos(angle_rad),
            encoder_pcd_cm * math.sin(angle_rad),
            0.0
        )
        util.create_circle(sketch, hole_center, M3_SCREW_THREAD_DIAMETER_MM / 2.0)

    util.extrudeMaxAreaProfile(sketch, extrudeDistanceMM, comp)


