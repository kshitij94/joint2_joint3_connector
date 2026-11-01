import os
import adsk.core, adsk.fusion

BUFFER = 0.0  # mm
SEPARATOR = 1.0  # mm

def create_component(design: adsk.fusion.Design, name: str) -> adsk.fusion.Component:
    rootComp = design.rootComponent
    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp

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

from typing import Optional

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

import math

def create_circle(sketch: adsk.fusion.Sketch, center: adsk.core.Point3D, radius_mm: float) -> adsk.fusion.SketchCircle:
    circles = sketch.sketchCurves.sketchCircles
    circle = circles.addByCenterRadius(center, radius_mm / 10.0)
    circle.isFixed = True
    return circle

def find_profiles_in_circle(sketch: adsk.fusion.Sketch, center: adsk.core.Point3D, radius: float) -> adsk.core.ObjectCollection:
    """
    Finds all profiles in a sketch whose centroids lie within the specified circle.
    
    :param sketch: The sketch to search.
    :param center: The center of the circle.
    :param radius: The radius of the circle (in cm).
    :return: An ObjectCollection of the matching Profile objects.
    """
    profiles = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        try:
            area_props = prof.areaProperties()
            centroid = area_props.centroid
            dist = math.sqrt((centroid.x - center.x)**2 + (centroid.y - center.y)**2)
            # Add a tiny tolerance to make sure boundary profiles are caught
            if dist < radius + 1e-5:
                profiles.add(prof)
        except:
            pass
    return profiles

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

def extrudeMaxAreaProfile(
    sketch: adsk.fusion.Sketch,
    extrudeDistanceMM: float,
    comp: Optional[adsk.fusion.Component] = None,
    operation: adsk.fusion.FeatureOperations = adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
) -> Optional[adsk.fusion.ExtrudeFeature]:
    """
    Finds the profile with the largest area in the sketch (e.g. main body containing hole cutouts)
    and extrudes it by the specified distance in mm.
    """
    # Allow (comp, sketch, extrudeDistanceMM) argument order as well
    if isinstance(sketch, adsk.fusion.Component) and isinstance(extrudeDistanceMM, adsk.fusion.Sketch):
        comp, sketch, extrudeDistanceMM = sketch, extrudeDistanceMM, comp  # type: ignore

    if comp is None:
        comp = sketch.parentComponent

    base_prof = None
    max_area = 0.0
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        try:
            area = prof.areaProperties().area
            if area > max_area:
                max_area = area
                base_prof = prof
        except:
            pass

    if not base_prof and sketch.profiles.count > 0:
        base_prof = sketch.profiles.item(0)

    if base_prof:
        extrudes = comp.features.extrudeFeatures
        extInput = extrudes.createInput(base_prof, operation)
        distance = adsk.core.ValueInput.createByReal(extrudeDistanceMM / 10.0)
        extInput.setDistanceExtent(False, distance)
        return extrudes.add(extInput)
    return None


def save_stl(design: adsk.fusion.Design, components_to_export: list) -> list:
    """
    Exports a list of (Component, name) tuples as STL files to the Downloads directory.
    """
    downloads_dir = os.path.join(os.path.expanduser('~'), 'Downloads')
    if not os.path.exists(downloads_dir):
        downloads_dir = os.path.join(os.path.expanduser('~'), 'Desktop')

    export_mgr = design.exportManager
    exported_paths = []
    for comp, name in components_to_export:
        file_path = os.path.join(downloads_dir, f"{name}.stl")
        stl_options = export_mgr.createSTLExportOptions(comp, file_path)
        stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        stl_options.sendToPrintUtility = False
        export_mgr.execute(stl_options)
        exported_paths.append(file_path)
    return exported_paths


