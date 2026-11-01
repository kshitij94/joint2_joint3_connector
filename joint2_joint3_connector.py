"""Joint 2 to Joint 3 Connector Script for Autodesk Fusion 360."""

import importlib
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

import constants
import util
import base
import extender
importlib.reload(constants)
importlib.reload(util)
importlib.reload(base)
importlib.reload(extender)

BASE_SKETCH_OFFSET = 5.0  # mm
NON_MOTOR_EXTENDER_SKETCH_OFFSET = 0.0  # mm

app = adsk.core.Application.get()
ui = app.userInterface


def save_stl(design: adsk.fusion.Design, components_to_export: list) -> list:
    """Exports components as STL files into the user's Downloads directory (matching joint3)."""
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


def run(context):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design found.", "No Design")
            return

        non_motor_extender_comp = util.create_component(design, "Non Motor Side Extender")
        extender.construct_non_motor_side_extender(
            comp=non_motor_extender_comp,
            sketch_offset=NON_MOTOR_EXTENDER_SKETCH_OFFSET,
        )
        non_motor_extender_height = util.get_body_height(non_motor_extender_comp)

        base_comp = util.create_component(design, "Base")
        base.construct_base(
            comp=base_comp,
            sketch_offset=non_motor_extender_height,
        )

        base_comp_height = util.get_body_height(base_comp)

        motor_extender_comp = util.create_component(design, "Motor Side Extender")
        extender.construct_motor_side_extender(
            comp=motor_extender_comp,
            sketch_offset=NON_MOTOR_EXTENDER_SKETCH_OFFSET + base_comp_height + non_motor_extender_height,
        )

        components_to_export = [
            (base_comp, "Base"),
            (non_motor_extender_comp, "Non Motor Side Extender"),
            (motor_extender_comp, "Motor Side Extender"),
        ]
        save_stl(design, components_to_export)

    except:
        if ui:
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")

