import os
import sys

import maya.cmds as cmds

# Project root. Uses this file's location when run as a file; the fallback is for
# pasting the script into the Script Editor, where __file__ is not defined.
try:
    PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:
    PROJECT_ROOT = r"C:\Users\Geri\Documents\Projects\CG\MayaKinect"

SCRIPT_DIR = os.path.join(PROJECT_ROOT, "script")

# Plugin build to load (Debug; change to x64-Release for the release build)
PLUGIN_PATH = os.path.join(PROJECT_ROOT, "out", "build", "x64-Debug", "MayaKinect.mll")

# Make the mayakinect Python package importable
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

if not cmds.pluginInfo(PLUGIN_PATH, query=True, loaded=True):
    cmds.loadPlugin(PLUGIN_PATH)

print("MayaKinect plugin loaded: " + PLUGIN_PATH)
