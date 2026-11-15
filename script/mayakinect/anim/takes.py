"""Save and load recorded Kinect takes as JSON files in takes/."""

import datetime
import json
import re
from pathlib import Path

import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds
import maya.mel as mel

from .. import paths, skeleton_def

FORMAT_VERSION = 1
ROTATE_ATTRS = ("rotateX", "rotateY", "rotateZ")
TRANSLATE_ATTRS = ("translateX", "translateY", "translateZ")

# Scene time units for common frame rates (others use Maya's "<n>fps" units)
_FPS_UNITS = {24.0: "film", 25.0: "pal", 30.0: "ntsc", 48.0: "show", 50.0: "palf", 60.0: "ntscf"}


# scene helpers

def scene_fps():
    return float(mel.eval("currentTimeUnitToFPS()"))


def set_scene_fps(fps):
    """Set the scene frame rate. Returns False if Maya has no time unit for it."""
    unit = _FPS_UNITS.get(float(fps), "{:g}fps".format(fps))
    try:
        cmds.currentUnit(time=unit)
        return True
    except RuntimeError:
        return False


def mocap_attrs():
    """Every attribute a take can have keys on: rotate for all joints, translate for the root."""
    attrs = []
    for joint in skeleton_def.mocap_joints():
        attrs += ["{}.{}".format(joint, a) for a in ROTATE_ATTRS]
    root = skeleton_def.MOCAP_PREFIX + skeleton_def.ROOT
    attrs += ["{}.{}".format(root, a) for a in TRANSLATE_ATTRS]
    return attrs


def _existing_joints():
    return [j for j in skeleton_def.mocap_joints() if cmds.objExists(j)]


def mocap_has_keys():
    joints = _existing_joints()
    return bool(joints) and bool(cmds.keyframe(joints, query=True, keyframeCount=True))


def clear_mocap_keys():
    """Remove the take from the Mocap_* joints (needed before streaming live again)."""
    joints = _existing_joints()
    if joints:
        cmds.cutKey(joints, clear=True, attribute=list(ROTATE_ATTRS + TRANSLATE_ATTRS))


def mocap_key_range():
    """(first, last) keyed frame on the Mocap_* joints, or None."""
    joints = _existing_joints()
    times = cmds.keyframe(joints, query=True, timeChange=True) if joints else None
    if not times:
        return None
    return int(round(min(times))), int(round(max(times)))


def _plug(attr):
    sel = om.MSelectionList()
    sel.add(attr)
    return sel.getPlug(0)


def _curve_for(plug):
    curves = oma.MAnimUtil.findAnimation(plug)
    return oma.MFnAnimCurve(curves[0]) if len(curves) else None


# take files

def take_path(name):
    return paths.TAKES_DIR / (name + ".json")


def valid_name(name):
    """Take names become file names: letters, digits, _ and - only."""
    return bool(re.fullmatch(r"[A-Za-z0-9_\-]+", name or ""))


def next_take_name(prefix="Take_"):
    """First unused Take_### name."""
    used = set()
    if paths.TAKES_DIR.exists():
        for p in paths.TAKES_DIR.glob(prefix + "*.json"):
            m = re.fullmatch(re.escape(prefix) + r"(\d+)", p.stem)
            if m:
                used.add(int(m.group(1)))
    n = 1
    while n in used:
        n += 1
    return "{}{:03d}".format(prefix, n)


def save_take(name, overwrite=False):
    """Write the keys on the Mocap_* joints to takes/<name>.json. Returns the file path."""
    if not valid_name(name):
        raise ValueError("Invalid take name '{}': use letters, digits, _ and -".format(name))
    path = take_path(name)
    if path.exists() and not overwrite:
        raise FileExistsError("Take already exists: {}".format(path))

    ui_unit = om.MTime.uiUnit()
    curves = {}
    for attr in mocap_attrs():
        if not cmds.objExists(attr):
            continue
        curve = _curve_for(_plug(attr))
        if curve is None or curve.numKeys == 0:
            continue
        curves[attr] = [
            [round(curve.input(i).asUnits(ui_unit), 4), curve.value(i)]   # frame, internal value
            for i in range(curve.numKeys)
        ]
    if not curves:
        raise RuntimeError("No recorded keys on the Mocap joints to save.")

    times = [k[0] for keys in curves.values() for k in keys]
    from ..character import characters   # optional info, avoid a hard dependency at import
    data = {
        "version": FORMAT_VERSION,
        "name": name,
        "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "fps": scene_fps(),
        "start": int(round(min(times))),
        "end": int(round(max(times))),
        "model": characters.active() or "",
        "units": {"time": "frames", "angle": "rad", "linear": "cm"},
        "curves": curves,
    }

    paths.TAKES_DIR.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, separators=(",", ":"))
    tmp.replace(path)   # write-then-rename: a crash never leaves a half-written take
    return path


def read_take_info(path):
    """Metadata of a take file without the curves (for the takes list)."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data.pop("curves", None)
    data["path"] = str(path)
    return data


def list_takes():
    """Metadata of every take in takes/, newest first. Unreadable files are skipped."""
    if not paths.TAKES_DIR.exists():
        return []
    infos = []
    for p in paths.TAKES_DIR.glob("*.json"):
        try:
            infos.append(read_take_info(p))
        except (OSError, ValueError) as e:
            print("mayakinect: skipping take {}: {}".format(p.name, e))
    infos.sort(key=lambda d: d.get("date", ""), reverse=True)
    return infos


def load_take(path):
    """Replace the keys on the Mocap_* joints with the take in `path`. Returns its metadata."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data.get("version") != FORMAT_VERSION:
        raise RuntimeError("Unsupported take version: {}".format(data.get("version")))

    fps = float(data.get("fps", 30.0))
    if abs(scene_fps() - fps) > 1e-3 and not set_scene_fps(fps):
        cmds.warning("Take was recorded at {} fps; the scene could not be switched to it.".format(fps))

    if not all(cmds.objExists(j) for j in skeleton_def.mocap_joints()):
        cmds.createKinectSkeleton()
    clear_mocap_keys()

    ui_unit = om.MTime.uiUnit()
    for attr, keys in data["curves"].items():
        if not cmds.objExists(attr) or not keys:
            continue
        plug = _plug(attr)
        curve_type = (oma.MFnAnimCurve.kAnimCurveTL if attr.split(".")[-1].startswith("translate")
                      else oma.MFnAnimCurve.kAnimCurveTA)
        curve = oma.MFnAnimCurve()
        curve.create(plug, curve_type)
        times = om.MTimeArray([om.MTime(k[0], ui_unit) for k in keys])
        values = om.MDoubleArray([k[1] for k in keys])
        curve.addKeys(times, values, oma.MFnAnimCurve.kTangentClamped, oma.MFnAnimCurve.kTangentClamped)

    start, end = int(data["start"]), int(data["end"])
    cmds.playbackOptions(minTime=start, maxTime=end, animationStartTime=start, animationEndTime=end)
    cmds.currentTime(start)
    data.pop("curves", None)
    data["path"] = str(path)
    return data


def delete_take(path):
    Path(path).unlink()
