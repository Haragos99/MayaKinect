"""Import character models once, keep them in the scene, switch between them instantly."""

import os
import maya.cmds as cmds

from .. import paths, skeleton_def
from . import skinning
from .models import ModelInfo, from_path

ROOT_GROUP = "Characters_grp"
MODEL_ATTR = "kinectModel"
NAMESPACE_ATTR = "kinectNamespace"
ACTIVE_ATTR = "activeModel"


# helpers

def model_key(path):
    """Normalised path used to compare models (case-insensitive on Windows)."""
    return os.path.normcase(os.path.abspath(str(path))).replace("\\", "/")


def _root():
    if not cmds.objExists(ROOT_GROUP):
        cmds.group(empty=True, name=ROOT_GROUP)
    if not cmds.attributeQuery(ACTIVE_ATTR, node=ROOT_GROUP, exists=True):
        cmds.addAttr(ROOT_GROUP, longName=ACTIVE_ATTR, dataType="string")
    return ROOT_GROUP


def _model_groups():
    """{normalised obj path: group} for every imported character in the scene."""
    result = {}
    for node in cmds.ls("*." + MODEL_ATTR, objectsOnly=True, long=True) or []:
        value = cmds.getAttr(node + "." + MODEL_ATTR)
        if value:
            result[model_key(value)] = node
    return result


def _meshes(group):
    """Mesh transforms under a character group (no intermediate shapes)."""
    shapes = cmds.listRelatives(group, allDescendents=True, type="mesh", fullPath=True) or []
    shapes = [s for s in shapes if not cmds.getAttr(s + ".intermediateObject")]
    transforms = cmds.listRelatives(shapes, parent=True, fullPath=True) or []
    return list(dict.fromkeys(transforms))


def _skin_clusters(group):
    clusters = []
    for mesh in _meshes(group):
        clusters += cmds.ls(cmds.listHistory(mesh, pruneDagObjects=True) or [], type="skinCluster")
    return list(dict.fromkeys(clusters))


def _unique_namespace(base):
    ns, i = base, 1
    while cmds.namespace(exists=ns):
        i += 1
        ns = "{}{}".format(base, i)
    return ns


def _ensure_skeleton():
    joints = skeleton_def.skin_joints()
    if not all(cmds.objExists(j) for j in joints):
        cmds.createKinectSkeleton()
        joints = skeleton_def.skin_joints()
    return joints


def _as_model(model):
    return model if isinstance(model, ModelInfo) else from_path(model)


# public API

def cached():
    """All imported characters: list of (obj path, group)."""
    return [(cmds.getAttr(g + "." + MODEL_ATTR), g) for g in _model_groups().values()]


def find_group(model):
    """Group of an already imported model, or None."""
    return _model_groups().get(model_key(_as_model(model).obj))


def active():
    """Obj path of the visible model, or None."""
    if not cmds.objExists(ROOT_GROUP) or not cmds.attributeQuery(ACTIVE_ATTR, node=ROOT_GROUP, exists=True):
        return None
    value = cmds.getAttr(ROOT_GROUP + "." + ACTIVE_ATTR)
    if value and model_key(value) in _model_groups():
        return value
    return None


def is_active(model):
    current = active()
    return current is not None and model_key(current) == model_key(_as_model(model).obj)


def ensure_loaded(model):
    """Import and skin the model if it isn't in the scene yet. Returns its group."""
    model = _as_model(model)
    existing = find_group(model)
    if existing:
        return existing

    if not os.path.isfile(str(model.obj)):
        raise RuntimeError("Model file not found: {}".format(model.obj))

    joints = _ensure_skeleton()
    cmds.loadPlugin("objExport", quiet=True)

    cmds.undoInfo(openChunk=True, chunkName="importKinectCharacter")
    try:
        namespace = _unique_namespace("char_" + model.name)
        new_nodes = cmds.file(
            paths.maya_path(model.obj), i=True, type="OBJ",
            namespace=namespace, returnNewNodes=True,
            options="mo=1", ignoreVersion=True,
        ) or []

        new_transforms = [n for n in cmds.ls(new_nodes, type="transform", long=True) or []]
        top = [t for t in new_transforms if not cmds.listRelatives(t, parent=True)]
        if not top:
            raise RuntimeError("No geometry found in {}".format(model.obj))

        group = cmds.group(empty=True, name="Char_{}_grp".format(model.name), parent=_root())
        group = cmds.ls(group, long=True)[0]
        cmds.addAttr(group, longName=MODEL_ATTR, dataType="string")
        cmds.setAttr(group + "." + MODEL_ATTR, paths.maya_path(model.obj), type="string")
        cmds.addAttr(group, longName=NAMESPACE_ATTR, dataType="string")
        cmds.setAttr(group + "." + NAMESPACE_ATTR, namespace, type="string")
        cmds.parent(top, group)

        meshes = _meshes(group)
        if not meshes:
            raise RuntimeError("No meshes found in {}".format(model.obj))

        # Per-model scale/offset from models.json, frozen into the mesh before binding
        if model.scale != 1.0 or any(model.offset):
            for mesh in meshes:
                cmds.xform(mesh, scale=(model.scale,) * 3, translation=model.offset, worldSpace=True, relative=True)
                cmds.makeIdentity(mesh, apply=True, translate=True, rotate=True, scale=True)

        skinning.rest_bind(meshes, joints)
    except Exception:
        cmds.undoInfo(closeChunk=True)
        raise
    cmds.undoInfo(closeChunk=True)

    print("mayakinect: imported {} ({} mesh(es))".format(model.display_name, len(meshes)))
    return group


def switch_to(model):
    """Make `model` the visible, deforming character (imports it on first use). Returns its group."""
    model = _as_model(model)
    target = ensure_loaded(model)
    target_key = model_key(model.obj)

    for key, group in _model_groups().items():
        visible = key == target_key
        cmds.setAttr(group + ".visibility", visible)
        for sc in _skin_clusters(group):
            cmds.setAttr(sc + ".envelope", 1.0 if visible else 0.0)

    cmds.setAttr(_root() + "." + ACTIVE_ATTR, paths.maya_path(model.obj), type="string")
    return target


def remove(model):
    """Delete an imported model from the scene (and its namespace)."""
    model = _as_model(model)
    group = find_group(model)
    if not group:
        return False

    namespace = cmds.getAttr(group + "." + NAMESPACE_ATTR) if cmds.attributeQuery(
        NAMESPACE_ATTR, node=group, exists=True) else None
    was_active = is_active(model)
    cmds.delete(group)
    if namespace and cmds.namespace(exists=namespace):
        cmds.namespace(removeNamespace=namespace, deleteNamespaceContent=True)
    if was_active:
        cmds.setAttr(_root() + "." + ACTIVE_ATTR, "", type="string")
    return True
