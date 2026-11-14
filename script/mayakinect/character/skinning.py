"""Skin meshes to the skeleton in its rest pose, whatever pose it is in right now."""
import maya.cmds as cmds

from .. import skeleton_def

TMP_SUFFIX = "_restBindTmp"


def _short(name):
    return name.split("|")[-1].split(":")[-1]


def _make_rest_copy(joints):
    """Duplicate the skeleton hierarchy and put it in the rest pose. Returns {real joint: tmp joint}."""
    root = joints[0]
    # The copy stays under the same parent (Kinect_grp), so its rest world matrices
    # match the real skeleton's rest pose exactly
    copy_root = cmds.duplicate(root, name=_short(root) + TMP_SUFFIX)[0]
    copy_root = (cmds.ls(copy_root, long=True) or [copy_root])[0]

    # The duplicate can bring constraint nodes and other non-joint children along
    extra = [n for n in (cmds.listRelatives(copy_root, allDescendents=True, fullPath=True) or [])
             if cmds.nodeType(n) != "joint"]
    if extra:
        cmds.delete(extra)

    # Give every copied joint a unique name (<joint>_restBindTmp). Deepest first, so the
    # full paths of the joints still to be renamed stay valid.
    children = cmds.listRelatives(copy_root, allDescendents=True, type="joint", fullPath=True) or []
    for c in sorted(children, key=lambda p: p.count("|"), reverse=True):
        cmds.rename(c, _short(c) + TMP_SUFFIX)

    # Map the copied joints back to the real ones by name
    copies = [copy_root] + (cmds.listRelatives(copy_root, allDescendents=True, type="joint", fullPath=True) or [])
    by_short = {}
    for c in copies:
        by_short[_short(c)[:-len(TMP_SUFFIX)]] = c

    mapping = {}
    for j in joints:
        tmp = by_short.get(_short(j))
        if tmp is None:
            cmds.delete(copy_root)
            raise RuntimeError("rest_bind: joint {} not found in the rest copy".format(j))
        mapping[j] = tmp

    # Rest pose: no rotation on any joint (orientation lives in jointOrient), root at rest position
    for tmp in mapping.values():
        for attr in ("rotateX", "rotateY", "rotateZ"):
            cmds.setAttr("{}.{}".format(tmp, attr), 0)
    cmds.setAttr(mapping[root] + ".translate", *skeleton_def.REST_ROOT, type="double3")

    return copy_root, mapping


def rest_bind(meshes, joints=None, max_influences=4):
    """Skin every mesh transform in `meshes` to `joints` (default: skeleton_def.skin_joints())
    in the rest pose. Returns the list of created skinClusters."""
    joints = list(joints or skeleton_def.skin_joints())
    missing = [j for j in joints if not cmds.objExists(j)]
    if missing:
        raise RuntimeError("rest_bind: joints missing in the scene: {}".format(missing))

    # Full paths, so names stay unambiguous while the copy exists
    joints = [cmds.ls(j, long=True)[0] for j in joints]
    copy_root, mapping = _make_rest_copy(joints)
    tmp_to_real = {_short(t): r for r, t in mapping.items()}
    skin_clusters = []
    try:
        for mesh in meshes:
            sc = cmds.skinCluster(
                list(mapping.values()), mesh,
                toSelectedBones=True,
                maximumInfluences=max_influences,
                normalizeWeights=1,
                bindMethod=0,               # closest distance
                name=_short(mesh) + "_skinCluster",
            )[0]

            # Reconnect each influence from the tmp joint to the real joint
            for plug in cmds.getAttr(sc + ".matrix", multiIndices=True) or []:
                src = cmds.listConnections("{}.matrix[{}]".format(sc, plug), source=True, destination=False)
                if not src:
                    continue
                real = tmp_to_real.get(_short(src[0]))
                if real is None:
                    continue
                cmds.connectAttr(real + ".worldMatrix[0]", "{}.matrix[{}]".format(sc, plug), force=True)
                if cmds.objExists("{}.lockWeights[{}]".format(sc, plug)):
                    # skinCluster adds this attribute to its influences; a joint that was
                    # never bound before doesn't have it yet
                    if not cmds.attributeQuery("lockInfluenceWeights", node=real, exists=True):
                        cmds.addAttr(real, longName="lockInfluenceWeights", shortName="liw",
                                     attributeType="bool", defaultValue=False)
                    cmds.connectAttr(real + ".lockInfluenceWeights", "{}.lockWeights[{}]".format(sc, plug), force=True)
                if cmds.attributeQuery("objectColorRGB", node=real, exists=True):
                    cmds.connectAttr(real + ".objectColorRGB", "{}.influenceColor[{}]".format(sc, plug), force=True)

            # The bind pose node references the tmp joints; it's not needed
            poses = cmds.listConnections(sc + ".bindPose", source=True, destination=False) or []
            if poses:
                cmds.delete(poses)

            skin_clusters.append(sc)
    finally:
        if cmds.objExists(copy_root):
            cmds.delete(copy_root)

    return skin_clusters
