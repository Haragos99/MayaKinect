"""Python copy of the Kinect skeleton definition.

Must match SkeletonBuilder::kJointNames and the hierarchy in src/skeleton/skeletonbuilder.cpp.
Run check_against_plugin() after changing either side.
"""

MOCAP_PREFIX = "Mocap_"
BIND_PREFIX = "Bind_"
GROUP_NAME = "Kinect_grp"

# Rest translation of the root joint (SpineBase) in cm
REST_ROOT = (0.0, 100.0, 0.0)

# (joint, parent) in the same order as the C++ list. Parent None = root.
JOINTS = [
    ("SpineBase", None),
    ("SpineMid", "SpineBase"),
    ("SpineShoulder", "SpineMid"),
    ("Neck", "SpineShoulder"),
    ("Head", "Neck"),
    ("ShoulderLeft", "SpineShoulder"),
    ("ElbowLeft", "ShoulderLeft"),
    ("WristLeft", "ElbowLeft"),
    ("HandLeft", "WristLeft"),
    ("ShoulderRight", "SpineShoulder"),
    ("ElbowRight", "ShoulderRight"),
    ("WristRight", "ElbowRight"),
    ("HandRight", "WristRight"),
    ("HipLeft", "SpineBase"),
    ("KneeLeft", "HipLeft"),
    ("AnkleLeft", "KneeLeft"),
    ("FootLeft", "AnkleLeft"),
    ("HipRight", "SpineBase"),
    ("KneeRight", "HipRight"),
    ("AnkleRight", "KneeRight"),
    ("FootRight", "AnkleRight"),
]

JOINT_NAMES = [name for name, _ in JOINTS]
ROOT = JOINT_NAMES[0]


def mocap_joints():
    return [MOCAP_PREFIX + name for name in JOINT_NAMES]


def bind_joints():
    return [BIND_PREFIX + name for name in JOINT_NAMES]


def skin_joints():
    """Joints that meshes are skinned to: Bind_* once the rig exists, otherwise Mocap_*."""
    import maya.cmds as cmds

    bind = bind_joints()
    if all(cmds.objExists(j) for j in bind):
        return bind
    return mocap_joints()


def check_against_plugin():
    """Compare this list with the plugin's. Creates the skeleton if it doesn't exist."""
    import maya.cmds as cmds

    plugin_names = list(cmds.createKinectSkeleton())
    expected = mocap_joints()
    if plugin_names != expected:
        raise RuntimeError(
            "skeleton_def.py and the plugin differ:\n  python: {}\n  plugin: {}".format(expected, plugin_names)
        )

    for name, parent in JOINTS:
        actual = cmds.listRelatives(MOCAP_PREFIX + name, parent=True) or [None]
        wanted = MOCAP_PREFIX + parent if parent else GROUP_NAME
        if actual[0] != wanted:
            raise RuntimeError("{} has parent {}, expected {}".format(name, actual[0], wanted))

    print("skeleton_def: matches the plugin ({} joints)".format(len(expected)))
    return True
