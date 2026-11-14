#include "skeletonbuilder.h"
#include <maya/MFnTransform.h>

bool SkeletonBuilder::findJoint(const MString& mayaName, MObject& outJoint)
{
    MSelectionList sel;
    if (sel.add(mayaName) != MS::kSuccess || sel.isEmpty())
    {
        return false;
    }

    MObject node;
    if (sel.getDependNode(0, node) != MS::kSuccess || !node.hasFn(MFn::kJoint))
    {
        return false;
    }

    outJoint = node;
    return true;
}

bool SkeletonBuilder::skeletonExists() const
{
    for (const char* name : kJointNames)
    {
        MObject joint;
        if (!findJoint(mayaName(name), joint))
        {
            return false;
        }
    }

    return true;
}

bool SkeletonBuilder::findExistingSkeleton()
{
    std::map<std::string, MObject> found;
    for (const char* name : kJointNames)
    {
        MObject joint;
        if (!findJoint(mayaName(name), joint))
        {
            MGlobal::displayWarning("Kinect skeleton is incomplete, missing joint: " + mayaName(name));
            return false;
        }
        found[name] = joint;
    }

    m_jointMap = std::move(found);
    MGlobal::displayInfo("Kinect skeleton found in scene.");
    return true;
}

MStringArray SkeletonBuilder::getMayaJointNames() const
{
    MStringArray names;
    for (const char* name : kJointNames)
    {
        names.append(mayaName(name));
    }
    return names;
}

MObject SkeletonBuilder::findOrCreateGroup()
{
    MSelectionList sel;
    MObject group;
    if (sel.add(kGroupName) == MS::kSuccess && !sel.isEmpty()
        && sel.getDependNode(0, group) == MS::kSuccess && group.hasFn(MFn::kTransform))
    {
        return group;
    }

    // Default transform (no rotation, scale 1) so the joint orientations stay unchanged
    MFnTransform groupFn;
    group = groupFn.create(MObject::kNullObj);
    groupFn.setName(kGroupName);
    return group;
}

void SkeletonBuilder::createSkeleton()
{
    m_jointMap.clear();

    // Root
    MObject spineBase = createJoint("SpineBase", findOrCreateGroup(), MVector(0, 100, 0), 3.14159265358979323846);
    m_jointMap["SpineBase"] = spineBase;

    MObject spineMid = createJoint("SpineMid", spineBase, MVector(0, 20, 0));
    m_jointMap["SpineMid"] = spineMid;

    MObject spineShoulder = createJoint("SpineShoulder", spineMid, MVector(0, 20, 0));
    m_jointMap["SpineShoulder"] = spineShoulder;

    MObject neck = createJoint("Neck", spineShoulder, MVector(0, 10, 0));
    m_jointMap["Neck"] = neck;

    MObject head = createJoint("Head", neck, MVector(0, 15, 0));
    m_jointMap["Head"] = head;


    // LEFT ARM  (negative X)
    MObject shoulderL = createJoint("ShoulderLeft", spineShoulder, MVector(-15, 5, 0));
    m_jointMap["ShoulderLeft"] = shoulderL;

    MObject elbowL = createJoint("ElbowLeft", shoulderL, MVector(-25, 0, 0));
    m_jointMap["ElbowLeft"] = elbowL;

    MObject wristL = createJoint("WristLeft", elbowL, MVector(-25, 0, 0));
    m_jointMap["WristLeft"] = wristL;

    MObject handL = createJoint("HandLeft", wristL, MVector(-15, 0, 0));
    m_jointMap["HandLeft"] = handL;


    // RIGHT ARM (positive X)
    MObject shoulderR = createJoint("ShoulderRight", spineShoulder, MVector(15, 5, 0));
    m_jointMap["ShoulderRight"] = shoulderR;

    MObject elbowR = createJoint("ElbowRight", shoulderR, MVector(25, 0, 0));
    m_jointMap["ElbowRight"] = elbowR;

    MObject wristR = createJoint("WristRight", elbowR, MVector(25, 0, 0));
    m_jointMap["WristRight"] = wristR;

    MObject handR = createJoint("HandRight", wristR, MVector(15, 0, 0));
    m_jointMap["HandRight"] = handR;


    // LEFT LEG (negative X)
    MObject hipL = createJoint("HipLeft", spineBase, MVector(-10, -10, 0));
    m_jointMap["HipLeft"] = hipL;

    MObject kneeL = createJoint("KneeLeft", hipL, MVector(0, -40, 0));
    m_jointMap["KneeLeft"] = kneeL;

    MObject ankleL = createJoint("AnkleLeft", kneeL, MVector(0, -40, 0));
    m_jointMap["AnkleLeft"] = ankleL;

    MObject footL = createJoint("FootLeft", ankleL, MVector(0, -10, -10));
    m_jointMap["FootLeft"] = footL;


    // RIGHT LEG (positive X)
    MObject hipR = createJoint("HipRight", spineBase, MVector(10, -10, 0));
    m_jointMap["HipRight"] = hipR;

    MObject kneeR = createJoint("KneeRight", hipR, MVector(0, -40, 0));
    m_jointMap["KneeRight"] = kneeR;

    MObject ankleR = createJoint("AnkleRight", kneeR, MVector(0, -40, 0));
    m_jointMap["AnkleRight"] = ankleR;

    MObject footR = createJoint("FootRight", ankleR, MVector(0, -10, -10));
    m_jointMap["FootRight"] = footR;
    MGlobal::displayInfo("Kinect skeleton created successfully.");
}



MObject SkeletonBuilder::createJoint(const MString& name,
    const MObject& parent,
    const MVector& position,
    const double oriatinAngel)
{
    MStatus status;

    MFnIkJoint jointFn;
    MObject joint = jointFn.create(parent, &status);

    if (!status)
    {
        MGlobal::displayError("Failed to create joint: " + name);
        return MObject::kNullObj;
    }

    const MString wantedName = MString(kPrefix) + name;
    const MString actualName = jointFn.setName(wantedName);
    if (actualName != wantedName)
    {
        // A node with this name already exists (e.g. a partial skeleton), Maya added a number
        MGlobal::displayWarning("Joint name already in use, created " + actualName + " instead of " + wantedName);
    }
	jointFn.setOrientation(MEulerRotation(0, oriatinAngel, 0));
    jointFn.setTranslation(position, MSpace::kTransform);

    return joint;
}

MObject& SkeletonBuilder::getJoint(const std::string& name)
{
    return m_jointMap[name];
}