#include "skeletonbuilder.h"

void SkeletonBuilder::createSkeleton()
{
    m_jointMap.clear();

    // Root
    MObject spineBase = createJoint("SpineBase", MObject::kNullObj, MVector(0, 100, 0));
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

    MObject footL = createJoint("FootLeft", ankleL, MVector(0, -10, 10));
    m_jointMap["FootLeft"] = footL;


    // RIGHT LEG (positive X)
    MObject hipR = createJoint("HipRight", spineBase, MVector(10, -10, 0));
    m_jointMap["HipRight"] = hipR;

    MObject kneeR = createJoint("KneeRight", hipR, MVector(0, -40, 0));
    m_jointMap["KneeRight"] = kneeR;

    MObject ankleR = createJoint("AnkleRight", kneeR, MVector(0, -40, 0));
    m_jointMap["AnkleRight"] = ankleR;

    MObject footR = createJoint("FootRight", ankleR, MVector(0, -10, 10));
    m_jointMap["FootRight"] = footR;
    MGlobal::displayInfo("Kinect skeleton created successfully.");
}



MObject SkeletonBuilder::createJoint(const MString& name,
    const MObject& parent,
    const MVector& position)
{
    MStatus status;

    MFnIkJoint jointFn;
    MObject joint = jointFn.create(parent, &status);

    if (!status)
    {
        MGlobal::displayError("Failed to create joint: " + name);
        return MObject::kNullObj;
    }

    jointFn.setName(name);
    jointFn.setTranslation(position, MSpace::kTransform);

    return joint;
}

MObject& SkeletonBuilder::getJoint(const std::string& name)
{
    return m_jointMap[name];
}