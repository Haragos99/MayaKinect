#pragma once
#include <maya/MObject.h>
#include <map>
#include <array>
#include <string>
#include <maya/MFnIkJoint.h>
#include <maya/MVector.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MStringArray.h>

class SkeletonBuilder
{
public:
    // Prefix of the joint names in the Maya scene ("Mocap_ElbowLeft").
    // The keys of the joint map stay unprefixed ("ElbowLeft").
    static constexpr const char* kPrefix = "Mocap_";
    static constexpr const char* kGroupName = "Kinect_grp";

    // Every joint of the Kinect skeleton. Must match script/mayakinect/skeleton_def.py.
    static constexpr std::array<const char*, 21> kJointNames = {
        "SpineBase", "SpineMid", "SpineShoulder", "Neck", "Head",
        "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft",
        "ShoulderRight", "ElbowRight", "WristRight", "HandRight",
        "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
        "HipRight", "KneeRight", "AnkleRight", "FootRight"
    };

    static MString mayaName(const std::string& name) { return MString(kPrefix) + name.c_str(); }

    void createSkeleton();
    bool skeletonExists() const;
    // Fills the joint map from a skeleton that is already in the scene (e.g. after reopening a file).
    bool findExistingSkeleton();
    MStringArray getMayaJointNames() const;
    MObject& getJoint(const std::string& name);
    std::map<std::string, MObject>& getJointMap() { return m_jointMap; }

private:
    MObject createJoint(const MString& name, const MObject& parent, const MVector& position, const double oriatinAngel = 0);
    MObject findOrCreateGroup();
    static bool findJoint(const MString& mayaName, MObject& outJoint);
    std::map<std::string, MObject> m_jointMap;
};
