#pragma once
#include <maya/MObject.h>
#include <map>
#include <maya/MFnIkJoint.h>
#include <maya/MVector.h>
#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>

class SkeletonBuilder
{
public:
    void createSkeleton();
    bool skeletonExists() const;
    MObject& getJoint(const std::string& name);
    std::map<std::string, MObject>& getJointMap() { return m_jointMap; }

private:
    MObject createJoint(const MString& name, const MObject& parent, const MVector& position, const double oriatinAngel = 0);
    std::map<std::string, MObject> m_jointMap;
};
