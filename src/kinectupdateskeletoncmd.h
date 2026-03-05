#pragma once
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include "kincetcontroller.h"
#include <maya/MArgList.h>

class KinectUpdateSkeletonCmd : public MPxCommand {

public:
    KinectUpdateSkeletonCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new KinectUpdateSkeletonCmd; }

private:


};