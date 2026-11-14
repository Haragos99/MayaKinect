#pragma once
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include "../kinect/kincetcontroller.h"

// createKinectSkeleton
// Creates the Mocap_* skeleton, or finds the one already in the scene.
// Works without a Kinect connected. Returns the Maya joint names.
class CreateKinectSkeletonCmd : public MPxCommand {

public:
    CreateKinectSkeletonCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new CreateKinectSkeletonCmd; }
};

// setKinectDrive [0|1]
// No argument: query. 0: the stream stops moving the joints, 1: it moves them again.
class SetKinectDriveCmd : public MPxCommand {

public:
    SetKinectDriveCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new SetKinectDriveCmd; }
};
