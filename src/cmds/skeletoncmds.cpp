#include "skeletoncmds.h"

MStatus CreateKinectSkeletonCmd::doIt(const MArgList& args)
{
    if (args.length() != 0)
    {
        MGlobal::displayError("createKinectSkeleton takes no arguments.");
        return MS::kFailure;
    }

    auto& controller = KinectController::instance();
    controller.createSkeleton();

    setResult(controller.getSkeletonJointNames());
    return MS::kSuccess;
}


MStatus SetKinectDriveCmd::doIt(const MArgList& args)
{
    auto& controller = KinectController::instance();
    if (args.length() == 0)
    {
        setResult(controller.isDriveEnabled());
        return MS::kSuccess;
    }

    if (args.length() != 1)
    {
        MGlobal::displayError("Expected zero arguments to query or one boolean argument to set the Kinect drive.");
        return MS::kFailure;
    }

    MStatus status;
    const int enabled = args.asInt(0, &status);
    if (status != MS::kSuccess || (enabled != 0 && enabled != 1))
    {
        MGlobal::displayError("Kinect drive value must be 0 or 1.");
        return MS::kFailure;
    }

    controller.setDriveEnabled(enabled != 0);
    setResult(controller.isDriveEnabled());
    return MS::kSuccess;
}
