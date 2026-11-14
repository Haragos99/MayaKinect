#include <maya/MFnPlugin.h>
#include <maya/MTimerMessage.h>
#include "cmds/closekinectcmd.h"
#include "cmds/kinectupdateskeletoncmd.h"
#include "cmds/mayakinectcmd.h"
#include "cmds/skeletoncmds.h"
#include "cmds/recordingcmds.h"



MStatus initializePlugin(MObject obj)
{
    MFnPlugin plugin(obj, "Gergo Haragos", "1.0", "Any");
    plugin.registerCommand("runMayaKinect", MayaKinectCmd::creator);
    plugin.registerCommand("setKinectRotationFilter", SetKinectRotationFilterCmd::creator);
    plugin.registerCommand("updateMayaKinect", KinectUpdateSkeletonCmd::creator);
    plugin.registerCommand("closeKinect", CloseKinectCmd::creator);
    plugin.registerCommand("createKinectSkeleton", CreateKinectSkeletonCmd::creator);
    plugin.registerCommand("setKinectDrive", SetKinectDriveCmd::creator);
    plugin.registerCommand("startKinectRecording", StartKinectRecordingCmd::creator, StartKinectRecordingCmd::newSyntax);
    plugin.registerCommand("stopKinectRecording", StopKinectRecordingCmd::creator);
    plugin.registerCommand("isKinectRecording", IsKinectRecordingCmd::creator);
    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
    MFnPlugin plugin(obj);

    plugin.deregisterCommand("runMayaKinect");
    plugin.deregisterCommand("setKinectRotationFilter");
    plugin.deregisterCommand("updateMayaKinect");
    plugin.deregisterCommand("closeKinect");
    plugin.deregisterCommand("createKinectSkeleton");
    plugin.deregisterCommand("setKinectDrive");
    plugin.deregisterCommand("startKinectRecording");
    plugin.deregisterCommand("stopKinectRecording");
    plugin.deregisterCommand("isKinectRecording");
    return MS::kSuccess;
}