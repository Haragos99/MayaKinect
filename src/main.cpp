#include <maya/MFnPlugin.h>
#include "mayakinectcmd.h"
#include <maya/MTimerMessage.h>
#include "kinectupdateskeletoncmd.h"
#include "closekinectcmd.h"
#include "importcharactercmd.h"



MStatus initializePlugin(MObject obj)
{
    MFnPlugin plugin(obj, "Gergo Haragos", "1.0", "Any");
    plugin.registerCommand("runMayaKinect", MayaKinectCmd::creator);
    plugin.registerCommand("updateMayaKinect", KinectUpdateSkeletonCmd::creator);
    plugin.registerCommand("closeKinect", CloseKinectCmd::creator);
    plugin.registerCommand("importCharacter", ImportCharacterCmd::creator);
    return MS::kSuccess;
}


MStatus uninitializePlugin(MObject obj)
{
    MFnPlugin plugin(obj);

    plugin.deregisterCommand("runMayaKinect");
    plugin.deregisterCommand("updateMayaKinect");
    plugin.deregisterCommand("closeKinect");
    plugin.deregisterCommand("importCharacter");
    return MS::kSuccess;
}