#include "closekinectcmd.h"


MStatus CloseKinectCmd::doIt(const MArgList& args)
{
    auto& controller = KinectController::instance();
    controller.stop();

    return MS::kSuccess;
}