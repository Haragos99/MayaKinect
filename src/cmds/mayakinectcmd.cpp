#include "mayakinectcmd.h"
#include <maya/MGlobal.h>



MStatus MayaKinectCmd::doIt(const MArgList&)
{
	auto& contoller = KinectController::instance();
	contoller.initKinect();
	return MS::kSuccess;
}
