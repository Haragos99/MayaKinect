#include "mayakinectcmd.h"
#include <maya/MGlobal.h>

MStatus MayaKinectCmd::doIt(const MArgList&)
{
	auto& contoller = KinectController::instance();
	contoller.initKinect();
	return MS::kSuccess;
}


MStatus SetKinectRotationFilterCmd::doIt(const MArgList& args)
{
	auto& controller = KinectController::instance();
	if (args.length() == 0)
	{
		setResult(controller.isRotationFilterEnabled());
		return MS::kSuccess;
	}

	if (args.length() != 1)
	{
		MGlobal::displayError("Expected zero arguments to query or one boolean argument to set the rotation filter.");
		return MS::kFailure;
	}

	MStatus status;
	const int enabled = args.asInt(0, &status);
	if (status != MS::kSuccess || (enabled != 0 && enabled != 1))
	{
		MGlobal::displayError("Rotation filter value must be 0 or 1.");
		return MS::kFailure;
	}

	controller.setRotationFilterEnabled(enabled != 0);
	setResult(controller.isRotationFilterEnabled());
	return MS::kSuccess;
}
