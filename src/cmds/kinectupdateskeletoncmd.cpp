#include "kinectupdateskeletoncmd.h"

unsigned char* KinectUpdateSkeletonCmd:: getPythonImageBuffer(const MString& addressString)
{
    uintptr_t address = std::stoull(addressString.asChar());

    return reinterpret_cast<unsigned char*>(address);
}

MStatus KinectUpdateSkeletonCmd::doIt(const MArgList& args)
{
    if (args.length() != 2) 
    {
        MGlobal::displayError("Expected 2 argument: the memory address as a string and a buffer size for check.");
        return MS::kFailure;
    }

    try
    {
		MString addressString = args.asString(0);
        const size_t bufferSize = std::stoull(args.asString(1).asChar());
		constexpr size_t requiredSize = 640 * 480 * 4; // RGBA image size for Kinect

		if (addressString.isEmpty() || bufferSize < requiredSize) // Check if the buffer is valid and has enough capacity
        {
            MGlobal::displayError( "Invalid image buffer or insufficient capacity.");
            return MS::kFailure;
        }

        auto* outBuffer = getPythonImageBuffer(addressString);

        auto& controller = KinectController::instance();
        controller.updateMayaSkeleton();

        bool status = controller.getImageBuffer(outBuffer);
        setResult(status);

        return MS::kSuccess;
    }
    catch (const std::exception& e)
    {
        MGlobal::displayError(e.what());
        return MS::kFailure;
    }
}
