#include "kinectupdateskeletoncmd.h"

MStatus KinectUpdateSkeletonCmd::doIt(const MArgList& args)
{
    if (args.length() != 1) {
        MGlobal::displayError("Expected 1 argument: the memory address as a string.");
        return MS::kFailure;
    }
    // We pass the memory address as a string to avoid Maya MEL's 32-bit integer limits
    MString addrStr = args.asString(0);
    unsigned long long addr = std::stoull(addrStr.asChar());
    unsigned char* outBuffer = reinterpret_cast<unsigned char*>(addr);

    auto& controller = KinectController::instance();
    controller.updateMayaSkeleton();
    bool status = controller.getImageBuffer(outBuffer);
    setResult(status);
    return MS::kSuccess;
}
