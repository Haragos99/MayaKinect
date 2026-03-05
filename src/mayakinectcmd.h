#pragma once
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include "kincetcontroller.h"

class MayaKinectCmd : public MPxCommand {

public:
    MayaKinectCmd() = default;
    MStatus doIt(const MArgList&) override;
    static void* creator() { return new MayaKinectCmd; }

private:


};