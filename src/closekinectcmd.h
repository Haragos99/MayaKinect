#pragma once
#pragma once
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include "kincetcontroller.h"
#include <maya/MArgList.h>




class CloseKinectCmd : public MPxCommand {
public:
    static void* creator() { return new CloseKinectCmd; }
    MStatus doIt(const MArgList& args) override;
};