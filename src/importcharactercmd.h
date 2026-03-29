#pragma once
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include "kincetcontroller.h"
#include <maya/MArgList.h>
#include <maya/MString.h>
class ImportCharacterCmd : public MPxCommand {

public:
    ImportCharacterCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new ImportCharacterCmd; }

private:
    bool importModel( MString& modelPath, MStringArray& outMeshTransforms);
    bool bindToSkeleton(const MStringArray& meshTransforms);
    std::map<std::string, MObject> jointMap;

};

