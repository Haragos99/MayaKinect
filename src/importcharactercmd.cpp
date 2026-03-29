#include "importcharactercmd.h"

MStatus ImportCharacterCmd::doIt(const MArgList& args)
{

    if (args.length() != 1) {
        MGlobal::displayError("Expected 1 argument: path as a string.");
        return MS::kFailure;
    }

    MStatus status;
    MString modelPath = args.asString(0, &status);
    if (status != MS::kSuccess || modelPath.length() == 0) {
        MGlobal::displayError("ImportCharacterCmd: Invalid path argument.");
        return MS::kFailure;
    }

    auto& controller = KinectController::instance();
    controller.createSkeleton();
	jointMap = controller.getSkeletonMap();

    // Step 1 — import the mesh
    MStringArray meshTransform;
    if (!importModel(modelPath, meshTransform))
    {
        return MS::kFailure;
    }

    // Step 2 — bind mesh to skeleton
    MString skinCluster;
    if (!bindToSkeleton(meshTransform))
    {
        return MS::kFailure;
    }
    MGlobal::displayInfo("importCharacter: Done. SkinCluster: " + skinCluster);

    return MS::kSuccess;
}



// Import the file and return the first mesh transform found
bool ImportCharacterCmd::importModel(MString& modelPath, MStringArray& outMeshTransforms)
{
    MGlobal::executeCommand("select -clear;");

    // Import and get created nodes
    MStringArray newNodes;
    MString cmd =
        "file -import -type \"OBJ\" -returnNewNodes \"" + modelPath + "\";";

    if (MGlobal::executeCommand(cmd, newNodes) != MS::kSuccess)
    {
        MGlobal::displayError("Import failed.");
        return false;
    }

    // From imported nodes  keep only mesh transforms
    for (unsigned int i = 0; i < newNodes.length(); ++i)
    {
        MString node = newNodes[i];

        // Check if it's a mesh shape
        MString checkCmd = "nodeType \"" + node + "\";";
        MString type;
        MGlobal::executeCommand(checkCmd, type);

        if (type == "mesh")
        {
            // Get parent transform
            MString parentCmd =
                "listRelatives -parent -fullPath \"" + node + "\";";

            MStringArray parents;
            MGlobal::executeCommand(parentCmd, parents);

            if (parents.length() > 0)
            {
                outMeshTransforms.append(parents[0]);
            }
        }
    }

    if (outMeshTransforms.length() == 0)
    {
        MGlobal::displayError("No meshes found in imported file.");
        return false;
    }

    return true;
}



bool ImportCharacterCmd::bindToSkeleton(const MStringArray& meshes)
{

    MStringArray joints;
    MGlobal::executeCommand("ls -type joint;", joints);

    if (joints.length() == 0)
    {
        MGlobal::displayError("No joints found.");
        return false;
    }

    for (unsigned int i = 0; i < meshes.length(); ++i)
    {
        MString cmd = "skinCluster ";

        //  FLAGS FIRST
        cmd += "-toSelectedBones ";
        cmd += "-maximumInfluences 4 ";
        cmd += "-normalizeWeights 1 ";

        //  THEN JOINTS
        for (unsigned int j = 0; j < joints.length(); ++j)
        {
            cmd += "\"" + joints[j] + "\" ";
        }

        //  THEN MESH
        cmd += "\"" + meshes[i] + "\";";

        if (MGlobal::executeCommand(cmd) != MS::kSuccess)
        {
            MGlobal::displayError("Binding failed for mesh: " + meshes[i]);
            return false;
        }
    }

    return true;
}