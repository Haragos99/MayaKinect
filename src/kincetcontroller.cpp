#include "kincetcontroller.h"
#include <maya/MMatrix.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>

KinectController::KinectController()
{
	//skeleton.createSkeleton();

}

MStatus KinectController::initKinect()
{
	auto status = kinect.initialize();
	
	if (FAILED(status))
	{
		MGlobal::displayError("No Kinect connected");
		return MS::kFailure;
	}

	MGlobal::displayInfo("Kinect Connected ");
	skeleton.createSkeleton();
	return MS::kSuccess;
}

std::vector<JointData>& KinectController::getSkeleton()
{
	kinect.processSkeleton();
	return kinect.getLatestJoints();
}

bool KinectController::getImageBuffer(unsigned char* outbuffer)
{
	return kinect.getFrame(outbuffer);
}


void KinectController::stop()
{
	kinect.shutdownKinect();
}


void KinectController::updateMayaSkeleton()
{
	auto& joints = getSkeleton();
	auto& kinectToMayaMap = kinect.getKinectToMayaMap();
	auto& jointMap = skeleton.getJointMap();
	auto& offsets = kinect.getOffsets();
	auto parentMap = kinect.parent_joint_map;
	std::vector<MVector> globalPos(joints.size());
	std::vector<MQuaternion> globalRot(joints.size());

	for (int i = 0; i < joints.size(); ++i)
	{
		// Find the name for this Kinect index
		auto itName = kinectToMayaMap.find(i);
		if (itName == kinectToMayaMap.end())
		{
			continue;
		}

		auto itMaya = jointMap.find(itName->second);
		if (itMaya == jointMap.end()) 
		{
			continue;
		}
		MObject mayaJoint = itMaya->second;
		MFnIkJoint fnJoint(mayaJoint);
		MFnTransform fnTrans(mayaJoint);

		MQuaternion jointOrient;
		fnJoint.getOrientation(jointOrient);

		int parent = parentMap[i];

		MEulerRotation localEuler = kinect.getEulers(i);
		MQuaternion localRot = localEuler.asQuaternion();

		if (itName->second == "SpineBase")
		{
			MVector pos = joints[i].position * 100;
			fnTrans.setTranslation(pos, MSpace::kTransform);
		}
		else
		{
			fnTrans.setRotation(localRot, MSpace::kTransform);
		}
	}
}

void KinectController::createDebugSpheres()
{
	debugSpheres.clear();

	// Joint names for labeling — matches NUI_SKELETON_POSITION order
	const char* jointNames[] = {
		"dbg_HipCenter",     "dbg_Spine",          "dbg_ShoulderCenter",
		"dbg_Head",          "dbg_ShoulderLeft",    "dbg_ElbowLeft",
		"dbg_WristLeft",     "dbg_HandLeft",        "dbg_ShoulderRight",
		"dbg_ElbowRight",    "dbg_WristRight",      "dbg_HandRight",
		"dbg_HipLeft",       "dbg_KneeLeft",        "dbg_AnkleLeft",
		"dbg_FootLeft",      "dbg_HipRight",        "dbg_KneeRight",
		"dbg_AnkleRight",    "dbg_FootRight"
	};

	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
	{
		// Create sphere via MEL command
		MString cmd;
		cmd += "sphere -r 2 -n \"";   // radius 2cm
		cmd += jointNames[i];
		cmd += "\";";

		MStringArray result;
		MGlobal::executeCommand(cmd, result);

		if (result.length() == 0) continue;

		// Get the transform node MObject so we can move it fast later
		MString transformName = result[0];
		MSelectionList sel;
		sel.add(transformName);
		MObject obj;
		sel.getDependNode(0, obj);
		debugSpheres.append(obj);

		// Color the sphere based on body region
		MString colorCmd;
		if (i <= 3)  // spine/head — white
			colorCmd = "setAttr \"" + transformName + ".overrideEnabled\" 1; setAttr \"" + transformName + ".overrideColor\" 16;";
		else if (i <= 7)  // left arm — blue
			colorCmd = "setAttr \"" + transformName + ".overrideEnabled\" 1; setAttr \"" + transformName + ".overrideColor\" 6;";
		else if (i <= 11) // right arm — red
			colorCmd = "setAttr \"" + transformName + ".overrideEnabled\" 1; setAttr \"" + transformName + ".overrideColor\" 13;";
		else              // legs — green
			colorCmd = "setAttr \"" + transformName + ".overrideEnabled\" 1; setAttr \"" + transformName + ".overrideColor\" 14;";

		MGlobal::executeCommand(colorCmd);
	}

	// Group all spheres together for cleanliness
	MGlobal::executeCommand(
		"group -n \"KinectDebugSpheres\" dbg_HipCenter dbg_Spine dbg_ShoulderCenter "
		"dbg_Head dbg_ShoulderLeft dbg_ElbowLeft dbg_WristLeft dbg_HandLeft "
		"dbg_ShoulderRight dbg_ElbowRight dbg_WristRight dbg_HandRight "
		"dbg_HipLeft dbg_KneeLeft dbg_AnkleLeft dbg_FootLeft "
		"dbg_HipRight dbg_KneeRight dbg_AnkleRight dbg_FootRight;"
	);


	MGlobal::displayInfo("KinectDebug: Created " + MString() + NUI_SKELETON_POSITION_COUNT + " debug spheres.");
}

void KinectController::updateDebugSpheres()
{
	auto& joints = getSkeleton();
	auto& offsets = kinect.getOffsets();
	auto parentMap = kinect.parent_joint_map;
	std::vector<MVector> globalPos(joints.size());
	std::vector<MQuaternion> globalRot(joints.size());



	for (int i = 0; i < joints.size() && i < debugSpheres.length(); ++i)
	{
		int parent = parentMap[i];

		MEulerRotation localEuler = kinect.getEulers(i);

		MQuaternion localRot = localEuler.asQuaternion();

		if (parent == i) // root
		{
			globalPos[i] = offsets[i];
			globalRot[i] = localRot;
		}
		else
		{
			globalRot[i] = globalRot[parent] * localRot;
			globalPos[i] = globalPos[parent] + offsets[i].rotateBy(globalRot[parent]);
		}


		MFnTransform fnTrans(debugSpheres[i]);
		fnTrans.setTranslation(globalPos[i], MSpace::kTransform);
		fnTrans.setRotation(globalRot[i], MSpace::kTransform);



		// Position already has Z flipped and is in meters
		// Multiply by 100 to convert to Maya cm
		MVector pos(
			joints[i].position.x * 100.0,
			joints[i].position.y * 100.0,
			joints[i].position.z * 100.0
		);

		//fnTrans.setTranslation(pos, MSpace::kTransform);
	}
}


void KinectController::applyRotation(const MString& jointName, const MQuaternion& quat)
{
	MSelectionList sel;
	sel.add(jointName);
	MDagPath dagPath;
	if (sel.getDagPath(0, dagPath) != MS::kSuccess)
		return;

	MFnIkJoint joint(dagPath);

	// No flip here — already done in creatSkeleton
	MQuaternion worldQ = quat;
	worldQ.normalizeIt();

	// STEP 1: Remove parent world rotation to get local space
	MQuaternion parentWorldQ = MQuaternion::identity;
	if (dagPath.length() > 1)
	{
		MDagPath parentPath = dagPath;
		parentPath.pop();
		MFnTransform parentFn(parentPath);
		parentFn.getRotation(parentWorldQ, MSpace::kWorld);
		parentWorldQ.normalizeIt();
	}

	MQuaternion jointOrient;
	joint.getOrientation(jointOrient);
	jointOrient.normalizeIt();

	MQuaternion finalQ = jointOrient.inverse() * worldQ;
	finalQ.normalizeIt();

	joint.setRotation(finalQ, MSpace::kTransform);

}

void KinectController::start()
{
}


void KinectController::applyRotation(const MString& jointName, const MMatrix& rotMatrix)
{
	MSelectionList sel;
	sel.add(jointName);
	MDagPath dagPath;
	if (sel.getDagPath(0, dagPath) != MS::kSuccess)
		return;

	MFnIkJoint joint(dagPath);


	MQuaternion jointOrientQ;
	joint.getOrientation(jointOrientQ);
	MMatrix jointOrientMatrix = jointOrientQ.asMatrix();

	MMatrix finalMatrix = jointOrientMatrix.inverse() * rotMatrix;


	MTransformationMatrix tm(finalMatrix);
	MQuaternion finalQ = tm.rotation();
	finalQ.normalizeIt();

	joint.setRotation(finalQ, MSpace::kTransform);
}


KinectController& KinectController::instance()
{
	static KinectController instance;
	return instance;
}