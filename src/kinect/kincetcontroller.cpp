#include "kincetcontroller.h"
#include <maya/MMatrix.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>

KinectController::KinectController()
{
	//skeleton.createSkeleton();
	MGlobal::displayInfo("KinectController initialized");

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
	createSkeleton();
	
	return MS::kSuccess;
}


void KinectController::createSkeleton()
{
	if (!skeleton.skeletonExists())
	{
		MGlobal::displayInfo("Kinect skeleton not found in scene. Creating new skeleton...");
		skeleton.createSkeleton();
	}
}

std::map<std::string, MObject>& KinectController::getSkeletonMap()
{
	return skeleton.getJointMap();
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
	kinect.processSkeleton();

	auto& joints = kinect.getLatestJoints();
	if (joints.size() != NUI_SKELETON_POSITION_COUNT)
	{
		if (++lostFrames == 15)
			for (auto& filter : rotFilters) filter.reset();
		return;
	}
	lostFrames = 0;

	// turn on only after the unfiltered pose matches the green skeleton
	constexpr bool useRotationFilter = false;
	const double dt = std::clamp(kinect.getFrameDt(), 1.0 / 120.0, 0.1);
	auto& kinectToMayaMap = kinect.getKinectToMayaMap();
	auto& jointMap = skeleton.getJointMap();

	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
	{
		auto nameIt = kinectToMayaMap.find(i);
		if (nameIt == kinectToMayaMap.end())
		{
			continue;
		}
		const std::string& jointName = nameIt->second;


		// Skip foot joints to prevent unnatural leg twisting
		if (jointName == "AnkleRight" || jointName == "AnkleLeft")
		{
			continue;
		}

		auto mayaIt = jointMap.find(jointName);
		if (mayaIt == jointMap.end())
		{
			continue;
		}
		MFnTransform transform(mayaIt->second);

		MQuaternion q = kinect.getLocalQuat(i);
		q.normalizeIt();

		if (useRotationFilter)
		{
			RotFilterConfig config = torsoCfg;
			if (jointName.find("Shoulder") != std::string::npos ||
				jointName.find("Elbow") != std::string::npos ||
				jointName.find("Wrist") != std::string::npos ||
				jointName.find("Hand") != std::string::npos)
			{
				config = armCfg;
			}
			else if (jointName.find("Hip") != std::string::npos ||
				jointName.find("Knee") != std::string::npos ||
				jointName.find("Ankle") != std::string::npos ||
				jointName.find("Foot") != std::string::npos)
			{
				config = legCfg;
			}
			if (joints[i].trackingState != NUI_SKELETON_POSITION_TRACKED)
			{
				config.minCutoffHz *= 0.5;
			}

			q = rotFilters[i].update(q, dt, config);
		}

		if (i == NUI_SKELETON_POSITION_HIP_CENTER)
		{
			transform.setTranslation(joints[i].position * 100.0, MSpace::kTransform);
		}
		transform.setRotation(q, MSpace::kTransform);
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