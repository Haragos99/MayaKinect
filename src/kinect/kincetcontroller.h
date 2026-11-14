#pragma once
#include "kinectdevice.h"
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MObjectArray.h>
#include <maya/MGlobal.h>
#include "../skeleton/skeletonbuilder.h"
#include "../utilty/rotationfilter.h"
#include "../recording/takerecorder.h"
#include <chrono>

class KinectController
{
public:

	static KinectController& instance();

	KinectController();

	void start();

	MStatus initKinect();

	void updateMayaSkeleton();

	std::vector<JointData>& getSkeleton();

	bool getImageBuffer(unsigned char* outbuffer);

	void stop();
	bool isRotationFilterEnabled() const;
	void setRotationFilterEnabled(bool enabled);
	void createSkeleton();
	MStringArray getSkeletonJointNames() const;
	bool isDriveEnabled() const;
	void setDriveEnabled(bool enabled);
	TakeRecorder& getRecorder() { return recorder; }
	void createDebugSpheres();
	void updateDebugSpheres();
	std::map<std::string, MObject>& getSkeletonMap();
	void applyRotation(const MString& jointName, const MQuaternion& quat);
	void applyRotation(const MString& jointName, const MMatrix& rotMatrix);

private:
	SkeletonBuilder skeleton; // responsable for the maya skeleton
	KinectDevice kinect; // responble the kinnect
	MObjectArray debugSpheres;
	int lostFrames = 0;
	bool rotationFilterEnabled = true;
	bool driveEnabled = true; // false: updateMayaSkeleton() does not touch the joints
	TakeRecorder recorder;    // collects the applied joint data while recording
	std::array<JointRotFilter, NUI_SKELETON_POSITION_COUNT> rotFilters{};
	std::chrono::steady_clock::time_point lastUpdateTs{};
	bool hasLastUpdateTs = false;

	RotFilterConfig torsoCfg{ 0.8, 0.3, 1.0 };
	RotFilterConfig armCfg{ 1.2, 0.6, 1.0 };
	RotFilterConfig legCfg{ 1.0, 0.5, 1.0 };
};