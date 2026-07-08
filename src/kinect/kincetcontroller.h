#pragma once
#include "kinectdevice.h"
#include <maya/MString.h>
#include <maya/MObjectArray.h>
#include <maya/MGlobal.h>
#include "../skeleton/skeletonbuilder.h"
#include "../utilty/rotationfilter.h"
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
	void createSkeleton();
	void createDebugSpheres();
	void updateDebugSpheres();
	std::map<std::string, MObject>& getSkeletonMap();
	void applyRotation(const MString& jointName, const MQuaternion& quat);
	void applyRotation(const MString& jointName, const MMatrix& rotMatrix);

private:
	SkeletonBuilder skeleton; // responsable for the maya skeleton
	KinectDevice kinect; // responble the kinnect
	MObjectArray debugSpheres;

	std::array<JointRotFilter, NUI_SKELETON_POSITION_COUNT> rotFilters{};
	std::chrono::steady_clock::time_point lastUpdateTs{};
	bool hasLastUpdateTs = false;

	RotFilterConfig torsoCfg{ 0.4, 300.0, 0.50 };    // Reduced from 0.6
	RotFilterConfig armCfg{ 0.5, 360.0, 0.40 };      // Reduced from 0.8
	RotFilterConfig legCfg{ 0.5, 300.0, 0.45 };      // Reduced from 0.8
};