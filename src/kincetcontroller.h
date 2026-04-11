#pragma once
#include "skeletonbuilder.h"
#include "kinectdevice.h"
#include <maya/MString.h>
#include <maya/MObjectArray.h>
#include <maya/MGlobal.h>
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

};