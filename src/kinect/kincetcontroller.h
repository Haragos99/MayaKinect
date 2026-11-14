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
	bool isStreaming() const { return streaming; }
	// Starts collecting frames. Fails if the stream isn't running or a recording is already running.
	MStatus startRecording(int startFrame, double fps, MString* outError = nullptr);
	// Stops collecting and writes the take as keyframes on the Mocap_* joints.
	// Returns the number of frames written, or -1 on error.
	int stopRecording(MString* outError = nullptr);
	bool isRecording() const { return recorder.isRecording(); }
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
	bool streaming = false;   // true between a successful initKinect() and stop()
	std::array<JointRotFilter, NUI_SKELETON_POSITION_COUNT> rotFilters{};
	std::chrono::steady_clock::time_point lastUpdateTs{};
	bool hasLastUpdateTs = false;

	RotFilterConfig torsoCfg{ 0.8, 0.3, 1.0 };
	RotFilterConfig armCfg{ 1.2, 0.6, 1.0 };
	RotFilterConfig legCfg{ 1.0, 0.5, 1.0 };
};