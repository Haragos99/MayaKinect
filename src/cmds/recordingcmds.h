#pragma once
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include "../kinect/kincetcontroller.h"

// startKinectRecording [-startFrame/-sf <int>] [-framesPerSecond/-fps <double>]
// Starts recording the live Kinect stream. Defaults: start frame 1, fps = the scene's frame rate.
class StartKinectRecordingCmd : public MPxCommand {

public:
    StartKinectRecordingCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new StartKinectRecordingCmd; }
    static MSyntax newSyntax();
};

// stopKinectRecording
// Stops recording and writes the take as keyframes on the Mocap_* joints.
// Returns the number of frames written.
class StopKinectRecordingCmd : public MPxCommand {

public:
    StopKinectRecordingCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new StopKinectRecordingCmd; }
};

// isKinectRecording
// Returns true while a recording is running.
class IsKinectRecordingCmd : public MPxCommand {

public:
    IsKinectRecordingCmd() = default;
    MStatus doIt(const MArgList& args) override;
    static void* creator() { return new IsKinectRecordingCmd; }
};
