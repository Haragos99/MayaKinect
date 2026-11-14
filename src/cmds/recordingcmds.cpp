#include "recordingcmds.h"
#include <maya/MArgDatabase.h>
#include <maya/MTime.h>
#include <cmath>

namespace
{
    const char* kStartFrameShort = "-sf";
    const char* kStartFrameLong = "-startFrame";
    const char* kFpsShort = "-fps";
    const char* kFpsLong = "-framesPerSecond";

    // Frames per second of the scene's current time unit (e.g. 30 for ntsc)
    double sceneFps()
    {
        return MTime(1.0, MTime::kSeconds).as(MTime::uiUnit());
    }
}

MSyntax StartKinectRecordingCmd::newSyntax()
{
    MSyntax syntax;
    syntax.addFlag(kStartFrameShort, kStartFrameLong, MSyntax::kLong);
    syntax.addFlag(kFpsShort, kFpsLong, MSyntax::kDouble);
    return syntax;
}

MStatus StartKinectRecordingCmd::doIt(const MArgList& args)
{
    MStatus status;
    MArgDatabase db(syntax(), args, &status);
    if (status != MS::kSuccess)
    {
        MGlobal::displayError("startKinectRecording: invalid arguments. Usage: startKinectRecording [-sf <int>] [-fps <double>]");
        return status;
    }

    int startFrame = 1;
    if (db.isFlagSet(kStartFrameShort))
    {
        db.getFlagArgument(kStartFrameShort, 0, startFrame);
    }

    const double scene = sceneFps();
    double fps = scene;
    if (db.isFlagSet(kFpsShort))
    {
        db.getFlagArgument(kFpsShort, 0, fps);
    }

    // Keys are written in the scene's time unit. A different fps would make playback too fast or slow.
    if (std::abs(fps - scene) > 1e-3)
    {
        MGlobal::displayWarning(MString("startKinectRecording: the scene runs at ") + scene +
            " fps but recording uses " + fps + " fps; playback speed will not match. "
            "Set the scene frame rate first (e.g. currentUnit -time ntsc for 30 fps).");
    }

    MString error;
    auto& controller = KinectController::instance();
    if (controller.startRecording(startFrame, fps, &error) != MS::kSuccess)
    {
        MGlobal::displayError("startKinectRecording: " + error);
        return MS::kFailure;
    }

    MGlobal::displayInfo(MString("Kinect recording started (start frame ") + startFrame + ", " + fps + " fps).");
    return MS::kSuccess;
}


MStatus StopKinectRecordingCmd::doIt(const MArgList& args)
{
    if (args.length() != 0)
    {
        MGlobal::displayError("stopKinectRecording takes no arguments.");
        return MS::kFailure;
    }

    MString error;
    auto& controller = KinectController::instance();
    const double seconds = controller.getRecorder().durationSeconds();
    const int frames = controller.stopRecording(&error);
    if (frames < 0)
    {
        MGlobal::displayError("stopKinectRecording: " + error);
        return MS::kFailure;
    }

    MGlobal::displayInfo(MString("Kinect recording stopped: ") + frames + " frames (" + seconds + " s).");
    setResult(frames);
    return MS::kSuccess;
}


MStatus IsKinectRecordingCmd::doIt(const MArgList& args)
{
    if (args.length() != 0)
    {
        MGlobal::displayError("isKinectRecording takes no arguments.");
        return MS::kFailure;
    }

    setResult(KinectController::instance().isRecording());
    return MS::kSuccess;
}
