#include "takerecorder.h"

void TakeRecorder::start(int newStartFrame, double newFps)
{
    samples.clear();
    // About 10 minutes at 30 fps, so the vector doesn't reallocate during a normal take
    samples.reserve(30 * 60 * 10);
    startFrame = newStartFrame;
    fps = newFps > 0.0 ? newFps : 30.0;
    recording = true;
}

void TakeRecorder::stop()
{
    recording = false;
}

void TakeRecorder::clear()
{
    recording = false;
    samples.clear();
}

void TakeRecorder::addSample(const FrameSample& sample)
{
    if (!recording || sample.empty())
    {
        return;
    }

    // Same Kinect frame as before: the timer ticked without new data
    if (!samples.empty() && samples.back().timestampMs == sample.timestampMs)
    {
        return;
    }

    samples.push_back(sample);
}

double TakeRecorder::durationSeconds() const
{
    if (samples.size() < 2)
    {
        return 0.0;
    }
    return (samples.back().timestampMs - samples.front().timestampMs) / 1000.0;
}
