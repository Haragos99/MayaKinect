#pragma once
#include <maya/MQuaternion.h>
#include <maya/MVector.h>
#include <maya/MObject.h>
#include <maya/MString.h>
#include <array>
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

// Number of Kinect v1 skeleton joints (NUI_SKELETON_POSITION_COUNT) and the index of the
// root joint (NUI_SKELETON_POSITION_HIP_CENTER). Checked with static_asserts in
// kincetcontroller.cpp, so this header doesn't need NuiApi.h.
constexpr int kRecorderJointCount = 20;
constexpr int kRecorderRootIndex = 0;

// One Kinect frame as it was applied to the Mocap_* skeleton.
struct FrameSample
{
    int64_t timestampMs = 0;                                    // Kinect frame time
    MVector rootTranslation;                                    // SpineBase translation (cm)
    bool hasRoot = false;
    std::array<MQuaternion, kRecorderJointCount> rotations{};   // local rotation per Kinect joint index
    std::bitset<kRecorderJointCount> valid;                     // which joints were set in this frame

    bool empty() const { return valid.none() && !hasRoot; }
};

// Collects FrameSamples in memory while recording. Writing them as keyframes
// happens once at the end, so recording doesn't slow down the live view
class TakeRecorder
{
public:
    // Clears the buffer and starts collecting. startFrame/fps are used when writing the keys.
    void start(int startFrame, double fps);
    // Stops collecting; the samples stay in the buffer until the next start() or clear().
    void stop();
    void clear();

    bool isRecording() const { return recording; }

    // Adds a sample while recording. Ignored when not recording, when empty, or when it has
    // the same timestamp as the previous sample (no new Kinect frame).
    void addSample(const FrameSample& sample);

    size_t sampleCount() const { return samples.size(); }
    const std::vector<FrameSample>& getSamples() const { return samples; }
    int getStartFrame() const { return startFrame; }
    double getFps() const { return fps; }
    // Recorded duration in seconds (first to last sample).
    double durationSeconds() const;

    // Writes the buffer as animation curves on the skeleton joints.
    // Returns the number of frames written (0 when the buffer is empty), or -1 on error.
    int writeCurves(const std::map<std::string, MObject>& jointMap,
                    const std::map<int, std::string>& kinectToMaya,
                    MString* outError = nullptr) const;

    // Frame each sample is written to (same rule as writeCurves). Exposed for testing.
    int frameForSample(size_t index) const;

private:
    std::vector<FrameSample> samples;
    bool recording = false;
    int startFrame = 1;
    double fps = 30.0;
};
