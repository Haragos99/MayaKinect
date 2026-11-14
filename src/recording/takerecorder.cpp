#include "takerecorder.h"
#include <maya/MDGModifier.h>
#include <maya/MDoubleArray.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnTransform.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MTime.h>
#include <maya/MTimeArray.h>
#include <maya/MTransformationMatrix.h>
#include <cmath>

namespace
{
    void setError(MString* outError, const MString& message)
    {
        if (outError)
        {
            *outError = message;
        }
    }

    // Queues deletion of the anim curves driving these plugs, so a new take never mixes
    // with the keys of the previous one (also removes keys past the new end).
    void queueRemoveAnimation(const MPlug& plug, MDGModifier& modifier)
    {
        MPlugArray sources;
        if (!plug.connectedTo(sources, true, false))
        {
            return;
        }
        for (unsigned int i = 0; i < sources.length(); ++i)
        {
            MObject node = sources[i].node();
            if (node.hasFn(MFn::kAnimCurve))
            {
                modifier.deleteNode(node);
            }
        }
    }

    // A plug that is driven by something other than an anim curve (e.g. a constraint)
    // can't get keys.
    bool isDrivenByNonCurve(const MPlug& plug)
    {
        MPlugArray sources;
        if (!plug.connectedTo(sources, true, false))
        {
            return false;
        }
        for (unsigned int i = 0; i < sources.length(); ++i)
        {
            if (!sources[i].node().hasFn(MFn::kAnimCurve))
            {
                return true;
            }
        }
        return false;
    }

    bool writeCurve(const MPlug& plug, MFnAnimCurve::AnimCurveType type,
                    MTimeArray& times, MDoubleArray& values, MString* outError)
    {
        MStatus status;
        MFnAnimCurve curve;
        curve.create(plug, type, nullptr, &status);
        if (status != MS::kSuccess)
        {
            setError(outError, "Could not create an anim curve for " + plug.name());
            return false;
        }
        status = curve.addKeys(&times, &values,
                               MFnAnimCurve::kTangentClamped, MFnAnimCurve::kTangentClamped);
        if (status != MS::kSuccess)
        {
            setError(outError, "Could not add keys to " + plug.name());
            return false;
        }
        return true;
    }

    // MTransformationMatrix::RotationOrder starts with kInvalid, MEulerRotation's doesn't.
    MEulerRotation::RotationOrder toEulerOrder(MTransformationMatrix::RotationOrder order)
    {
        if (order <= MTransformationMatrix::kInvalid || order >= MTransformationMatrix::kLast)
        {
            return MEulerRotation::kXYZ;
        }
        return static_cast<MEulerRotation::RotationOrder>(order - MTransformationMatrix::kXYZ);
    }
}

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

int TakeRecorder::frameForSample(size_t index) const
{
    const double seconds = (samples[index].timestampMs - samples.front().timestampMs) / 1000.0;
    return startFrame + static_cast<int>(std::llround(seconds * fps));
}

int TakeRecorder::writeCurves(const std::map<std::string, MObject>& jointMap,
                              const std::map<int, std::string>& kinectToMaya,
                              MString* outError) const
{
    if (samples.empty())
    {
        return 0;
    }

    // 1. Sample -> frame. Several samples on one frame: keep the last one.
    std::vector<std::pair<int, size_t>> frames; // (frame, sample index)
    frames.reserve(samples.size());
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const int frame = frameForSample(i);
        if (!frames.empty() && frames.back().first >= frame)
        {
            frames.back().second = i;
        }
        else
        {
            frames.emplace_back(frame, i);
        }
    }

    // Keys are written in the scene's time unit; the panel sets it to match the fps
    const MTime::Unit timeUnit = MTime::uiUnit();
    MTimeArray allTimes;
    for (const auto& f : frames)
    {
        allTimes.append(MTime(static_cast<double>(f.first), timeUnit));
    }

    // 2. Find the joints that were recorded and check they can be keyed
    struct JointToWrite { int index; MObject node; bool root; };
    std::vector<JointToWrite> toWrite;
    for (int j = 0; j < kRecorderJointCount; ++j)
    {
        bool recorded = false;
        for (const auto& f : frames)
        {
            if (samples[f.second].valid.test(j)) { recorded = true; break; }
        }
        const bool root = (j == kRecorderRootIndex);
        bool rootRecorded = false;
        if (root)
        {
            for (const auto& f : frames)
            {
                if (samples[f.second].hasRoot) { rootRecorded = true; break; }
            }
        }
        if (!recorded && !rootRecorded)
        {
            continue; // e.g. wrists/ankles, which are skipped live
        }

        auto nameIt = kinectToMaya.find(j);
        if (nameIt == kinectToMaya.end())
        {
            continue;
        }
        auto nodeIt = jointMap.find(nameIt->second);
        if (nodeIt == jointMap.end() || nodeIt->second.isNull())
        {
            setError(outError, MString("Joint not found in the scene: ") + nameIt->second.c_str());
            return -1;
        }
        toWrite.push_back({ j, nodeIt->second, rootRecorded });
    }

    static const char* kRotateAttrs[] = { "rotateX", "rotateY", "rotateZ" };
    static const char* kTranslateAttrs[] = { "translateX", "translateY", "translateZ" };

    // 3. Remove the previous take's curves, refuse plugs driven by something else
    MDGModifier removeOld;
    for (const auto& joint : toWrite)
    {
        MFnDependencyNode fn(joint.node);
        for (const char* attr : kRotateAttrs)
        {
            MPlug plug = fn.findPlug(attr, true);
            if (isDrivenByNonCurve(plug))
            {
                setError(outError, plug.name() + " is driven by another node (constraint?), can't write keys");
                return -1;
            }
            queueRemoveAnimation(plug, removeOld);
        }
        if (joint.root)
        {
            for (const char* attr : kTranslateAttrs)
            {
                MPlug plug = fn.findPlug(attr, true);
                if (isDrivenByNonCurve(plug))
                {
                    setError(outError, plug.name() + " is driven by another node (constraint?), can't write keys");
                    return -1;
                }
                queueRemoveAnimation(plug, removeOld);
            }
        }
    }
    removeOld.doIt();

    // 4. Write the new curves
    for (const auto& joint : toWrite)
    {
        MFnTransform transform(joint.node);
        MFnDependencyNode fn(joint.node);
        const MEulerRotation::RotationOrder order = toEulerOrder(transform.rotationOrder());

        // Rotation
        MTimeArray times;
        MDoubleArray values[3];
        bool hasPrevious = false;
        MEulerRotation previous;
        for (unsigned int k = 0; k < frames.size(); ++k)
        {
            const FrameSample& sample = samples[frames[k].second];
            if (!sample.valid.test(joint.index))
            {
                continue;
            }

            MEulerRotation euler = sample.rotations[joint.index].asEulerRotation();
            euler.reorderIt(order);
            if (hasPrevious)
            {
                // Same orientation, but the angles closest to the previous key: no flips
                euler.setToClosestSolution(previous);
            }
            previous = euler;
            hasPrevious = true;

            times.append(allTimes[k]);
            values[0].append(euler.x);   // radians (Maya's internal angle unit)
            values[1].append(euler.y);
            values[2].append(euler.z);
        }
        if (times.length() > 0)
        {
            for (int a = 0; a < 3; ++a)
            {
                MPlug plug = fn.findPlug(kRotateAttrs[a], true);
                if (!writeCurve(plug, MFnAnimCurve::kAnimCurveTA, times, values[a], outError))
                {
                    return -1;
                }
            }
        }

        // Root translation
        if (joint.root)
        {
            MTimeArray rootTimes;
            MDoubleArray rootValues[3];
            for (unsigned int k = 0; k < frames.size(); ++k)
            {
                const FrameSample& sample = samples[frames[k].second];
                if (!sample.hasRoot)
                {
                    continue;
                }
                rootTimes.append(allTimes[k]);
                rootValues[0].append(sample.rootTranslation.x);   // cm (Maya's internal unit)
                rootValues[1].append(sample.rootTranslation.y);
                rootValues[2].append(sample.rootTranslation.z);
            }
            for (int a = 0; a < 3 && rootTimes.length() > 0; ++a)
            {
                MPlug plug = fn.findPlug(kTranslateAttrs[a], true);
                if (!writeCurve(plug, MFnAnimCurve::kAnimCurveTL, rootTimes, rootValues[a], outError))
                {
                    return -1;
                }
            }
        }
    }

    return frames.back().first - startFrame + 1;
}
