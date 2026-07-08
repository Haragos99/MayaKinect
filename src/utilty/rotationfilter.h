#pragma once
#include <maya/MQuaternion.h>
#include <array>
#include <algorithm>
#include <cmath>

struct RotFilterConfig
{
    double deadzoneDeg = 0.8;      // kis rezgések kivágása
    double maxDegPerSec = 220.0;   // hirtelen ugrás limit
    double alpha = 0.14;           // low-pass erősség (0..1)
};

class JointRotFilter
{
public:
    JointRotFilter() = default;

    void reset();

    MQuaternion update(const MQuaternion& rawInput, double dt, const RotFilterConfig& cfg);

private:
    bool initialized = false;
    MQuaternion prev = MQuaternion::identity;

    static MQuaternion slerp(const MQuaternion& q1, const MQuaternion& q2, double t);
};