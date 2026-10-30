#pragma once
#include <maya/MQuaternion.h>
#include <array>
#include <algorithm>
#include <cmath>

struct RotFilterConfig
{
    double minCutoffHz = 1.0;
    double beta = 0.5;
    double dCutoffHz = 1.0;
};

class JointRotFilter
{
public:
    void reset() { initialized = false; speed = 0.0; }
    MQuaternion update(const MQuaternion& rawInput, double dt,
        const RotFilterConfig& cfg);

private:
    bool initialized = false;
    MQuaternion prev = MQuaternion::identity;
    double speed = 0.0;

    static double alpha(double cutoffHz, double dt);
    static MQuaternion slerp(const MQuaternion& q1, const MQuaternion& q2,double t);
};
