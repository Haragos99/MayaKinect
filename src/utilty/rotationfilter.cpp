#include "rotationfilter.h" 

void JointRotFilter::reset()
{
    initialized = false;
    prev = MQuaternion::identity;
}

MQuaternion JointRotFilter::update(const MQuaternion& rawInput, double dt, const RotFilterConfig& cfg)
{
    MQuaternion raw = rawInput;
    raw.normalizeIt();

    if (!initialized)
    {
        prev = raw;
        initialized = true;
        return prev;
    }

    MQuaternion a = prev;
    MQuaternion b = raw;

    // shortest-arc: if dot<0, invert b
    double d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0)
    {
        b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
        d = -d;
    }
    d = std::clamp(d, -1.0, 1.0);

    // deegre 
    double angleDeg = 2.0 * std::acos(std::abs(d)) * 180.0 / 3.14159265358979323846;

    // deadzone
    if (angleDeg < cfg.deadzoneDeg)
        return prev;

    // rate limit
    double safeDt = std::max(dt, 1e-4);
    double maxStep = cfg.maxDegPerSec * safeDt;
    double tClamp = (angleDeg > maxStep) ? (maxStep / angleDeg) : 1.0;
    tClamp = std::clamp(tClamp, 0.0, 1.0);

    MQuaternion limited = slerp(a, b, tClamp);

    // low-pass
    double alpha = std::clamp(cfg.alpha, 0.0, 1.0);
    MQuaternion out = slerp(a, limited, alpha);
    out.normalizeIt();

    prev = out;
    return prev;
}

MQuaternion JointRotFilter::slerp(const MQuaternion& q1, const MQuaternion& q2, double t)
{
    t = std::clamp(t, 0.0, 1.0);

    MQuaternion a = q1; 
    a.normalizeIt();

    MQuaternion b = q2; 
    b.normalizeIt();

    double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

    if (dot < 0.0)
    {
        b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD)
    {
        MQuaternion r(
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z),
            a.w + t * (b.w - a.w)
        );
        r.normalizeIt();
        return r;
    }

    dot = std::clamp(dot, -1.0, 1.0);
    double theta0 = std::acos(dot);
    double theta = theta0 * t;
    double sinTheta = std::sin(theta);
    double sinTheta0 = std::sin(theta0);

    double s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    double s1 = sinTheta / sinTheta0;

    MQuaternion r(
        s0 * a.x + s1 * b.x,
        s0 * a.y + s1 * b.y,
        s0 * a.z + s1 * b.z,
        s0 * a.w + s1 * b.w
    );
    r.normalizeIt();
    return r;
}