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

    MQuaternion previusRotaion = prev;
    MQuaternion rawRotaion = raw;

    // shortest-arc: if dot<0, invert b
    double dot = previusRotaion.x * rawRotaion.x + 
                 previusRotaion.y * rawRotaion.y + 
                 previusRotaion.z * rawRotaion.z +
                 previusRotaion.w * rawRotaion.w;

    if (dot < 0.0)
    {
        rawRotaion.x = -rawRotaion.x; 
        rawRotaion.y = -rawRotaion.y;
        rawRotaion.z = -rawRotaion.z;
        rawRotaion.w = -rawRotaion.w;

        dot = -dot;
    }
    dot = std::clamp(dot, -1.0, 1.0);

    // deegre 
    double angleDeg = 2.0 * std::acos(std::abs(dot)) * 180.0 / 3.14159265358979323846;

    // deadzone
    if (angleDeg < cfg.deadzoneDeg)
        return prev;

    // rate limit
    double safeDt = std::max(dt, 1e-4);
    double maxStep = cfg.maxDegPerSec * safeDt;
    double tClamp = (angleDeg > maxStep) ? (maxStep / angleDeg) : 1.0;
    tClamp = std::clamp(tClamp, 0.0, 1.0);

    MQuaternion limited = slerp(previusRotaion, rawRotaion, tClamp);

    // low-pass
    double alpha = std::clamp(cfg.alpha, 0.0, 1.0);
    MQuaternion out = slerp(previusRotaion, limited, alpha);
    out.normalizeIt();

    prev = out;
    return prev;
}

MQuaternion JointRotFilter::slerp(const MQuaternion& q1, const MQuaternion& q2, double t)
{
    MQuaternion prev = q1; 
    prev.normalizeIt();

    MQuaternion limited = q2; 
    limited.normalizeIt();

    double dot = prev.x * limited.x + 
                 prev.y * limited.y +
                 prev.z * limited.z + 
                 prev.w * limited.w;

    if (dot < 0.0)
    {
        limited.x = -limited.x;
        limited.y = -limited.y;
        limited.z = -limited.z;
        limited.w = -limited.w;

        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD)
    {
        MQuaternion rotaion(
            prev.x + t * (limited.x - prev.x),
            prev.y + t * (limited.y - prev.y),
            prev.z + t * (limited.z - prev.z),
            prev.w + t * (limited.w - prev.w)
        );
        rotaion.normalizeIt();
        return rotaion;
    }

    dot = std::clamp(dot, -1.0, 1.0);
    double theta0 = std::acos(dot);
    double theta = theta0 * t;
    double sinTheta = std::sin(theta);
    double sinTheta0 = std::sin(theta0);

    double s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    double s1 = sinTheta / sinTheta0;

    MQuaternion rotaion(
        s0 * prev.x + s1 * limited.x,
        s0 * prev.y + s1 * limited.y,
        s0 * prev.z + s1 * limited.z,
        s0 * prev.w + s1 * limited.w
    );
    rotaion.normalizeIt();
    return rotaion;
}