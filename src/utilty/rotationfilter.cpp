#include "rotationfilter.h" 

double JointRotFilter::alpha(double cutoffHz, double dt)
{
    const double tau = 1.0 / (2.0 * 3.14159265358979323846 * cutoffHz);
    return 1.0 / (1.0 + tau / dt);
}

MQuaternion JointRotFilter::update(const MQuaternion& rawInput, double dt,
    const RotFilterConfig& cfg)
{
    MQuaternion raw = rawInput;
    raw.normalizeIt();
    if (!initialized)
    {
        prev = raw;
        initialized = true;
        return prev;
    }

    dt = (std::max)(dt, 1e-3);
    double dot = std::abs(prev.x * raw.x + prev.y * raw.y +
        prev.z * raw.z + prev.w * raw.w);
    double angle = 2.0 * std::acos((std::min)(dot, 1.0));

    speed += alpha(cfg.dCutoffHz, dt) * (angle / dt - speed);
    const double cutoff = cfg.minCutoffHz + cfg.beta * speed;
    prev = slerp(prev, raw, alpha(cutoff, dt));
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
