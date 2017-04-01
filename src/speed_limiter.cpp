/**
 * @author: Enrique Fernández
 * @author Gérald Lelong
 */

#include <algorithm>
#include <math.h>
#include <ackermann_controller/speed_limiter.h>

template<typename T>
T clamp(T x, T min, T max)
{
    return std::min(std::max(min, x), max);
}

namespace ackermann_controller
{

SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits,
    bool has_acceleration_limits,
    bool has_deceleration_limits,
    bool has_jerk_limits,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_deceleration,
    double max_deceleration,
    double min_jerk,
    double max_jerk
)
    : has_velocity_limits(has_velocity_limits)
    , has_acceleration_limits(has_acceleration_limits)
    , has_deceleration_limits(has_deceleration_limits)
    , has_jerk_limits(has_jerk_limits)
    , min_velocity(min_velocity)
    , max_velocity(max_velocity)
    , min_acceleration(min_acceleration)
    , max_acceleration(max_acceleration)
    , min_deceleration(min_deceleration)
    , max_deceleration(max_deceleration)
    , min_jerk(min_jerk)
    , max_jerk(max_jerk)
{
}

double SpeedLimiter::limit(double& v, double v0, double v1, double dt)
{
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_velocity(double& v)
{
    const double tmp = v;

    if (has_velocity_limits)
    {
        v = clamp(v, min_velocity, max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
{
    const double tmp = v;

    if (fabs(v) > fabs(v0))
    {
        if (has_acceleration_limits)
        {
            const double dv_min = min_acceleration * dt;
            const double dv_max = max_acceleration * dt;

            const double dv = clamp(v - v0, dv_min, dv_max);

            v = v0 + dv;
        }
    }
    else
    {
        if (has_deceleration_limits)
        {
            const double dv_min = min_deceleration * dt;
            const double dv_max = max_deceleration * dt;

            const double dv = clamp(v - v0, dv_min, dv_max);

            v = v0 + dv;
        }
    }

    return tmp != 0.0 ? v / tmp : 1.0;
}

double SpeedLimiter::limit_jerk(double& v, double v0, double v1, double dt)
{
    const double tmp = v;

    if (has_jerk_limits)
    {
        const double dv  = v  - v0;
        const double dv0 = v0 - v1;

        const double dt2 = 2. * dt * dt;

        const double da_min = min_jerk * dt2;
        const double da_max = max_jerk * dt2;

        const double da = clamp(dv - dv0, da_min, da_max);

        v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
}
}
