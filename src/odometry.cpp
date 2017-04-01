/**
 * @author Luca Marchionni
 * @author Bence Magyar
 * @author Enrique Fernández
 * @author Paul Mathieu
 * @author Gérald Lelong
 */

#include <ackermann_controller/odometry.h>
#include <boost/bind.hpp>

namespace ackermann_controller
{

namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
    : timestamp_(0.0)
    , x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_(0.0)
    , angular_(0.0)
    , wheelbase_(1.0)
    , velocity_rolling_window_size_(velocity_rolling_window_size)
    , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2))
{
}

void Odometry::init(const ros::Time& time)
{
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
}

bool Odometry::update(
    const std::vector<ActuatedJoint>& steering_joints,
    const std::vector<Wheel>& odometry_joints,
    const ros::Time &time)
{
    double linear_sum = 0.0;
    double angular_sum = 0.0;
    double steering_angle_sum = 0.0;

    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;

    for (std::vector<Wheel>::const_iterator it = odometry_joints.begin(); it != odometry_joints.end(); ++it)
    {
        const double wheel_est_vel = it->handle_.getVelocity() * dt;
        linear_sum += wheel_est_vel * it->radius_;
    }

    const double linear = linear_sum / odometry_joints.size();

    for (std::vector<ActuatedJoint>::const_iterator it = steering_joints.begin(); it != steering_joints.end(); ++it)
    {
        const double steering_angle = it->getPosition();
        double virtual_steering_angle = std::atan(wheelbase_ * std::tan(steering_angle)/std::abs(wheelbase_ + it->lateral_deviation_ * std::tan(steering_angle)));
        steering_angle_sum += virtual_steering_angle;
        angular_sum += linear * tan(virtual_steering_angle) / wheelbase_;
    }

    const double angular = angular_sum / steering_joints.size();
    const double steering_angle = steering_angle_sum / steering_joints.size();

    /// Integrate odometry:
    const double curvature_radius = wheelbase_ / cos(M_PI/2.0 - steering_angle);

    if (fabs(curvature_radius) > 0.0001)
    {
        const double elapsed_distance = linear;
        const double elapsed_angle = elapsed_distance / curvature_radius;
        const double x_curvature = curvature_radius * sin(elapsed_angle);
        const double y_curvature = curvature_radius * (cos(elapsed_angle) - 1.0);
        const double wheel_heading = heading_ + steering_angle;
        y_ += x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
        x_ += x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
        heading_ += elapsed_angle;
    }

    if (dt < 0.0001)
        return false; // Interval too small to integrate with

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const ros::Time &time)
{
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
}

/**
* \brief Other possible integration method provided by the class
* \param linear linear speed
* \param angular angular speed
*/
void Odometry::integrateExact(double linear, double angular)
{
    if (fabs(angular) < 1e-6)
        integrateRungeKutta2(linear, angular);
    else
    {
        /// Exact integration (should solve problems when angular is zero):
        const double heading_old = heading_;
        const double r = linear/angular;
        heading_ += angular;
        x_       +=  r * (sin(heading_) - sin(heading_old));
        y_       += -r * (cos(heading_) - cos(heading_old));
    }
}

void Odometry::resetAccumulators()
{
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}

}
