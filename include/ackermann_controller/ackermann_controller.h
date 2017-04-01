/**
 * @author Enrique Fernández
 * @author Gérald Lelong
 */

#ifndef ACKERMANN_CONTROLLER_H
#define ACKERMANN_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <ackermann_controller/odometry.h>
#include <ackermann_controller/speed_limiter.h>
#include <ackermann_controller/joint.h>

namespace ackermann_controller {

class AckermannController: public controller_interface::MultiInterfaceController<
    hardware_interface::VelocityJointInterface,
    hardware_interface::JointStateInterface,
    hardware_interface::PositionJointInterface>
{
public:

    AckermannController();

    /**
    * \brief Initialize controller
    * \param hw            Joint interface
    * \param root_nh       Node handle at root namespace
    * \param controller_nh Node handle inside the controller namespace
    */
    bool init(
        hardware_interface::RobotHW* hw,
        ros::NodeHandle& root_nh,
        ros::NodeHandle &controller_nh);

    /**
    * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
    * \param time   Current time
    * \param period Time since the last called to update
    */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
    * \brief Starts controller
    * \param time Current time
    */
    void starting(const ros::Time& time);

    /**
    * \brief Stops controller
    * \param time Current time
    */
    void stopping(const ros::Time& /*time*/);

private:

    std::string name_;

    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    std::vector<ActuatedWheel> spinning_joints_;
    std::vector<Wheel> odometry_joints_;
    std::vector<ActuatedJoint> steering_joints_;

    struct Commands
    {
        double lin;
        double ang;
        ros::Time stamp;

        Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };

    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    ackermann_controller::Odometry odometry_;

    bool steering_angle_instead_of_angular_speed_;
    double wheelbase_;
    double cmd_vel_timeout_;
    int velocity_rolling_window_size_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string base_link_;
    bool enable_odom_tf_;

    Commands last1_cmd_;
    Commands last0_cmd_;
    ackermann_controller::SpeedLimiter limiter_;


private:

    void brake();

    void cmdVelCallback(const geometry_msgs::Twist& command);

    bool initParams(ros::NodeHandle& controller_nh);

    void updateOdometry(const ros::Time& time, const ros::Duration& period);

    void moveRobot(const ros::Time& time, const ros::Duration& period);

    boost::shared_ptr<urdf::ModelInterface> getURDFModel(const ros::NodeHandle& nh);

    std::vector<std::string> getJointNames(ros::NodeHandle& controller_nh, const std::string& param);

    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

};
}

#endif
