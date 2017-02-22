#include <cmath>

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include "ackerbot_controllers/ackermann_controller.h"

static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
    return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                     std::pow(vec1.y-vec2.y,2) +
                     std::pow(vec1.z-vec2.z,2));
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const boost::shared_ptr<const urdf::Link>& link)
{
    if (!link)
    {
        ROS_ERROR("Link == NULL.");
        return false;
    }

    if (!link->collision)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
        return false;
    }

    if (!link->collision->geometry)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
        return false;
    }

    if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
        return false;
    }

    return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius)
{
    if (!isCylinder(wheel_link))
    {
        ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
        return false;
    }

    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
}

namespace ackerbot_controllers {

AckermannController::AckermannController()
    : open_loop_(false)
    , command_struct_()
    , length_(1.0)
    , wheel_separation_(0.0)
    , wheel_radius_(0.0)
    , wheel_separation_multiplier_(1.0)
    , wheel_radius_multiplier_(1.0)
    , cmd_vel_timeout_(0.5)
    , odom_frame_id_("odom")
    , base_frame_id_("base_link")
    , enable_odom_tf_(true)
    , steering_angle_instead_of_angular_speed_(true)
{
}

bool AckermannController::init(hardware_interface::VelocityJointInterface* hw,
                               ros::NodeHandle& root_nh,
                               ros::NodeHandle &controller_nh)
{
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> left_spinning_names, right_spinning_names;
    std::string left_spinning_joint_name, right_spinning_joint_name;

    if (controller_nh.getParam("left_spinning_joint", left_spinning_joint_name))
    {
        left_spinning_names.push_back(left_spinning_joint_name);
    }

    if(controller_nh.getParam("right_spinning_joint", right_spinning_joint_name))
    {
        right_spinning_names.push_back(right_spinning_joint_name);
    }

    left_spinning_joints_size_ = left_spinning_names.size();
    right_spinning_joints_size_ = right_spinning_names.size();

    left_spinning_joints_.resize(left_spinning_joints_size_);
    right_spinning_joints_.resize(right_spinning_joints_size_);

    has_left_steering_ = left_steering_.init(hw, root_nh, controller_nh, "left_steering_joint");
    has_right_steering_ = right_steering_.init(hw, root_nh, controller_nh, "right_steering_joint");

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    controller_nh.param("length", length_, length_);

    controller_nh.param("wheel_separation_multiplier", wheel_separation_multiplier_, wheel_separation_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by "
                          << wheel_separation_multiplier_ << ".");

    controller_nh.param("wheel_radius_multiplier", wheel_radius_multiplier_, wheel_radius_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Wheel radius will be multiplied by "
                          << wheel_radius_multiplier_ << ".");

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                          << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odom frame_id set to " << odom_frame_id_);

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

    controller_nh.param("steering_angle_instead_of_angular_speed", steering_angle_instead_of_angular_speed_, steering_angle_instead_of_angular_speed_);
    ROS_INFO_STREAM_NAMED(name_, "Steering angle instead of angular speed is " << (steering_angle_instead_of_angular_speed_?"enabled":"disabled"));

    // Velocity and acceleration limits:
    controller_nh.param("has_velocity_limits"    , limiter_.has_velocity_limits    , limiter_.has_velocity_limits    );
    controller_nh.param("has_acceleration_limits", limiter_.has_acceleration_limits, limiter_.has_acceleration_limits);
    controller_nh.param("has_deceleration_limits", limiter_.has_deceleration_limits, limiter_.has_deceleration_limits);
    controller_nh.param("has_jerk_limits"        , limiter_.has_jerk_limits        , limiter_.has_jerk_limits        );
    controller_nh.param("max_velocity"           , limiter_.max_velocity           ,  limiter_.max_velocity          );
    controller_nh.param("min_velocity"           , limiter_.min_velocity           , -limiter_.max_velocity          );
    controller_nh.param("max_acceleration"       , limiter_.max_acceleration       ,  limiter_.max_acceleration      );
    controller_nh.param("min_acceleration"       , limiter_.min_acceleration       , -limiter_.max_acceleration      );
    controller_nh.param("max_deceleration"       , limiter_.max_deceleration       ,  limiter_.max_deceleration      );
    controller_nh.param("min_deceleration"       , limiter_.min_deceleration       , -limiter_.max_deceleration      );
    controller_nh.param("max_jerk"               , limiter_.max_jerk               ,  limiter_.max_jerk              );
    controller_nh.param("min_jerk"               , limiter_.min_jerk               , -limiter_.max_jerk              );

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !controller_nh.getParam("wheel_separation", wheel_separation_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);

    if (!setOdomParamsFromUrdf(root_nh,
                               left_spinning_names[0],
                               right_spinning_names[0],
                               lookup_wheel_separation,
                               lookup_wheel_radius))
    {
        return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws = wheel_separation_multiplier_ * wheel_separation_;
    const double wr = wheel_radius_multiplier_     * wheel_radius_;
    odometry_.setWheelParams(ws, wr);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel separation " << ws
                          << ", wheel radius " << wr);

    setOdomPubFields(root_nh, controller_nh);

    // Get the joint object to use in the realtime loop
    for (int i = 0; i < left_spinning_joints_size_; ++i)
    {
        ROS_INFO_STREAM_NAMED(name_, "Adding left wheel with joint name: " << left_spinning_names[i]);
        left_spinning_joints_[i] = hw->getHandle(left_spinning_names[i]);  // throws on failure
    }
    for (int i = 0; i < right_spinning_joints_size_; ++i)
    {
        ROS_INFO_STREAM_NAMED(name_, "Adding right wheel with joint name: " << right_spinning_names[i]);
        right_spinning_joints_[i] = hw->getHandle(right_spinning_names[i]);  // throws on failure
    }

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &AckermannController::cmdVelCallback, this);

    return true;
}

void AckermannController::update(const ros::Time& time, const ros::Duration& period)
{
    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
        odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
        double left_pos  = 0.0;
        double right_pos = 0.0;
        for (size_t i = 0; i < left_spinning_joints_size_; ++i)
        {
            const double lp = left_spinning_joints_[i].getPosition();
            if (std::isnan(lp))
                return;

            left_pos += lp;
        }
        for (size_t i = 0; i < right_spinning_joints_size_; ++i)
        {
            const double rp = right_spinning_joints_[i].getPosition();
            if (std::isnan(rp))
                return;

            right_pos += rp;
        }

        left_pos  /= left_spinning_joints_size_;
        right_pos /= right_spinning_joints_size_;

        // Estimate linear and angular velocity using joint information
        odometry_.update(left_pos, right_pos, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
        last_state_publish_time_ += publish_period_;
        // Compute and store orientation info
        const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

        // Populate odom message and publish
        if (odom_pub_->trylock())
        {
            odom_pub_->msg_.header.stamp = time;
            odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
            odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
            odom_pub_->msg_.pose.pose.orientation = orientation;
            odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
            odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
            odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = time;
            odom_frame.transform.translation.x = odometry_.getX();
            odom_frame.transform.translation.y = odometry_.getY();
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

    // MOVE ROBOT
    // Retrieve current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
        curr_cmd.lin = 0.0;
        curr_cmd.ang = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Apply multipliers:
    const double ws = wheel_separation_multiplier_ * wheel_separation_;
    const double wr = wheel_radius_multiplier_     * wheel_radius_;

    // Compute wheels velocities:
    const double vel_left  = (curr_cmd.lin * (1.0 + ws * std::tan(curr_cmd.ang) / (2.0 * length_))) / wr;
    const double vel_right = (curr_cmd.lin * (1.0 - ws * std::tan(curr_cmd.ang) / (2.0 * length_))) / wr;

    // Set wheels velocities:
    for (size_t i = 0; i < left_spinning_joints_size_; ++i)
    {
        left_spinning_joints_[i].setCommand(vel_left);
    }
    for (size_t i = 0; i < right_spinning_joints_size_; ++i)
    {
        right_spinning_joints_[i].setCommand(vel_right);
    }

    if(has_left_steering_) left_steering_.update(time, period);
    if(has_right_steering_) right_steering_.update(time, period);
}

void AckermannController::starting(const ros::Time& time)
{
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);

    if(has_left_steering_) left_steering_.starting(time);
    if(has_right_steering_) right_steering_.starting(time);
}

void AckermannController::stopping(const ros::Time& /*time*/)
{
    brake();
}

void AckermannController::brake()
{
    const double vel = 0.0;
    for (size_t i = 0; i < left_spinning_joints_size_; ++i)
    {
        left_spinning_joints_[i].setCommand(vel);
    }
    for (size_t i = 0; i < right_spinning_joints_size_; ++i)
    {
        right_spinning_joints_[i].setCommand(vel);
    }
}

void AckermannController::cmdVelCallback(const geometry_msgs::Twist& command)
{
    if (isRunning())
    {
        command_struct_.ang   = command.angular.z;
        command_struct_.lin   = command.linear.x;
        command_struct_.stamp = ros::Time::now();
        command_.writeFromNonRT (command_struct_);

        double angle = 0.0;

        if(steering_angle_instead_of_angular_speed_)
        {
          angle = command.angular.z;
        }
        else
        {
          angle = command.linear.x * tan(command.angular.z) / length_;
        }

        if(has_left_steering_) left_steering_.enforceJointLimits(angle);
        if(has_right_steering_) right_steering_.enforceJointLimits(angle);
        const double e = wheel_separation_multiplier_ * wheel_separation_ / 2.0;
        double left_angle = std::atan(length_ * std::tan(angle)/std::abs(length_ + e * std::tan(angle)));
        double right_angle = std::atan(length_ * std::tan(angle)/std::abs(length_ - e * std::tan(angle)));
        if(has_left_steering_) left_steering_.setCommand(left_angle);
        if(has_right_steering_) right_steering_.setCommand(right_angle);

        ROS_DEBUG_STREAM_NAMED(name_,
                               "Added values to command. "
                               << "Ang: "   << command_struct_.ang << ", "
                               << "Lin: "   << command_struct_.lin << ", "
                               << "Stamp: " << command_struct_.stamp);
    }
    else
    {
        ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
}

bool AckermannController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
        const std::string& left_wheel_name,
        const std::string& right_wheel_name,
        bool lookup_wheel_separation,
        bool lookup_wheel_radius)
{
    if (!(lookup_wheel_separation || lookup_wheel_radius))
    {
        // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
        return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
        ROS_ERROR_NAMED(name_, "Robot description couldn't be retrieved from param server.");
        return false;
    }

    boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));

    boost::shared_ptr<const urdf::Joint> left_wheel_joint(model->getJoint(left_wheel_name));
    boost::shared_ptr<const urdf::Joint> right_wheel_joint(model->getJoint(right_wheel_name));

    if (lookup_wheel_separation)
    {
        // Get wheel separation
        if (!left_wheel_joint)
        {
            ROS_ERROR_STREAM_NAMED(name_, left_wheel_name
                                   << " couldn't be retrieved from model description");
            return false;
        }

        if (!right_wheel_joint)
        {
            ROS_ERROR_STREAM_NAMED(name_, right_wheel_name
                                   << " couldn't be retrieved from model description");
            return false;
        }

        ROS_INFO_STREAM("left wheel to origin: " << left_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                        << left_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                        << left_wheel_joint->parent_to_joint_origin_transform.position.z);
        ROS_INFO_STREAM("right wheel to origin: " << right_wheel_joint->parent_to_joint_origin_transform.position.x << ","
                        << right_wheel_joint->parent_to_joint_origin_transform.position.y << ", "
                        << right_wheel_joint->parent_to_joint_origin_transform.position.z);

        wheel_separation_ = euclideanOfVectors(left_wheel_joint->parent_to_joint_origin_transform.position,
                                               right_wheel_joint->parent_to_joint_origin_transform.position);

    }

    if (lookup_wheel_radius)
    {
        // Get wheel radius
        if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius_))
        {
            ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << left_wheel_name << " wheel radius");
            return false;
        }
    }

    return true;
}

void AckermannController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
        ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
                                      (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                                      (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
                                      (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
                                      (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
                                      (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
                                      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
                                       (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                                       (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
                                       (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
                                       (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
                                       (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
                                       (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}
}

PLUGINLIB_EXPORT_CLASS(ackerbot_controllers::AckermannController, controller_interface::ControllerBase)
