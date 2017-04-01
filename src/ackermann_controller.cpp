/**
 * @author Enrique Fernández
 * @author Gérald Lelong
 */

#include <cmath>
#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <boost/assign.hpp>
#include <stdexcept>
#include <string>
#include <ackermann_controller/ackermann_controller.h>

using hardware_interface::VelocityJointInterface;
using hardware_interface::PositionJointInterface;
using hardware_interface::JointStateInterface;

namespace ackermann_controller {

AckermannController::AckermannController()
    : open_loop_(false)
    , command_struct_()
    , steering_angle_instead_of_angular_speed_(true)
    , wheelbase_(1.0)
    , cmd_vel_timeout_(0.5)
    , velocity_rolling_window_size_(10)
    , odom_frame_id_("odom")
    , base_frame_id_("base_link")
    , base_link_("base_link")
    , enable_odom_tf_(true)
{
}

bool AckermannController::init(
    hardware_interface::RobotHW * hw,
    ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh)
{
    VelocityJointInterface * vhw = hw->get<VelocityJointInterface>();
    PositionJointInterface * phw = hw->get<PositionJointInterface>();
    JointStateInterface * shw = hw->get<JointStateInterface>();

    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    if (!initParams(controller_nh))
    {
        return false;
    }

    try
    {
        boost::shared_ptr<urdf::ModelInterface> urdf_model = getURDFModel(root_nh);

        std::vector<std::string>::const_iterator it;
        std::vector<std::string> spinning_joint_names = getJointNames(controller_nh, "spinning_joints");
        std::vector<std::string> odometry_joint_names = getJointNames(controller_nh, "odometry_joints");
        std::vector<std::string> steering_joint_names = getJointNames(controller_nh, "steering_joints");

        for (it = spinning_joint_names.begin(); it != spinning_joint_names.end(); ++it)
        {
            ActuatedWheel wheel(*it, base_link_, urdf_model, vhw->getHandle(*it));
            ROS_INFO_STREAM_NAMED(name_,
                                  "Found spinning joint " << wheel.name_
                                  << " with lateral deviation " << wheel.lateral_deviation_
                                  << " and radius " << wheel.radius_);
            spinning_joints_.push_back(wheel);
        }

        for (it = odometry_joint_names.begin(); it != odometry_joint_names.end(); ++it)
        {
            Wheel wheel(*it, base_link_, urdf_model, shw->getHandle(*it));
            ROS_INFO_STREAM_NAMED(name_,
                                  "Found odometry joint " << wheel.name_
                                  << " with lateral deviation " << wheel.lateral_deviation_
                                  << " and radius " << wheel.radius_);
            odometry_joints_.push_back(wheel);
        }

        for (it = steering_joint_names.begin(); it != steering_joint_names.end(); ++it)
        {
            ActuatedJoint steering_joint(*it, base_link_, urdf_model, phw->getHandle(*it));
            ROS_INFO_STREAM_NAMED(name_,
                                  "Found steering joint " << steering_joint.name_
                                  << " with lateral deviation " << steering_joint.lateral_deviation_);
            steering_joints_.push_back(steering_joint);
        }
    }
    catch(const std::runtime_error& e)
    {
        ROS_ERROR_STREAM_NAMED(name_, e.what());
        return false;
    }

    odometry_.setWheelbase(wheelbase_);
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);

    setOdomPubFields(root_nh, controller_nh);

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &AckermannController::cmdVelCallback, this);

    return true;
}

void AckermannController::update(const ros::Time& time, const ros::Duration& period)
{
    updateOdometry(time, period);
    moveRobot(time, period);
}

void AckermannController::updateOdometry(const ros::Time& time, const ros::Duration& period)
{
    if (open_loop_)
    {
        odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
    }
    else
    {
        odometry_.update(steering_joints_, odometry_joints_, time);
    }

    if (last_state_publish_time_ + publish_period_ < time)
    {
        last_state_publish_time_ += publish_period_;

        const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

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
}

void AckermannController::moveRobot(const ros::Time& time, const ros::Duration& period)
{
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

    double angle = 0.0;

    if(steering_angle_instead_of_angular_speed_)
    {
        angle = curr_cmd.ang;
    }
    else
    {
        angle = curr_cmd.lin * tan(curr_cmd.ang) / wheelbase_;
    }

    // Compute steering angles
    for (std::vector<ActuatedJoint>::iterator it = steering_joints_.begin(); it != steering_joints_.end(); ++it)
    {
        double steering_angle = std::atan(wheelbase_ * std::tan(angle)/std::abs(wheelbase_ + it->lateral_deviation_ * std::tan(angle)));
        it->setCommand(steering_angle);
    }

    // Compute wheels velocities
    for (std::vector<ActuatedWheel>::iterator it = spinning_joints_.begin(); it != spinning_joints_.end(); ++it)
    {
        double velocity = (curr_cmd.lin * (1.0 + 2.0 * it->lateral_deviation_ * std::tan(curr_cmd.ang) / (2.0 * wheelbase_))) / it->radius_;
        it->setCommand(velocity);
    }
}

void AckermannController::starting(const ros::Time& time)
{
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
}

void AckermannController::stopping(const ros::Time& /*time*/)
{
    brake();
}

void AckermannController::brake()
{
    const double velocity = 0.0;

    std::vector<ActuatedWheel>::iterator it;
    for (it = spinning_joints_.begin(); it != spinning_joints_.end(); ++it)
    {
        it->setCommand(velocity);
    }
}

void AckermannController::cmdVelCallback(const geometry_msgs::Twist& command)
{
    if (isRunning())
    {
        command_struct_.ang = command.angular.z;
        command_struct_.lin = command.linear.x;
        command_struct_.stamp = ros::Time::now();
        command_.writeFromNonRT(command_struct_);

        ROS_DEBUG_STREAM_NAMED(
            name_,
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

bool AckermannController::initParams(ros::NodeHandle& controller_nh)
{
    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);
    ROS_INFO_STREAM_NAMED(name_, "Open loop is " << (open_loop_?"enabled":"disabled"));

    controller_nh.param("wheelbase", wheelbase_, wheelbase_);
    ROS_INFO_STREAM_NAMED(name_, "Wheelbase set to " << wheelbase_);

    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size_, velocity_rolling_window_size_);
    ROS_INFO_STREAM_NAMED(
        name_, "Velocity rolling window size of " << velocity_rolling_window_size_ << ".");

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odom frame_id set to " << odom_frame_id_);

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("base_model_link", base_link_, base_link_);
    ROS_INFO_STREAM_NAMED(name_, "Base base_model_link set to " << base_link_);

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

    return true;
}

boost::shared_ptr<urdf::ModelInterface> AckermannController::getURDFModel(const ros::NodeHandle& nh)
{
    const std::string model_param_name = "robot_description";
    std::string robot_model_str = "";

    if (!nh.hasParam(model_param_name) || !nh.getParam(model_param_name, robot_model_str))
    {
        throw std::runtime_error("Robot description couldn't be retrieved from param server.");
    }

    boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));

    return model;
}

std::vector<std::string> AckermannController::getJointNames(
    ros::NodeHandle& controller_nh,
    const std::string& param)
{
    std::vector<std::string> names;

    XmlRpc::XmlRpcValue wheel_list;
    if (!controller_nh.getParam(param, wheel_list))
    {
        throw std::runtime_error(std::string("Couldn't retrieve param '") + param + "'.");
    }

    if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        if (wheel_list.size() == 0)
        {
            throw std::runtime_error(param + " is an empty list");
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
            if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                throw std::runtime_error(param + " child isn't a string.");
            }
        }

        names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
            names[i] = static_cast<std::string>(wheel_list[i]);
        }
    }
    else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        names.push_back(wheel_list);
    }
    else
    {
        throw std::runtime_error(param + " is neither a list of strings nor a string.");
    }

    return names;
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
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}
}

PLUGINLIB_EXPORT_CLASS(ackermann_controller::AckermannController, controller_interface::ControllerBase)
