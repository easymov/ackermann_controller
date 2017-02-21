#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "ackerbot_controllers/odometry.h"
#include "ackerbot_controllers/speed_limiter.h"
#include "ackerbot_controllers/joint_position_controller.h"

namespace ackerbot_controllers {

class AckermannController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
    public:

        AckermannController();

        /**
        * \brief Initialize controller
        * \param hw            Velocity joint interface for the wheels
        * \param root_nh       Node handle at root namespace
        * \param controller_nh Node handle inside the controller namespace
        */
        bool init(hardware_interface::VelocityJointInterface* hw,
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

        std::vector<hardware_interface::JointHandle> left_spinning_joints_;
        std::vector<hardware_interface::JointHandle> right_spinning_joints_;
        std::vector<hardware_interface::JointHandle> left_steering_joints_;
        std::vector<hardware_interface::JointHandle> right_steering_joints_;

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
        ackerbot_controllers::Odometry odometry_;

        double length_;
        double wheel_separation_;
        double wheel_radius_;
        double wheel_separation_multiplier_;
        double wheel_radius_multiplier_;
        double cmd_vel_timeout_;
        std::string odom_frame_id_;
        std::string base_frame_id_;
        bool enable_odom_tf_;
        size_t left_spinning_joints_size_;
        size_t right_spinning_joints_size_;
        bool has_left_steering_;
        bool has_right_steering_;

        Commands last1_cmd_;
        Commands last0_cmd_;
        ackerbot_controllers::SpeedLimiter limiter_;
        ackerbot_controllers::JointPositionController left_steering_;
        ackerbot_controllers::JointPositionController right_steering_;


    private:

        void brake();

        void cmdVelCallback(const geometry_msgs::Twist& command);

        /**
        * \brief Get the wheel names from a wheel param
        * \param [in]  controller_nh Controller node handler
        * \param [in]  wheel_param   Param name
        * \param [out] wheel_names   Vector with the whel names
        * \return true if the wheel_param is available and the wheel_names are
        *        retrieved successfully from the param server; false otherwise
        */
        bool getWheelNames(ros::NodeHandle& controller_nh,
                        const std::string& wheel_param,
                        std::vector<std::string>& wheel_names);

        /**
        * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
        * \param root_nh Root node handle
        * \param left_wheel_name Name of the left wheel joint
        * \param right_wheel_name Name of the right wheel joint
        */
        bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                                const std::string& left_wheel_name,
                                const std::string& right_wheel_name,
                                bool lookup_wheel_separation,
                                bool lookup_wheel_radius);

        /**
        * \brief Sets the odometry publishing fields
        * \param root_nh Root node handle
        * \param controller_nh Node handle inside the controller namespace
        */
        void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
};
}
