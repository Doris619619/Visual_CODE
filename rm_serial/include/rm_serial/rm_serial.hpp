#ifndef SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#include "sensor_msgs/msg/joint_state.hpp"

#include "rm_serial/crc.hpp"
#include "rm_serial/packet.hpp"
#include "rm_serial/serial_driver.hpp"
#include "rm_serial/robot_info.hpp"
// ros2
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <rm_decision_interfaces/msg/test_msg.hpp>
#include <rm_decision_interfaces/msg/robot_status.hpp>

#include <pb_rm_interfaces/msg/robot_state_info.hpp>
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "rm_utils/heartbeat.hpp"

using namespace fyt;

namespace rm_serial_driver
{
    class EXTSerialDriver : public rclcpp::Node
    {
    public:
        explicit EXTSerialDriver(const rclcpp::NodeOptions &options);

    private:
        void receiveData();
        
        //...
        void cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void visionTargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
        void cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
        void setParam(const rclcpp::Parameter &param);
        bool getDetectColor(uint8_t robot_id, uint8_t & color);
        //...

        // Serial port
        std::unique_ptr<Port> port_;
        // Param client to set detect_colr
        // using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        // bool initial_set_param_ = false;
        // uint8_t previous_receive_color_ = 0;
        // rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        // ResultFuturePtr set_param_future_;
        // Service client to reset tracker
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
        // Aimimg point receiving from serial port for visualization
        visualization_msgs::msg::Marker aiming_point_;
        // Broadcast tf from odom to gimbal_link
        double timestamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // Transmit referee system
        rclcpp::Publisher<rm_decision_interfaces::msg::TestMsg>::SharedPtr rc_msg_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<pb_rm_interfaces::msg::RobotStateInfo>::SharedPtr robot_state_info_pub_;
        //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_motion_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        // Send and Receive packet (Referee system)
        rm_decision_interfaces::msg::TestMsg test_msg_;

        //...
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_gimbal_joint_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr cmd_tracking_sub_;
        rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_shoot_sub_;
        // ...

        float last_gimbal_pitch_odom_joint_, last_gimbal_yaw_odom_joint_;
        RobotModels robot_models_;

        // Param client to set detect_color
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        ResultFuturePtr set_param_future_;
        std::string detector_node_name_;

        // Heartbeat
        HeartBeatPublisher::SharedPtr heartbeat_;
    };
}

#endif // DOWN_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
