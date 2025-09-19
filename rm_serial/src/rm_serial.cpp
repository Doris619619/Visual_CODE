#include "rm_serial/rm_serial.hpp"

namespace rm_serial_driver
{
    EXTSerialDriver::EXTSerialDriver(const rclcpp::NodeOptions &options)
        : Node("rm_serial_ext", options), port_{new Port(2)}
    {
        RCLCPP_INFO(get_logger(), "Start EXTSerialDriver!");

        port_->getParams("/dev/ttyACM0", 115200, "none", "none", "1");

        // Create Publisher
        // rc_msg_pub_ =
        //     this->create_publisher<rm_decision_interfaces::msg::TestMsg>("/test_msg", 10);
        // latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
        // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
        // // joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);
        // // TF broadcaster
        // timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // // Detect parameter client
        // detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
        // // Tracker reset service client
        // reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);
        robot_state_info_pub_ =
            this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
        joint_state_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
        //robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);

        cmd_gimbal_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "cmd_gimbal_joint", 10,
            std::bind(&EXTSerialDriver::cmdGimbalJointCallback, this, std::placeholders::_1));

        cmd_tracking_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
            "tracker/target", 10,
            std::bind(&EXTSerialDriver::visionTargetCallback, this, std::placeholders::_1));
        
        cmd_shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
            "cmd_shoot", 10,
            std::bind(&EXTSerialDriver::cmdShootCallback, this, std::placeholders::_1));
            
        try
        {
            port_->serial_driver_->init_port(port_->device_name_, *port_->device_config_);
            if (!port_->serial_driver_->port()->is_open())
            {
                port_->serial_driver_->port()->open();
                port_->receive_thread_ = std::thread(&EXTSerialDriver::receiveData, this);
                // LOG
                RCLCPP_INFO(get_logger(), "serial open OK!");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port: %s - %s", port_->device_name_.c_str(), ex.what());
            throw ex;
        }

        // aiming_point_.header.frame_id = "odom";
        // aiming_point_.ns = "aiming_point";
        // aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
        // aiming_point_.action = visualization_msgs::msg::Marker::ADD;
        // aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
        // aiming_point_.color.r = 1.0;
        // aiming_point_.color.g = 1.0;
        // aiming_point_.color.b = 1.0;
        // aiming_point_.color.a = 1.0;
        // aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

        robot_models_.chassis = {
            {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
        robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
        robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
        robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
        robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};
    
        // Heartbeat
        heartbeat_ = HeartBeatPublisher::create(this);
    }

    void EXTSerialDriver::receiveData()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        data.reserve(sizeof(DownReceivePacket));

        while (rclcpp::ok())
        {
            try
            {
                port_->serial_driver_->port()->receive(header);

                if (header[0] == 0x5A)
                {
                    data.resize(sizeof(DownReceivePacket) - 1);
                    port_->serial_driver_->port()->receive(data);

                    data.insert(data.begin(), header[0]);
                    DownReceivePacket packet = fromVector<DownReceivePacket>(data);

                    bool crc_ok =
                        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

                    if (crc_ok)
                    {

                        // RCLCPP_INFO(get_logger(), "CRC OK!");
                        // receive RC
                        // test_msg_.linear_vx = packet.linear_vx;
                        // test_msg_.linear_vy = packet.linear_vy;
                        // test_msg_.linear_vw = packet.linear_vw;
                        // test_msg_.mode = packet.mode;
                        // rc_msg_pub_->publish(test_msg_);

                        // if (!initial_set_param_ || packet.detect_color != previous_receive_color_)
                        // {
                        //     setParam(rclcpp::Parameter("detect_color", packet.detect_color));
                        //     previous_receive_color_ = packet.detect_color;
                        // }
                        // if (packet.reset_tracker)
                        // {
                        //     resetTracker();
                        // }

                        // geometry_msgs::msg::TransformStamped t;
                        // timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                        // t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                        // t.header.frame_id = "odom";
                        // t.child_frame_id = "gimbal_link";
                        // tf2::Quaternion q;
                        // q.setRPY(packet.roll, packet.pitch, packet.yaw);
                        // t.transform.rotation = tf2::toMsg(q);
                        // tf_broadcaster_->sendTransform(t);

                        // if (abs(packet.aim_x) > 0.01)
                        // {
                        //     aiming_point_.header.stamp = this->now();
                        //     aiming_point_.pose.position.x = packet.aim_x;
                        //     aiming_point_.pose.position.y = packet.aim_y;
                        //     aiming_point_.pose.position.z = packet.aim_z;
                        //     marker_pub_->publish(aiming_point_);
                        // }
                        
                        // auto msg = sensor_msgs::msg::JointState();
                        // msg.header.stamp = now();
                        // msg.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
                        // msg.position = {packet.pitch, packet.yaw};
                        // joint_pub_->publish(msg);
                        
                        // publishImuData
                        sensor_msgs::msg::JointState joint_msg;
                        sensor_msgs::msg::Imu imu_msg;
                        imu_msg.header.stamp = joint_msg.header.stamp = now();
                        imu_msg.header.frame_id = "gimbal_pitch_odom";
                      
                        // Convert Euler angles to quaternion
                        tf2::Quaternion q2;
                        q2.setRPY(packet.roll, packet.pitch, packet.yaw);
                        imu_msg.orientation = tf2::toMsg(q2);
                        // imu_msg.angular_velocity.x = packet.roll_vel;
                        // imu_msg.angular_velocity.y = packet.pitch_vel;
                        // imu_msg.angular_velocity.z = packet.yaw_vel;
                        imu_msg.angular_velocity.x = packet.roll;
                        imu_msg.angular_velocity.y = packet.pitch;
                        imu_msg.angular_velocity.z = packet.yaw;
                        imu_pub_->publish(imu_msg);
                        
                        last_gimbal_pitch_odom_joint_ = packet.pitch_odom;
                        last_gimbal_yaw_odom_joint_ = packet.yaw_odom;
                        joint_msg.name = {
                          "gimbal_pitch_joint",
                          "gimbal_yaw_joint",
                          "gimbal_pitch_odom_joint",
                          "gimbal_yaw_odom_joint",
                        };
                        joint_msg.position = {
                            -packet.pitch,
                            packet.yaw,
                            -last_gimbal_pitch_odom_joint_,
                            last_gimbal_yaw_odom_joint_,
                        };
                        joint_state_pub_->publish(joint_msg);

                        uint8_t detect_color;
                        // RCLCPP_ERROR(get_logger(), "robot id: %d", packet.robot_id);
                        if (getDetectColor(packet.robot_id, detect_color)) {
                          if (!initial_set_param_ || detect_color != previous_receive_color_) {
                            previous_receive_color_ = detect_color;
                            setParam(rclcpp::Parameter("detect_color", detect_color));
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                          }
                        }

                        // publishRobotMotion
                        //geometry_msgs::msg::Twist msg;

                        //msg.linear.x = packet.linear_vx;
                        //msg.linear.y = packet.linear_vy;
                        //msg.angular.z = packet.linear_vw;
                      
                        //robot_motion_pub_->publish(msg);

                        // publishRobotInfo
                        // pb_rm_interfaces::msg::RobotStateInfo ri_msg;
                        // rclcpp::Time current_time = now();
                        // ri_msg.header.stamp.sec = current_time.seconds();
                        // ri_msg.header.stamp.nanosec = current_time.nanoseconds();
                        // ri_msg.header.frame_id = "odom";
                      
                        // ri_msg.models.chassis = robot_models_.chassis.at(3);
                        // ri_msg.models.gimbal = robot_models_.gimbal.at(3);
                        // ri_msg.models.shoot = robot_models_.shoot.at(3);
                        // ri_msg.models.arm = robot_models_.arm.at(3);
                        // ri_msg.models.custom_controller =
                        //   robot_models_.custom_controller.at(3);
                      
                        // robot_state_info_pub_->publish(ri_msg);
		            }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "CRC error!");
                    }
                }
                // else if (header[0] == 0x68)
                // {
                //     data.resize(sizeof(EXTReceivePacket) - 1);
                //     port_->serial_driver_->port()->receive(data);

                //     data.insert(data.begin(), header[0]);
                //     EXTReceivePacket packet = fromVector<EXTReceivePacket>(data);

                //     bool crc_ok =
                //         crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

                //     if (crc_ok)
                //     {
                //         uint8_t detect_color;
                //         RCLCPP_ERROR(get_logger(), "robot id: %d", packet.robot_id);
                //         if (getDetectColor(packet.robot_id, detect_color)) {
                //           if (!initial_set_param_ || detect_color != previous_receive_color_) {
                //             previous_receive_color_ = detect_color;
                //             setParam(rclcpp::Parameter("detect_color", detect_color));
                //             std::this_thread::sleep_for(std::chrono::milliseconds(500));
                //           }
                //         }
		            // }
                //     else
                //     {
                //         RCLCPP_ERROR(get_logger(), "CRC error!");
                //     }
                // }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
                port_->reopenPort();
            }
        }
    }


    //...
    void EXTSerialDriver::cmdGimbalJointCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        if (msg->name.size() != msg->position.size()) {
          RCLCPP_ERROR(
            get_logger(), "JointState message name and position arrays are of different sizes");
          return;
        }
        try
        {
            SendPacket packet;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "gimbal_pitch_joint") {
                    packet.pitch = -msg->position[i];
                } else if (msg->name[i] == "gimbal_yaw_joint") {
                    packet.yaw = msg->position[i];
                }
            }

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            port_->reopenPort();
        }
      }
      
      void EXTSerialDriver::visionTargetCallback(
        const auto_aim_interfaces::msg::Target::SharedPtr msg)
      {
        try
        {
            SendPacket packet;
            packet.tracking = msg->tracking;

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            port_->reopenPort();
        }
      }

      void EXTSerialDriver::cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
      {
        try
        {
            SendPacket packet;
            packet.fric_on = true;
            packet.fire = msg->data;

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            port_->reopenPort();
        }
      }
      
    //...
    void EXTSerialDriver::setParam(const rclcpp::Parameter & param)
    {
      if (!initial_set_param_) {
        auto node_graph = this->get_node_graph_interface();
        auto node_names = node_graph->get_node_names();
        std::vector<std::string> possible_detectors = {
          "armor_detector_openvino", "armor_detector_opencv"};
    
        for (const auto & name : possible_detectors) {
          for (const auto & node_name : node_names) {
            if (node_name.find(name) != std::string::npos) {
              detector_node_name_ = node_name;
              break;
            }
          }
          if (!detector_node_name_.empty()) {
            break;
          }
        }
    
        if (detector_node_name_.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
          return;
        }
    
        detector_param_client_ =
          std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
        if (!detector_param_client_->service_is_ready()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
          return;
        }
      }
    
      if (
        !set_param_future_.valid() ||
        set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
        set_param_future_ = detector_param_client_->set_parameters(
          {param}, [this, param](const ResultFuturePtr & results) {
            for (const auto & result : results.get()) {
              if (!result.successful) {
                RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                return;
              }
            }
            RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
            initial_set_param_ = true;
          });
      }
    }
    bool EXTSerialDriver::getDetectColor(uint8_t robot_id, uint8_t & color)
    {
      if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
        return false;
      }
      color = (robot_id >= 100) ? 0 : 1;
      return true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::EXTSerialDriver)
