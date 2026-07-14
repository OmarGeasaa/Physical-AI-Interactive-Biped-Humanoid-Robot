void publish_feedback(const GatewayFeedbackPacket_t& data) {
        
        // ---------------------------------------------------------
        // 1. Pack the Custom Telemetry Array (For your Custom Nodes)
        // ---------------------------------------------------------
        auto fb_msg = std_msgs::msg::Float32MultiArray();
        
        // Add Linear Actuators (6 floats)
        fb_msg.data.insert(fb_msg.data.end(), std::begin(data.lin_actual), std::end(data.lin_actual));
        
        // Add RMD Actuators (2 floats)
        fb_msg.data.insert(fb_msg.data.end(), std::begin(data.rmd_actual), std::end(data.rmd_actual));
        
        // Add IMU Data (5 floats) - MUST match biped_mapping.hpp!
        fb_msg.data.push_back(data.imu_roll);
        fb_msg.data.push_back(data.imu_pitch);
        fb_msg.data.push_back(data.imu_roll_rate);
        fb_msg.data.push_back(data.imu_pitch_rate);
        fb_msg.data.push_back(data.imu_yaw); // <-- Yaw added correctly!
        
        // Publish array ONLY AFTER all 13 elements are added
        feedback_pub_->publish(fb_msg);

        // ---------------------------------------------------------
        // 2. Publish Standard IMU Message (For RViz & Nav2)
        // ---------------------------------------------------------
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "base_link";
        
        imu_msg.angular_velocity.x = data.imu_roll_rate;
        imu_msg.angular_velocity.y = data.imu_pitch_rate;
        
        imu_pub_->publish(imu_msg);
    }
