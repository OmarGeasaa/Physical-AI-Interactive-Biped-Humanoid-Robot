#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "biped_utils/biped_mapping.hpp"
#include "biped_kinematics/ankle_math.hpp"

class AnkleFKNode : public rclcpp::Node {
public:
    AnkleFKNode() : Node("ankle_fk_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        // Publisher: Broadcasts standard Joint States for RViz and TF Tree
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Subscriber: Listens to the global telemetry array from the Gateway
        state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/humanoid/actuator_feedback", 10,
            std::bind(&AnkleFKNode::state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ankle FK Node Active [%s]. Parsing flat telemetry array.", prefix_.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
    std::string prefix_;

    void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        // BOUND SAFETY FILTER: Ignore corrupted or incomplete telemetry arrays
        if (msg->data.size() < 12) return;

        // Extract the RMD motor data using the mapping dictionary
        int index = (prefix_ == "left") ? biped_mapping::fb::RMD_ACTUAL_0 : biped_mapping::fb::RMD_ACTUAL_1;
        double current_rmd_angle = msg->data[index];

        // --- HARDWARE DATA LIMITATION ---
        // The STM32 GatewayFeedbackPacket_t currently only transmits rmd_actual.
        // Until the STM32 firmware is updated to transmit the external encoder 
        // and current, we will approximate external_angle to the rigid angle.
        
        // 1. Execute the pure Math to find the Rigid Angle
        double rigid_angle_rad = biped_kinematics::calculate_rigid_ankle_angle(current_rmd_angle);
        
        // Placeholders until added to biped_mapping.hpp & STM32 firmware
        double external_angle = rigid_angle_rad; 
        double actual_current = 0.0;

        // 2. Calculate actual spring deflection
        double spring_deflection = rigid_angle_rad - external_angle;

        // 3. Package and publish the Standard ROS 2 JointState
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        
        joint_msg.name.push_back(prefix_ + "_ankle_joint"); 
        joint_msg.position.push_back(external_angle); 
        joint_msg.effort.push_back(actual_current);

        joint_pub_->publish(joint_msg);
        
        // Restored Debug Logging
        RCLCPP_DEBUG(this->get_logger(), 
            "[%s Ankle] Motor Ang: %.4f -> Rigid Ang: %.3frad | Ext Ang: %.3frad | Deflection: %.3frad", 
            prefix_.c_str(), current_rmd_angle, rigid_angle_rad, external_angle, spring_deflection);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnkleFKNode>());
    rclcpp::shutdown();
    return 0;
}
