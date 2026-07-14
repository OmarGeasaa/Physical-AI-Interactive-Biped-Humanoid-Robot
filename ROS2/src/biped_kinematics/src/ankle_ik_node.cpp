#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "biped_utils/biped_mapping.hpp"
#include "biped_kinematics/ankle_math.hpp"

class AnkleIKNode : public rclcpp::Node {
public:
    AnkleIKNode() : Node("ankle_ik_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        // Publisher: Sends the target position to the Command Aggregator
        command_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/humanoid/cmd_ankle_" + prefix_, 10);

        // Subscribes to the Swing trajectory target
        target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/walking_pattern/" + prefix_ + "_ankle_pitch", 10,
            std::bind(&AnkleIKNode::target_callback, this, std::placeholders::_1));

        // Subscribes to the Master Clock
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10,
            std::bind(&AnkleIKNode::gait_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ankle IK Node operational for [%s] leg. Strict Position Control active.", prefix_.c_str());
    }

private:
    std::string prefix_;
    int current_gait_phase_ = 0; // Defaults to Double Support

    // Master Clock Updater
    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;
    }

    // Identity Translation Logic
    bool is_swing() {
        if (prefix_ == "left" && current_gait_phase_ == 1) return true;
        if (prefix_ == "right" && current_gait_phase_ == 2) return true;
        return false;
    }

    void target_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        double target_rad;

        if (is_swing()) {
            // SWING PHASE: Track the cubic spline to clear the floor
            target_rad = std::max(-0.1745, std::min(0.2618, (double)msg->data));
        } else {
            // STANCE PHASE: Ignore the swing trajectory. Lock flat to the floor.
            // The RMD's internal PID will hold this position aggressively.
            target_rad = 0.0;
        }

        // Solve the kinematics for the RMD actuator position
        // (Note: Retaining your original math function here. If the RMD is direct-drive, 
        // you could bypass this and just pass target_rad directly).
        double required_position = biped_kinematics::calculate_required_actuator_length(target_rad);

        // Prepare the command for the Aggregator Node
        auto cmd_msg = std_msgs::msg::Float32MultiArray();
        
        // Pack the raw target position into the array
        cmd_msg.data.push_back(static_cast<float>(required_position));

        // Fire the command!
        command_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnkleIKNode>());
    rclcpp::shutdown();
    return 0;
}
