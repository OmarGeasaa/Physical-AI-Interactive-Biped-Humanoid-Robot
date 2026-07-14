#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "biped_utils/biped_mapping.hpp"
#include "biped_kinematics/hip_3upr.hpp"
#include <Eigen/Dense>
#include <cmath>

class HipIKNode : public rclcpp::Node {
public:
    HipIKNode() : Node("hip_ik_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        // Publisher: Sends the 3-element target array to the Command Aggregator Node
        command_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/humanoid/cmd_hip_" + prefix_, 10);

        // Subscriber 1: The LQR Balancing Brain
        lqr_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_pelvis_pose", 10,
            std::bind(&HipIKNode::lqr_callback, this, std::placeholders::_1));

        // Subscriber 2: The Flight Path Generator
        swing_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/swing_foot_pose", 10,
            std::bind(&HipIKNode::swing_callback, this, std::placeholders::_1));

        // Subscriber 3: The Master Clock
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10,
            std::bind(&HipIKNode::gait_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Hip IK Node active for [%s] leg. Synchronous 3-UPR array publishing active.", prefix_.c_str());
    }

private:
    std::string prefix_;
    int current_gait_phase_ = 0;

    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;
    }

    bool is_stance() {
        if (current_gait_phase_ == 0) return true; // Double Support
        if (prefix_ == "left" && current_gait_phase_ == 2) return true;
        if (prefix_ == "right" && current_gait_phase_ == 1) return true;
        return false;
    }

    bool is_swing() {
        if (prefix_ == "left" && current_gait_phase_ == 1) return true;
        if (prefix_ == "right" && current_gait_phase_ == 2) return true;
        return false;
    }

    // Core Execution Function (Calculates and packs the array)
    void execute_3upr_kinematics(double target_z, double target_roll, double target_pitch) {
        Eigen::Vector3d required_lengths = biped_kinematics::hip_3upr::calculate_IK(
            target_z, target_roll, target_pitch);

        auto cmd_msg = std_msgs::msg::Float32MultiArray();

        for (int i = 0; i < 3; i++) {
            double safe_length = required_lengths(i);

            // Hardware Clamps
            if (safe_length < biped_kinematics::hip_3upr::L_MIN) {
                safe_length = biped_kinematics::hip_3upr::L_MIN;
            } else if (safe_length > biped_kinematics::hip_3upr::L_MAX) {
                safe_length = biped_kinematics::hip_3upr::L_MAX;
            }

            // Pack the safe length directly into the array
            cmd_msg.data.push_back(static_cast<float>(safe_length));
        }

        // Fire all 3 commands simultaneously!
        command_pub_->publish(cmd_msg);
    }

    // ----------------------------------------------------
    // The Traffic Cops (Routing data based on phase)
    // ----------------------------------------------------

    void lqr_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!is_stance()) return; // Ignore LQR if this leg is flying!

        double lqr_x = msg->x;
        double lqr_y = msg->y;
        double target_z = msg->z; 

        // Balance Math: Convert Cartesian shift to Tilt
        double target_pitch = std::atan2(lqr_x, target_z); 
        double target_roll  = std::atan2(lqr_y, target_z);

        execute_3upr_kinematics(target_z, target_roll, target_pitch);
    }

    void swing_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!is_swing()) return; // Ignore Flight Path if this leg is balancing!

        double swing_x = msg->x;
        double swing_y = msg->y;
        double swing_z = msg->z; // This will dynamically change to lift the foot!

        // Flight Math: Convert the X/Y target into swing tilt
        double target_pitch = std::atan2(swing_x, 0.6); 
        double target_roll  = std::atan2(swing_y, 0.6);

        execute_3upr_kinematics(swing_z, target_roll, target_pitch);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr lqr_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr swing_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HipIKNode>());
    rclcpp::shutdown();
    return 0;
}
