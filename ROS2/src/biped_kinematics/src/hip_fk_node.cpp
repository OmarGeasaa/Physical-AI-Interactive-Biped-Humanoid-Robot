#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "biped_utils/biped_mapping.hpp"
#include "biped_kinematics/hip_3upr.hpp"
#include <Eigen/Dense>

class HipFKNode : public rclcpp::Node {
public:
    HipFKNode() : Node("hip_fk_node") {
        this->declare_parameter<std::string>("leg_prefix", "left");
        prefix_ = this->get_parameter("leg_prefix").as_string();

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Subscriber: Listens to the global telemetry array from the Gateway
        state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/humanoid/actuator_feedback", 10,
            std::bind(&HipFKNode::state_callback, this, std::placeholders::_1));

        // Initialize the solver guess to neutral (Z translation only)
        current_pose_guess_ << 0.0, 0.0, 0.0;

        RCLCPP_INFO(this->get_logger(), "Hip FK Node Active [%s]. Parsing flat telemetry array for synchronous 3-UPR solve.", prefix_.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;

    std::string prefix_;
    Eigen::Vector3d current_pose_guess_;

    void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        // BOUND SAFETY FILTER: Ignore incomplete arrays
        if (msg->data.size() < 12) return;

        Eigen::Vector3d current_lengths;

        // 1. Extract the synchronous snapshot of the 3 actuators using the dictionary
        if (prefix_ == "left") {
            current_lengths(0) = msg->data[biped_mapping::fb::LIN_ACTUAL_0];
            current_lengths(1) = msg->data[biped_mapping::fb::LIN_ACTUAL_1];
            current_lengths(2) = msg->data[biped_mapping::fb::LIN_ACTUAL_2];
        } else {
            current_lengths(0) = msg->data[biped_mapping::fb::LIN_ACTUAL_3];
            current_lengths(1) = msg->data[biped_mapping::fb::LIN_ACTUAL_4];
            current_lengths(2) = msg->data[biped_mapping::fb::LIN_ACTUAL_5];
        }

        // 2. Hardware Safety Check (Clamp the incoming sensor data)
        for(int i = 0; i < 3; ++i) {
            current_lengths(i) = std::max(biped_kinematics::hip_3upr::L_MIN, 
                                 std::min(biped_kinematics::hip_3upr::L_MAX, current_lengths(i)));
        }

        // 3. Run the Newton-Raphson Solver on the perfect snapshot
        Eigen::Vector3d solved_pose = biped_kinematics::hip_3upr::calculate_FK(
            current_lengths, current_pose_guess_);

        // Update the guess for the next tick to keep the solver fast
        current_pose_guess_ = solved_pose;

        // 4. Package and publish the Virtual Joint States for RViz & TF Tree
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        
        joint_msg.name.push_back(prefix_ + "_hip_z_joint");
        joint_msg.name.push_back(prefix_ + "_hip_roll_joint");
        joint_msg.name.push_back(prefix_ + "_hip_pitch_joint");
        
        joint_msg.position.push_back(solved_pose(0)); // Z translation
        joint_msg.position.push_back(solved_pose(1)); // Roll
        joint_msg.position.push_back(solved_pose(2)); // Pitch

        // Linear actuators don't inherently provide effort data in this struct yet, defaulting to 0.0
        joint_msg.effort = {0.0, 0.0, 0.0}; 

        joint_pub_->publish(joint_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "[%s Hip] Z: %.3fm | Roll: %.3frad | Pitch: %.3frad", 
            prefix_.c_str(), solved_pose(0), solved_pose(1), solved_pose(2));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HipFKNode>());
    rclcpp::shutdown();
    return 0;
}
