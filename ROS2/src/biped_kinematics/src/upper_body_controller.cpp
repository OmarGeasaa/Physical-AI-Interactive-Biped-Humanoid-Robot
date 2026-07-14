#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "biped_utils/biped_mapping.hpp"

class UpperBodyController : public rclcpp::Node {
public:
    UpperBodyController() : Node("upper_body_controller") {
        
        // Publisher: Sends the 4-element array to the Command Aggregator
        command_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/humanoid/cmd_upper_body", 10);

        // Subscriber: Nav2 Waypoint Task Executor
        target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/upper_body/targets", 10,
            std::bind(&UpperBodyController::target_callback, this, std::placeholders::_1));

        // Subscriber: LQR Master Clock
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10,
            std::bind(&UpperBodyController::gait_callback, this, std::placeholders::_1));

        // Initialize upper body memory cache to safe neutral positions (0.0)
        // Layout: [Neck, Shoulder, Gripper, Wrist_DC]
        upper_body_state_.resize(4, 0.0f);

        RCLCPP_INFO(this->get_logger(), "Upper Body Controller Online. Array caching active. Ready for Nav2 Waypoint tasks.");
    }

private:
    int current_gait_phase_ = 0; 
    sensor_msgs::msg::JointState::SharedPtr pending_task_ = nullptr;
    std::vector<float> upper_body_state_;

    // ----------------------------------------------------
    // 1. The Callbacks
    // ----------------------------------------------------

    void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        pending_task_ = msg;
        
        if (current_gait_phase_ != 0) {
            RCLCPP_INFO(this->get_logger(), "Task buffered. Waiting for legs to enter Double Support...");
        } else {
            execute_pending_task();
        }
    }

    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;

        if (current_gait_phase_ == 0 && pending_task_ != nullptr) {
            RCLCPP_INFO(this->get_logger(), "Double Support achieved. Executing buffered Nav2 task.");
            execute_pending_task();
        }
    }

    // ----------------------------------------------------
    // 2. Hardware Execution & Array Packing
    // ----------------------------------------------------

    void execute_pending_task() {
        if (pending_task_ == nullptr) return;

        // Update the internal cache with ONLY the joints specified in the Nav2 message
        for (size_t i = 0; i < pending_task_->name.size(); i++) {
            std::string name = pending_task_->name[i];

            if (name == "neck_pan" && pending_task_->position.size() > i) {
                upper_body_state_[0] = pending_task_->position[i];
            } 
            else if (name == "shoulder_pitch" && pending_task_->position.size() > i) {
                upper_body_state_[1] = pending_task_->position[i];
            } 
            else if (name == "gripper_servo" && pending_task_->position.size() > i) {
                upper_body_state_[2] = pending_task_->position[i];
            } 
            else if (name == "wrist_dc_motor" && pending_task_->velocity.size() > i) {
                // The DC motor requires velocity commands instead of position
                upper_body_state_[3] = pending_task_->velocity[i];
            } 
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown joint or missing data: %s. Skipping.", name.c_str());
            }
        }

        // Pack the fully updated state into the message
        auto cmd_msg = std_msgs::msg::Float32MultiArray();
        cmd_msg.data = upper_body_state_;

        // Send a single synchronized array to the Aggregator
        command_pub_->publish(cmd_msg);

        // Wipe the buffer clean
        pending_task_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "Task complete. Awaiting next Nav2 Waypoint.");
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UpperBodyController>());
    rclcpp::shutdown();
    return 0;
}
