#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "biped_utils/biped_mapping.hpp"

using namespace std::chrono_literals;

class BipedOdometryNode : public rclcpp::Node {
public:
    BipedOdometryNode() : Node("biped_odometry_node") {
        // 1. Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 2. Subscribers
        gait_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/gait_phase", 10, std::bind(&BipedOdometryNode::gait_callback, this, std::placeholders::_1));
            
        // NEW: Subscribes to the STM32 Gateway telemetry array instead of a standalone IMU topic
        feedback_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/humanoid/actuator_feedback", 10, std::bind(&BipedOdometryNode::feedback_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&BipedOdometryNode::joint_callback, this, std::placeholders::_1));

        // 3. 100Hz Publishing Loop
        timer_ = this->create_wall_timer(
            10ms, std::bind(&BipedOdometryNode::publish_odometry, this));

        RCLCPP_INFO(this->get_logger(), "Biped State Estimator Online. Fusing Telemetry Array and FK.");
    }

private:
    // Global Tracking Variables
    double global_x_ = 0.0;
    double global_y_ = 0.0;
    
    // IMU Variables
    double global_roll_ = 0.0;
    double global_pitch_ = 0.0;
    double global_yaw_ = 0.0; // Defaults to 0 until added to STM32 firmware
    
    int current_gait_phase_ = 0; 
    int previous_gait_phase_ = 0;

    // Anchor points (Where was the pelvis when the foot hit the floor?)
    double anchor_x_ = 0.0;
    double anchor_y_ = 0.0;

    // --- Callbacks ---

    void gait_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_gait_phase_ = msg->data;

        // Detect the exact moment a new foot plants on the ground
        if (current_gait_phase_ != previous_gait_phase_) {
            anchor_x_ = global_x_;
            anchor_y_ = global_y_;
        }
        previous_gait_phase_ = current_gait_phase_;
    }

    void feedback_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 12) return;

        // Extract the IMU data using the biped_utils dictionary!
        global_roll_ = msg->data[biped_mapping::fb::IMU_ROLL];
        global_pitch_ = msg->data[biped_mapping::fb::IMU_PITCH];
        
        // Note: global_yaw_ is omitted here as it is not in the current GatewayFeedbackPacket_t
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Here you extract the real-time FK translation of the pelvis 
        // derived from the biped_kinematics hip_fk_node.
        
        double current_stance_dx = 0.0; // Calculate forward shift based on hip states
        double current_stance_dy = 0.0; // Calculate lateral shift based on hip states

        // Accumulate onto the global map based on which leg is planted
        if (current_gait_phase_ == 1) { 
            global_x_ = anchor_x_ + current_stance_dx;
            global_y_ = anchor_y_ + current_stance_dy;
        } 
        else if (current_gait_phase_ == 2) { 
            global_x_ = anchor_x_ + current_stance_dx;
            global_y_ = anchor_y_ + current_stance_dy;
        }
    }

    void publish_odometry() {
        rclcpp::Time now = this->get_clock()->now();

        // 1. Convert STM32 Euler Angles to Quaternion for ROS 2 TF/Odom
        tf2::Quaternion q;
        q.setRPY(global_roll_, global_pitch_, global_yaw_);

        // --- ENVELOPE 1: THE TF BROADCAST ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";          
        t.child_frame_id = "base_footprint"; 
        
        t.transform.translation.x = global_x_;
        t.transform.translation.y = global_y_;
        t.transform.translation.z = 0.0;     
        
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);

        // --- ENVELOPE 2: THE ODOM TOPIC ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose.position.x = global_x_;
        odom_msg.pose.pose.position.y = global_y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gait_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BipedOdometryNode>());
    rclcpp::shutdown();
    return 0;
}