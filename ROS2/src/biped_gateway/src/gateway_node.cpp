#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Linux headers for serial port
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#pragma pack(push, 1)
/* Data received from Jetson (Command Payload) - 48 Bytes */
struct GatewayCommandPacket_t {
    uint8_t start1;
    uint8_t start2;
    float lin_ref[6];
    float rmd_ref[2];
    float servo_ref[3];
    int8_t dc_motor_1;
    int8_t dc_motor_2;
};

/* Data transmitted to Jetson (Feedback Payload) - 54 Bytes */
struct GatewayFeedbackPacket_t {
    uint8_t start1;
    uint8_t start2;
    float lin_actual[6];
    float rmd_actual[2];
    float imu_roll;
    float imu_pitch;
    float imu_roll_rate;
    float imu_pitch_rate;
    float imu_yaw;
};
#pragma pack(pop)

class NucleoGatewayNode : public rclcpp::Node {
public:
    NucleoGatewayNode() : Node("nucleo_gateway_node"), serial_fd_(-1) {

        // 1. Initialize Serial Port (Restored to B115200 to match STM32CubeMX config)
        if (!init_serial_port("/dev/nucleo", B115200)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port. Check permissions or udev rules.");
            rclcpp::shutdown();
        }

        // 2. Publishers (Telemetry to ROS 2)
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/humanoid/imu", 10);
        feedback_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/humanoid/actuator_feedback", 10);
        
        // 3. Subscribers (Commands from ROS 2)
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/humanoid/commands", 10,
            std::bind(&NucleoGatewayNode::command_callback, this, std::placeholders::_1)
        );

        // 4. Timer to poll UART for incoming feedback (~100Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&NucleoGatewayNode::read_serial_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "C++ Nucleo Gateway Bridge Active. Routing 13-element telemetry arrays.");
    }

    ~NucleoGatewayNode() {
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr feedback_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool init_serial_port(const char* port_name, int baud_rate) {
        serial_fd_ = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);

        tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, ignore modem control lines
        tty.c_cflag &= ~CSIZE;              // Clear current data size setting
        tty.c_cflag |= CS8;                 // 8-bit characters
        tty.c_cflag &= ~PARENB;             // No parity bit
        tty.c_cflag &= ~CSTOPB;             // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;            // No hardware flow control

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); 
        tty.c_oflag &= ~OPOST;                          // Raw output
        tty.c_cc[VMIN] = 0;  // Non-blocking read
        tty.c_cc[VTIME] = 0;

        // Clear out any garbage data Linux queued up while the node was offline
        tcflush(serial_fd_, TCIOFLUSH); 

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
        return true;
    }

    void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 13) {
            RCLCPP_WARN(this->get_logger(), "Received incomplete command array. Expected 13 elements.");
            return;
        }

        GatewayCommandPacket_t tx_packet;

        // Zero out memory to prevent struct padding errors
        std::memset(&tx_packet, 0, sizeof(GatewayCommandPacket_t));

        tx_packet.start1 = 0xAA;
        tx_packet.start2 = 0x55;
        
        // Populate struct directly from ROS 2 message array
        std::memcpy(tx_packet.lin_ref, msg->data.data(), 6 * sizeof(float));
        std::memcpy(tx_packet.rmd_ref, msg->data.data() + 6, 2 * sizeof(float));
        std::memcpy(tx_packet.servo_ref, msg->data.data() + 8, 3 * sizeof(float));

        tx_packet.dc_motor_1 = static_cast<int8_t>(msg->data[11]);
        tx_packet.dc_motor_2 = static_cast<int8_t>(msg->data[12]);

        // Write bytes and capture the return length for terminal logging
        int bytes_written = write(serial_fd_, &tx_packet, sizeof(GatewayCommandPacket_t));

        if (bytes_written != sizeof(GatewayCommandPacket_t)) {
            RCLCPP_ERROR(this->get_logger(), "UART Write failed! Only wrote %d bytes", bytes_written);
        }
    }

    void read_serial_loop() {
        uint8_t byte;
        // Search for alignment bytes
        if (read(serial_fd_, &byte, 1) > 0 && byte == 0xAA) {
            if (read(serial_fd_, &byte, 1) > 0 && byte == 0x55) {
                GatewayFeedbackPacket_t rx_packet;
                rx_packet.start1 = 0xAA;
                rx_packet.start2 = 0x55;

                // Read the remaining bytes of the struct
                int bytes_to_read = sizeof(GatewayFeedbackPacket_t) - 2;
                uint8_t* ptr = reinterpret_cast<uint8_t*>(&rx_packet) + 2;

                int total_read = 0;
                while (total_read < bytes_to_read) {
                    int n = read(serial_fd_, ptr + total_read, bytes_to_read - total_read);
                    if (n > 0) total_read += n;
                }

                publish_feedback(rx_packet);
            }
        }
    }

    void publish_feedback(const GatewayFeedbackPacket_t& data) {
        
        // ---------------------------------------------------------
        // 1. Pack the Custom Telemetry Array
        // ---------------------------------------------------------
        auto fb_msg = std_msgs::msg::Float32MultiArray();
        
        // Add Linear Actuators (6 floats)
        fb_msg.data.insert(fb_msg.data.end(), std::begin(data.lin_actual), std::end(data.lin_actual));
        
        // Add RMD Actuators (2 floats)
        fb_msg.data.insert(fb_msg.data.end(), std::begin(data.rmd_actual), std::end(data.rmd_actual));
        
        // Add IMU Data (5 floats)
        fb_msg.data.push_back(data.imu_roll);
        fb_msg.data.push_back(data.imu_pitch);
        fb_msg.data.push_back(data.imu_roll_rate);
        fb_msg.data.push_back(data.imu_pitch_rate);
        fb_msg.data.push_back(data.imu_yaw); 
        
        // Publish array ONLY AFTER all 13 elements are populated
        feedback_pub_->publish(fb_msg);

        // ---------------------------------------------------------
        // 2. Publish Standard IMU Message
        // ---------------------------------------------------------
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "base_link";
        
        imu_msg.angular_velocity.x = data.imu_roll_rate;
        imu_msg.angular_velocity.y = data.imu_pitch_rate;
        
        imu_pub_->publish(imu_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NucleoGatewayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
