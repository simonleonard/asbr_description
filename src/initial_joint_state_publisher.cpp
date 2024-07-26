#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class InitialJointStatePublisher : public rclcpp::Node {
public:
    InitialJointStatePublisher()
        : Node("initial_joint_state_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        publish_initial_state();
    }

private:
    void publish_initial_state() {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        message.position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; 
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published initial joint state.");

        // Stop publishing after setting the initial state
        rclcpp::shutdown();
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialJointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
