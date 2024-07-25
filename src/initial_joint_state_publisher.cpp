#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class InitialJointStatePublisher : public rclcpp::Node
{
public:
  InitialJointStatePublisher()
  : Node("initial_joint_state_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&InitialJointStatePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    message.position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; // Set your desired initial positions here

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Initial joint states published. Shutting down node.");
    rclcpp::shutdown(); // Shutdown after publishing once
  }
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
