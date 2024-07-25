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
    
    // Publish the initial joint state immediately upon node startup
    publish_initial_state();

    // Optionally continue publishing at regular intervals to ensure the state is maintained
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&InitialJointStatePublisher::publish_initial_state, this));

    RCLCPP_INFO(this->get_logger(), "Initial Joint State Publisher has been started.");
  }

private:
  void publish_initial_state()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    message.position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; 

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published initial joint state");
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
