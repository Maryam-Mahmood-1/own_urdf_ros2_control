#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
    : Node("joint_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this)); 
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        
        // Add names of the 7 joints
        message.joint_names.push_back("joint_1");
        message.joint_names.push_back("joint_2");
        message.joint_names.push_back("joint_3");
        message.joint_names.push_back("joint_4");
        message.joint_names.push_back("joint_5");
        message.joint_names.push_back("joint_6");
        message.joint_names.push_back("joint_7");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();

        // Set positions for each joint with a simple alternating pattern
        for (size_t i = 0; i < 7; ++i) {
            double position = 0.75 * (1 - cos(count_ * 0.2 + i * M_PI / 7));  // Varying each joint slightly
            point.positions.push_back(position);
        }

        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing positions for 7 joints at count: %zu", count_);
        count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}
