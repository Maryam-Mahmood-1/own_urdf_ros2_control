#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>
#include <vector>

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
    : Node("joint_publisher"), count_(0), joint_index_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        
        // Initialize all joint positions to 0.0
        current_positions_ = std::vector<double>(1, 0.0);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000), std::bind(&JointPublisher::timer_callback, this)); 
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();

        // Add names of the 7 joints
        message.joint_names.push_back("joint_1");
        

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();

        // Update only the selected joint with its cosine movement, keep others unchanged
        for (size_t i = 0; i < 1; ++i) {
            if (i == joint_index_) {
                // Calculate the position for the current joint
                double new_position = 0.66 * (1 - cos(count_ * 0.2 + i * M_PI / 7));

                // If joint_6 (index 5), invert the position
                if (i == 5) {
                    new_position = -new_position;
                }

                // Update the stored position for the current joint
                current_positions_[i] = new_position;
                RCLCPP_INFO(this->get_logger(), "Moving joint_%zu to position: %f", i + 1, new_position);
            }
            // Push the current (updated or unchanged) position for each joint
            point.positions.push_back(current_positions_[i]);
        }

        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);
        publisher_->publish(message);

        // Move to the next joint for the next callback
        joint_index_ = (joint_index_ + 1) % 7;
        count_ += 1;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
    size_t joint_index_;  // To keep track of which joint to move
    std::vector<double> current_positions_;  // Stores the current position of each joint
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}
