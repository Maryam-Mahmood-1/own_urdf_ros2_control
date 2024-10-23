#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>
#include <vector>
#include <random>

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
    : Node("joint_publisher"), count_(0), joint_index_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // Initialize joint names and positions
        joint_names_ = {"joint_1", "joint_2"};  // Define the names of the joints
        current_positions_ = std::vector<double>(joint_names_.size(), 0.0);

        // Set a timer to update the joint position every 2 seconds
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000), std::bind(&JointPublisher::timer_callback, this)); 

        // Seed the random number generator
        rng_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
        // Define the range for random positions (e.g., -1.0 to 1.0 radians)
        distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = joint_names_;  // Set the joint names in the message

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        
        // Generate random positions for both joints
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // Generate a new random position
            double new_position = distribution_(rng_);
            current_positions_[i] = new_position;  // Update the position for the current joint

            // Log the movement
            RCLCPP_INFO(this->get_logger(), "Moving %s to position: %f", joint_names_[i].c_str(), new_position);
            
            // Push the updated position for each joint
            point.positions.push_back(current_positions_[i]);
        }

        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
    size_t joint_index_;  // To keep track of which joint to move
    std::vector<std::string> joint_names_;  // List of joint names
    std::vector<double> current_positions_;  // Stores the current position of each joint

    // Random number generation
    std::default_random_engine rng_;
    std::uniform_real_distribution<double> distribution_;  // Distribution for generating random positions
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}
