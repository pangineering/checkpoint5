#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
        // Subscribe to the /scan topic
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleAvoidanceNode::scanCallback, this, std::placeholders::_1));

        // Publish to the /robot/cmd_vel topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

        // Set the linear velocity to move the robot forward
        linear_velocity_ = 0.2; // You can adjust this value to your desired velocity
        angular_velocity_ = 0.5; // You can adjust this value to your desired angular velocity
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Find the minimum distance in the laser scan data
        float min_distance = std::numeric_limits<float>::infinity();
        for (const auto& range : scan->ranges) {
            if (range < min_distance) {
                min_distance = range;
            }
        }

        // Check if the minimum distance is less than or equal to the obstacle distance (x)
        // If yes, stop the robot and initiate rotation
        if (min_distance <= obstacle_distance_) {
            stopRobotAndRotate();
        } else {
            moveRobotForward();
        }
    }

    void moveRobotForward() {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear_velocity_;
        twist.angular.z = 0.0;
        cmd_vel_publisher_->publish(twist);
    }

    void stopRobotAndRotate() {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.0;
        twist.angular.z = angular_velocity_;
        cmd_vel_publisher_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    float linear_velocity_;
    float angular_velocity_;
    float obstacle_distance_ = 1.0; // Set the obstacle distance (x) in meters
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
