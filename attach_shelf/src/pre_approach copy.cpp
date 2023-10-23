#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode(double obstacle_distance, double degrees)
      : Node("obstacle_avoidance_node"), obstacle_distance_(obstacle_distance),
        degrees_(degrees), stop_rotation_(false), aligning_to_shelf_(false),
        task_done_(false) {

    // Subscribe to the /scan topic
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ObstacleAvoidanceNode::scanCallback, this,
                  std::placeholders::_1));

    // Subscribe to the /odom topic to get the robot's yaw angle
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ObstacleAvoidanceNode::odometryCallback, this,
                  std::placeholders::_1));

    // Publish to the /robot/cmd_vel topic
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    // Set the linear velocity to move the robot forward
    linear_velocity_ =
        0.2; // You can adjust this value to your desired velocity

    // Create a timer to periodically check for obstacles and control the robot
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObstacleAvoidanceNode::timerCallback, this));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Find the minimum distance in the laser scan data
    float min_distance = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float range = scan->ranges[i];

      // Only consider the front laser ray (adjust the angle range as needed)
      if (i >= scan->ranges.size() / 2 - 10 &&
          i <= scan->ranges.size() / 2 + 10) {
        if (range < min_distance) {
          min_distance = range;
        }
      }
    }
    // Convert degrees to radians
    double degrees_in_radians = degrees_ * M_PI / 180.0;
    if (min_distance <= obstacle_distance_ && !stop_rotation_) {
      // Obstacle detected, switch to aligning_to_shelf_ mode
      aligning_to_shelf_ = true;
      stop_rotation_ = true; // Stop rotating

      // Stop the robot and align to the shelf
      stopRobot();
      alignToShelf();
    } else if (aligning_to_shelf_ &&
               std::abs(yaw_ - degrees_in_radians) > 0.05) {
      // Continue aligning to the shelf until the desired angle is reached
      RCLCPP_INFO(this->get_logger(), "Degree angle: %f", degrees_);
      alignToShelf();
    } else if (!stop_rotation_ && !task_done_) {
      // If the robot is not already stopped and not aligning to the shelf, move
      // it forward
      moveRobotForward();
    }
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    // Extract the orientation quaternion from the odometry data
    geometry_msgs::msg::Quaternion orientation = odom->pose.pose.orientation;

    // Calculate the yaw angle (rotation about the robot's vertical axis) from
    // the quaternion
    yaw_ = std::atan2(
        2.0 * (orientation.x * orientation.y + orientation.z * orientation.w),
        1.0 - 2.0 * (orientation.y * orientation.y +
                     orientation.z * orientation.z));

    // Ensure that the yaw angle is within the range [-pi, pi]
    if (yaw_ > M_PI) {
      yaw_ -= 2.0 * M_PI;
    } else if (yaw_ < -M_PI) {
      yaw_ += 2.0 * M_PI;
    }

    // Print the yaw angle to the console for debugging
    RCLCPP_INFO(this->get_logger(), "Yaw angle: %f", yaw_);
  }

  void timerCallback() {
    // This function is called periodically by the timer.
    // You can perform more complex or time-consuming operations here if needed.
    // This function runs in a separate timer thread.
  }

  void moveRobotForward() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_velocity_;
    twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist);
  }

  void stopRobot() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist);
  }

  void alignToShelf() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;

    // Convert degrees to radians
    double degrees_in_radians = degrees_ * M_PI / 180.0;
    RCLCPP_INFO(this->get_logger(), "Degree angle: %f", degrees_);
    RCLCPP_INFO(this->get_logger(), "Degree angle (Radian): %f",
                degrees_in_radians);

    // Calculate the absolute difference between the current yaw angle and the
    // desired angle
    double angle_difference = std::abs(yaw_ - degrees_in_radians);
    RCLCPP_INFO(this->get_logger(), "difference angle: %f", angle_difference);

    if (angle_difference > 0.05) {
      // Continue rotating towards the desired angle
      twist.angular.z = (degrees_in_radians > yaw_) ? 0.2 : -0.2;
    } else {
      // If the desired angle is reached (within a tolerance), stop rotation
      twist.angular.z = 0.0;
      aligning_to_shelf_ = false;
      stop_rotation_ = false;
      task_done_ = true;
    }

    cmd_vel_publisher_->publish(twist);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  double linear_velocity_;
  double obstacle_distance_;
  double degrees_;
  bool stop_rotation_;
  bool aligning_to_shelf_;
  bool task_done_;
  double yaw_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<rclcpp::Node>("obstacle_avoidance_node");

  // Declare parameters and their default values
  double obstacle_distance = 0.3; // Default value
  double degrees = -90.0;         // Default value

  // Declare parameters with default values
  node->declare_parameter("obstacle", obstacle_distance);
  node->declare_parameter("degrees", degrees);

  // Get parameter values from the launch file or command line
  node->get_parameter("obstacle", obstacle_distance);
  node->get_parameter("degrees", degrees);

  // Create the ObstacleAvoidanceNode with parameter values
  auto obstacle_avoidance_node =
      std::make_shared<ObstacleAvoidanceNode>(obstacle_distance, degrees);

  // Spin the node
  rclcpp::spin(obstacle_avoidance_node);

  rclcpp::shutdown();
  return 0;
}
