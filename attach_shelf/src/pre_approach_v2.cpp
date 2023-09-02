#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp" // Include the header for the Trigger service
#include <cmath>

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode() : Node("obstacle_avoidance_node") {
    // Subscribe to the /scan topic
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ObstacleAvoidanceNode::scanCallback, this,
                  std::placeholders::_1));

    // Publish to the /robot/cmd_vel topic
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    approach_shelf_client_ =
        this->create_client<std_srvs::srv::Trigger>("/approach_shelf");
    // Set the linear velocity to move the robot forward
    linear_velocity_ =
        0.2; // You can adjust this value to your desired velocity
    angular_velocity_ =
        0.5; // You can adjust this value to your desired angular velocity
  }

private:
  void callApproachShelfService() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = approach_shelf_client_->async_send_request(request);

    // Wait for the response from the service
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(),
                    "Approach to shelf triggered successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to trigger approach to shelf.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Service call to approach_shelf timed out.");
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Find the minimum distance in the laser scan data
    float min_distance = std::numeric_limits<float>::infinity();
    for (const auto &range : scan->ranges) {
      if (range < min_distance) {
        min_distance = range;
      }
    }

    // Check if the minimum distance is less than or equal to the obstacle
    // distance (x) If yes, stop the robot and initiate rotation
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
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr approach_shelf_client_;
  float linear_velocity_;
  float angular_velocity_;
  float obstacle_distance_ = 1.0; // Set the obstacle distance (x) in meters
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
