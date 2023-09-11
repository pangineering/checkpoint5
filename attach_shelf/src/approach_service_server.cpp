#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    // Create the approach service
    approach_service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::handleApproachService, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Subscribe to the laser scan topic
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::laserCallback, this,
                  std::placeholders::_1));

    // Create the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void handleApproachService(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    // TODO: Implement the approach behavior here
    // This is where you should detect the shelf legs and execute the approach
    // motion.

    // Detect shelf legs using laser intensities
    bool legs_detected = detectShelfLegs();

    if (legs_detected) {
      // Calculate the center point between both legs and set it as the target
      // position for the robot's approach.
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id =
          "world_frame"; // Replace with appropriate frame ID
      transform.child_frame_id =
          "cart_frame"; // Replace with appropriate frame ID

      // Calculate the center point between the detected legs
      // Replace these with actual calculations based on your setup
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;

      transform.transform.rotation.x =
          0.0; // Replace with actual quaternion values
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      // Send the transform
      tf_broadcaster_->sendTransform(transform);

      // Set the response to indicate success
      response->complete = true;

      // TODO: Implement the logic to move the robot towards the shelf using the
      // transform coordinates. You can use the TF data to navigate the robot.
      // Once it reaches the target position, move forward 30 cm more.

    } else {
      // Set the response to indicate failure
      response->complete = false;
    }
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Implement the laser intensity-based shelf leg detection here
    // You can use the scan data to detect shelf legs based on laser
    // intensities. Once legs are detected, you can trigger the approach
    // behavior.

    if (detectShelfLegs()) {
      // Shelf legs are detected, trigger the approach behavior
      // You can implement this logic here or call a separate function.
      // For example: executeApproachBehavior();
    }
  }

  bool detectShelfLegs() {
    // Implement your shelf leg detection logic using laser intensities here
    // You may need to process the scan data to identify shelf legs.
    // Return true if legs are detected, otherwise false.

    // Placeholder logic for demonstration purposes (replace with actual
    // detection)
    float min_intensity = std::numeric_limits<float>::max();
    for (const float intensity : latest_scan_->intensities) {
      if (intensity < min_intensity) {
        min_intensity = intensity;
      }
    }

    // If the minimum intensity is below a threshold, consider legs detected
    const float intensity_threshold = 0.5; // Adjust as needed
    return min_intensity < intensity_threshold;
  }

  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
