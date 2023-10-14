#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3));

    // Subscribe to the laser scan topic
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::laserCallback, this,
                  std::placeholders::_1));

    // Create the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    // Initialize robot's position
    robot_x_ = 0.0;
    robot_y_ = 0.0;

    // Initialize latest_scan_ as a null SharedPtr
    latest_scan_ = nullptr;
  }

private:
  void handleApproachService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Response>
          response) {
    // Check if the laser detected the shelf legs
    if (laser_detected_legs_) {
      // Check if the service should do the final approach
      if (request->attach_to_shelf) {
        // Perform the final approach logic here
        // Publish the cart_frame transform, move the robot underneath the
        // shelf, and lift it If successful, set response->complete to true
        // Otherwise, set response->complete to false
        // Implement this logic according to your robot's control mechanisms
        // You can use the tf_broadcaster_ and cmd_vel_publisher_ to achieve
        // this

        // Example (replace with your actual logic):
        // tf_broadcaster_->sendTransform(/* transform details */);
        // geometry_msgs::msg::Twist cmd_vel_msg;
        // cmd_vel_msg.linear.x = /* control command */;
        // cmd_vel_msg.angular.z = /* control command */;
        // cmd_vel_publisher_->publish(cmd_vel_msg);

        bool final_approach_successful = true; // Replace with your logic

        if (final_approach_successful) {
          response->complete = true;
        } else {
          response->complete = false;
        }
      } else {
        // Only publish the cart_frame transform, do not perform the final
        // approach
        // Set response->complete to true

        // Example (replace with your actual logic):
        // tf_broadcaster_->sendTransform(/* transform details */);
        response->complete = true;
      }
    } else {
      // Laser did not detect shelf legs
      // Set response->complete to false
      response->complete = false;
    }

    // Avoid unused parameter warning
    (void)request_header;
  }

  bool detectShelfLegs() {
    // Implement your shelf leg detection logic using laser intensities here
    // You may need to process the scan data to identify shelf legs.
    // Return true if both legs are detected, otherwise false.

    // Placeholder logic for demonstration purposes (replace with actual
    // detection)
    float min_intensity = std::numeric_limits<float>::max();
    if (latest_scan_) {
      for (size_t i = 0; i < latest_scan_->intensities.size(); ++i) {
        if (latest_scan_->intensities[i] < min_intensity) {
          min_intensity = latest_scan_->intensities[i];
          if (min_intensity < intensity_threshold_) {
            // Update the positions of detected legs based on their angles
            // Replace these calculations with your actual logic
            leg1_x_ = latest_scan_->ranges[i] *
                      std::cos(latest_scan_->angle_min +
                               i * latest_scan_->angle_increment);
            leg1_y_ = latest_scan_->ranges[i] *
                      std::sin(latest_scan_->angle_min +
                               i * latest_scan_->angle_increment);
            // Find the second leg
            // You may need additional logic to confirm the second leg detection
            // Replace these calculations with your actual logic
            leg2_x_ = latest_scan_->ranges[i] *
                      std::cos(latest_scan_->angle_min +
                               i * latest_scan_->angle_increment);
            leg2_y_ = latest_scan_->ranges[i] *
                      std::sin(latest_scan_->angle_min +
                               i * latest_scan_->angle_increment);
            return true; // Both legs detected
          }
        }
      }
    }

    return false; // Legs not detected
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the received laser scan message
    latest_scan_ = msg;

    // Implement logic to analyze laser intensity values and set
    // laser_detected_legs_ accordingly
    // Example: if (laser intensity values indicate legs are detected) {
    //   laser_detected_legs_ = true;
    // } else {
    //   laser_detected_legs_ = false;
    // }

    // Placeholder logic for demonstration purposes (replace with actual logic)
    laser_detected_legs_ = detectShelfLegs();
  }

  // Member variables
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  double robot_x_;
  double robot_y_;
  bool laser_detected_legs_ = false;
  double intensity_threshold_ = 0.5; // Adjust this threshold as needed
  double leg1_x_, leg1_y_, leg2_x_, leg2_y_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
