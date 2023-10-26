#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

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
    elevator_up_publisher_ =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
  }

private:
  void handleApproachService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Response>
          response) {
    // Check if the laser detected the shelf legs
    if (laser_detected_legs_) {
      RCLCPP_INFO(this->get_logger(), "attach_to_shelf: %s",
                  request->attach_to_shelf ? "true" : "false");
      // Perform the final approach logic

      // Calculate the center point between both detected legs
      double center_x = (leg1_x_ + leg2_x_) / 2.0;
      double center_y = (leg1_y_ + leg2_y_) / 2.0;

      // Publish a transform from the base frame to the cart_frame
      geometry_msgs::msg::TransformStamped cart_transform;
      cart_transform.header.frame_id =
          "robot_front_laser_base_link";            // Source frame
      cart_transform.child_frame_id = "cart_frame"; // Target frame
      cart_transform.transform.translation.x = center_x;
      cart_transform.transform.translation.y = center_y;
      cart_transform.transform.translation.z =
          0.0; // Assuming shelf height is at the same height as the robot's
               // base
      cart_transform.transform.rotation.x = 0.0;
      cart_transform.transform.rotation.y = 0.0;
      cart_transform.transform.rotation.z = 0.0;
      cart_transform.transform.rotation.w = 1.0; // No rotation

      tf_broadcaster_->sendTransform(cart_transform);

      // Debug print after sending the transform
      RCLCPP_INFO(this->get_logger(),
                  "After sending transform: x=%.2f, y=%.2f, z=%.2f, qx=%.2f, "
                  "qy=%.2f, qz=%.2f, qw=%.2f",
                  cart_transform.transform.translation.x,
                  cart_transform.transform.translation.y,
                  cart_transform.transform.translation.z,
                  cart_transform.transform.rotation.x,
                  cart_transform.transform.rotation.y,
                  cart_transform.transform.rotation.z,
                  cart_transform.transform.rotation.w);

      // Check if the service should do the final approach
      if (request->attach_to_shelf) {
        // Calculate the distance to move forward to be right underneath the
        // shelf
        double forward_distance = 0.3; // 30 cm

        // Move the robot towards the cart_frame

        moveTowardsTransform(cart_transform.transform.translation.x,
                             cart_transform.transform.translation.y);
        moveTowardsTransform(cart_transform.transform.translation.x,
                             cart_transform.transform.translation.y);
        moveTowardsTransform(cart_transform.transform.translation.x,
                             cart_transform.transform.translation.y);
        moveTowardsTransform(cart_transform.transform.translation.x,
                             cart_transform.transform.translation.y);
        // Move forward by the specified distance
        RCLCPP_INFO(this->get_logger(), "Done TF");
        // moveForward(forward_distance);
        while (forward_distance > 0) {
          // Calculate the distance to move in this step (up to a maximum of
          // forward_distance)
          double step_distance =
              std::min(0.1, forward_distance); // Adjust step distance as needed

          // Move the robot forward while maintaining the adjusted orientation
          geometry_msgs::msg::Twist forward_cmd_vel_msg;
          forward_cmd_vel_msg.linear.x =
              0.1; // Example linear velocity for forward motion
          forward_cmd_vel_msg.angular.z =
              0.0; // Example angular velocity (no rotation)

          // Publish the Twist message to control the robot's movement
          cmd_vel_publisher_->publish(forward_cmd_vel_msg);

          // Update the remaining distance to move forward
          forward_distance -= step_distance;

          // Sleep for a short duration (you may need to adjust this)
          rclcpp::sleep_for(
              std::chrono::milliseconds(100)); // Example sleep duration
        }
        geometry_msgs::msg::Twist forward_cmd_vel_msg;
        forward_cmd_vel_msg.linear.x =
            0.0; // Example linear velocity for forward motion
        forward_cmd_vel_msg.angular.z =
            0.0; // Example angular velocity (no rotation)

        // Publish the Twist message to control the robot's movement
        cmd_vel_publisher_->publish(forward_cmd_vel_msg);
        RCLCPP_INFO(this->get_logger(), "Done move forward");
        rclcpp::sleep_for(
            std::chrono::milliseconds(200)); // Example sleep duration
        loadShelf();
        rclcpp::sleep_for(
            std::chrono::milliseconds(100)); // Example sleep duration
        response->complete = true;
        RCLCPP_INFO(this->get_logger(), "True");

      } else {
        // Only publish the cart_frame transform, do not perform the final
        // approach
        response->complete = true;
        RCLCPP_INFO(this->get_logger(),
                    "Only publish the cart_frame transform, do not perform the "
                    "final approach");
      }
    } else {
      // Laser did not detect both shelf legs
      response->complete = false;
      RCLCPP_INFO(this->get_logger(), "Laser did not detect both shelf legs");
    }

    // Avoid unused parameter warning
    (void)request_header;
  }

  bool moveTowardsTransform(double target_x, double target_y) {
    // Calculate the distance between the current robot position and the target
    // position
    double distance_to_target =
        sqrt(pow(target_x - robot_x_, 2) + pow(target_y - robot_y_, 2));

    // Calculate the angle to the target position
    double angle_to_target = atan2(target_y - robot_y_, target_x - robot_x_);

    // Implement your robot's control mechanisms to adjust its orientation
    // and move towards the target position

    // Placeholder logic for demonstration purposes (replace with actual
    // control code)
    // This example assumes that the robot moves directly towards the target
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.5;  // Adjust linear velocity as needed
    cmd_vel_msg.angular.z = 0.0; // No rotation

    // Publish the Twist message to control the robot's movement
    cmd_vel_publisher_->publish(cmd_vel_msg);

    // Wait for the robot to reach the target position (you may need to
    // implement feedback control)
    rclcpp::sleep_for(
        std::chrono::milliseconds(1000)); // Example sleep duration

    // Stop the robot
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_msg);

    // Update the robot's position
    robot_x_ = target_x;
    robot_y_ = target_y;
    RCLCPP_INFO(this->get_logger(), "Move");

    // return true; // The robot successfully moved towards the target
  }

  bool moveForward(double forward_distance) {
    // ... (previous code)

    // Calculate the time needed to move the specified distance
    std::chrono::nanoseconds time_to_move_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(time_to_move_ns));
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.1;
    // Publish the Twist message to control the robot's movement
    cmd_vel_publisher_->publish(cmd_vel_msg);

    // Wait for the robot to move the specified distance
    rclcpp::sleep_for(time_to_move_ns);

    while (forward_distance > 0) {
      // Calculate the distance to move in this step (up to a maximum of
      // forward_distance)
      double step_distance =
          std::min(0.1, forward_distance); // Adjust step distance as needed

      // Move the robot forward while maintaining the adjusted orientation
      geometry_msgs::msg::Twist forward_cmd_vel_msg;
      forward_cmd_vel_msg.linear.x =
          0.2; // Example linear velocity for forward motion
      forward_cmd_vel_msg.angular.z =
          0.0; // Example angular velocity (no rotation)

      // Publish the Twist message to control the robot's movement
      cmd_vel_publisher_->publish(forward_cmd_vel_msg);

      // Update the remaining distance to move forward
      forward_distance -= step_distance;

      // Sleep for a short duration (you may need to adjust this)
      rclcpp::sleep_for(
          std::chrono::milliseconds(100)); // Example sleep duration
    }

    // Stop the robot after moving forward by the desired distance
    geometry_msgs::msg::Twist stop_cmd_vel_msg;
    stop_cmd_vel_msg.linear.x = 0.0;
    stop_cmd_vel_msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop_cmd_vel_msg);
    return true; // The robot successfully moved forward by the specified
                 // distance
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

  void loadShelf() {
    // Trigger the elevator up action by publishing a message to /elevator_up
    auto elevator_up_msg = std_msgs::msg::Empty();
    elevator_up_publisher_->publish(elevator_up_msg);
    RCLCPP_INFO(this->get_logger(), "Triggered elevator up action.");
  }

  // Member variables
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;
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
