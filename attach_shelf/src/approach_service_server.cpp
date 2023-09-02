#include "attach_shelf/srv/go_to_loading.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    // Create the approach service
    approach_service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::handleApproachService, this,
                  std::placeholders::_1, std::placeholders::_2));

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

    bool success = false; // Set this based on the approach result
    if (success) {
      // Publish a transform to cart_frame (assuming you have calculated the
      // proper coordinates)
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id =
          "world_frame"; // Replace with appropriate frame ID
      transform.child_frame_id =
          "cart_frame"; // Replace with appropriate frame ID

      // Set the transform translation and rotation
      transform.transform.translation.x =
          0.0; // Replace with actual x-coordinate
      transform.transform.translation.y =
          0.0; // Replace with actual y-coordinate
      transform.transform.translation.z =
          0.0; // Replace with actual z-coordinate

      transform.transform.rotation.x =
          0.0; // Replace with actual quaternion values
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      // Send the transform
      tf_broadcaster_->sendTransform(transform);

      // Set the response to indicate success
      response->complete = true;
    } else {
      // Set the response to indicate failure
      response->complete = false;
    }
  }

  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr approach_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}