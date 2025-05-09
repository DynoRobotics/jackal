#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class WorldOdomTfPub : public rclcpp::Node
{
  public:
    WorldOdomTfPub()
    : Node("world_odom_tf_pub"), count_(0)
    {
      latest_pose_ = std::make_shared<nav_msgs::msg::Odometry>();
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      true_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "jackal/odom", 10,
        std::bind(&WorldOdomTfPub::true_pose_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(50ms, std::bind(&WorldOdomTfPub::tf_broadcast, this));
    }

  private:
    void tf_broadcast(){
      if (latest_pose_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "No pose data received yet.");
        return;
      }
      geometry_msgs::msg::TransformStamped t;

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world"; // should this be world or jackal/odom?
      t.child_frame_id = "jackal/base_link";

      // Turtle only exists in 2D, thus we get x and y translation
      // coordinates from the message and set the z coordinate to 0
      t.transform.translation.x = latest_pose_->pose.pose.position.x;
      t.transform.translation.y = latest_pose_->pose.pose.position.y;
      t.transform.translation.z = latest_pose_->pose.pose.position.z;

      // For the same reason, turtle can only rotate around one axis
      // and this why we set rotation in x and y to 0 and obtain
      // rotation in z axis from the message
      t.transform.rotation.x = latest_pose_->pose.pose.orientation.x;
      t.transform.rotation.y = latest_pose_->pose.pose.orientation.y;
      t.transform.rotation.z = latest_pose_->pose.pose.orientation.z;
      t.transform.rotation.w = latest_pose_->pose.pose.orientation.w;

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    void true_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      RCLCPP_DEBUG(this->get_logger(), "Received true pose: '%s'", msg->header.frame_id.c_str());
      // Log the true pose data
      RCLCPP_DEBUG(this->get_logger(), "Position: [%.3f, %.3f, %.3f]",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
      
      // save the latest message
      latest_pose_ = msg;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr true_pose_sub_;
    nav_msgs::msg::Odometry::SharedPtr latest_pose_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldOdomTfPub>());
  rclcpp::shutdown();
  return 0;
}