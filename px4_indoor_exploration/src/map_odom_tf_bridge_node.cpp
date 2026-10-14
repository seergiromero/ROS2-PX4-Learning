#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <mutex>
#include <string>

class MapOdomTfBridge : public rclcpp::Node
{
public:
  MapOdomTfBridge()
  : Node("map_odom_tf_bridge")
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("fast_lio_body_frame", "body");
    declare_parameter<std::string>("px4_base_frame", "base_link");
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    fast_lio_body_frame_ = get_parameter("fast_lio_body_frame").as_string();
    px4_base_frame_ = get_parameter("px4_base_frame").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // PX4 odometry: odom -> base_link.
    px4_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        px4_pose_ = toTfTransform(msg->pose.pose);
        px4_pose_valid_ = true;
      });

    // FAST-LIO odometry: map -> body.
    fast_lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!px4_pose_valid_) {
          return;
        }
        const tf2::Transform map_to_body = toTfTransform(msg->pose.pose);
        map_to_odom_ = map_to_body * px4_pose_.inverse();
        map_to_odom_valid_ = true;
        publishTransform(msg->header.stamp, map_to_odom_);
      });

    // Republish the latest map -> odom periodically so that RViz always has a
    // recent transform even when a message arrives with a slightly newer stamp.
    republish_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (map_to_odom_valid_) {
          publishTransform(now(), map_to_odom_);
        }
      });
  }

private:
  static tf2::Transform toTfTransform(const geometry_msgs::msg::Pose & pose)
  {
    tf2::Transform t;
    t.setOrigin(tf2::Vector3(
      pose.position.x, pose.position.y, pose.position.z));
    t.setRotation(tf2::Quaternion(
      pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w));
    return t;
  }

  void publishTransform(
    const builtin_interfaces::msg::Time & stamp,
    const tf2::Transform & map_to_odom)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_to_odom.getOrigin().x();
    transform.transform.translation.y = map_to_odom.getOrigin().y();
    transform.transform.translation.z = map_to_odom.getOrigin().z();
    const tf2::Quaternion q = map_to_odom.getRotation();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string fast_lio_body_frame_;
  std::string px4_base_frame_;

  std::mutex mutex_;
  tf2::Transform px4_pose_;
  bool px4_pose_valid_ = false;
  tf2::Transform map_to_odom_;
  bool map_to_odom_valid_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fast_lio_odom_sub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfBridge>());
  rclcpp::shutdown();
  return 0;
}
