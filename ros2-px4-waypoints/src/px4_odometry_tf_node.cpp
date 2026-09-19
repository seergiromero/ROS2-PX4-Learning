#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>

#include "ros2-px4-waypoints/frame_conversions.hpp"

using ros2_px4_waypoints::px4_frame_conversions::nedPositionToEnu;
using ros2_px4_waypoints::px4_frame_conversions::nedVelocityToBodyFlu;
using ros2_px4_waypoints::px4_frame_conversions::px4AttitudeToRos;
using ros2_px4_waypoints::px4_frame_conversions::bodyFrdVectorToFlu;

class Px4OdometryTf : public rclcpp::Node
{
public:
  Px4OdometryTf()
  : Node("px4_odometry_tf")
  {
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        publish(*msg);
      });
  }

private:
  void publish(const px4_msgs::msg::VehicleOdometry & msg)
  {
    if (msg.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
      return;
    }
    if (!ros2_px4_waypoints::px4_frame_conversions::allFinitePosition(
        msg.position.data()))
    {
      return;
    }
    if (!ros2_px4_waypoints::px4_frame_conversions::allFiniteQuaternion(
        msg.q.data()))
    {
      return;
    }

    const tf2::Vector3 position_enu = nedPositionToEnu(msg.position.data());
    const tf2::Quaternion base_to_enu = px4AttitudeToRos(msg.q.data());

    const auto stamp = now();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position_enu.x();
    transform.transform.translation.y = position_enu.y();
    transform.transform.translation.z = position_enu.z();
    transform.transform.rotation = toMessage(base_to_enu);
    tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = position_enu.x();
    odom.pose.pose.position.y = position_enu.y();
    odom.pose.pose.position.z = position_enu.z();
    odom.pose.pose.orientation = toMessage(base_to_enu);

    odom.pose.covariance[0] = msg.position_variance[0];
    odom.pose.covariance[7] = msg.position_variance[1];
    odom.pose.covariance[14] = msg.position_variance[2];

    if (msg.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
      const tf2::Vector3 velocity_body =
        nedVelocityToBodyFlu(msg.velocity.data(), base_to_enu);
      odom.twist.twist.linear.x = velocity_body.x();
      odom.twist.twist.linear.y = velocity_body.y();
      odom.twist.twist.linear.z = velocity_body.z();
    } else if (msg.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      const tf2::Vector3 velocity_body = bodyFrdVectorToFlu(msg.velocity.data());
      odom.twist.twist.linear.x = velocity_body.x();
      odom.twist.twist.linear.y = velocity_body.y();
      odom.twist.twist.linear.z = velocity_body.z();
    }

    const tf2::Vector3 angular_body_frd_to_flu =
      bodyFrdVectorToFlu(msg.angular_velocity.data());
    odom.twist.twist.angular.x = angular_body_frd_to_flu.x();
    odom.twist.twist.angular.y = angular_body_frd_to_flu.y();
    odom.twist.twist.angular.z = angular_body_frd_to_flu.z();
    odom_pub_->publish(odom);
  }

  static geometry_msgs::msg::Quaternion toMessage(const tf2::Quaternion & q)
  {
    geometry_msgs::msg::Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();
    return result;
  }

  std::string odom_frame_;
  std::string base_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdometryTf>());
  rclcpp::shutdown();
  return 0;
}
