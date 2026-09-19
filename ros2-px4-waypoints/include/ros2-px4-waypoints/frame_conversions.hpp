#ifndef ROS2_PX4_WAYPOINTS__FRAME_CONVERSIONS_HPP_
#define ROS2_PX4_WAYPOINTS__FRAME_CONVERSIONS_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <array>
#include <cmath>

namespace ros2_px4_waypoints
{
namespace px4_frame_conversions
{

inline bool allFinite(const std::array<float, 3> & values)
{
  for (const float value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

inline bool allFinite(const std::array<float, 4> & values)
{
  for (const float value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

inline bool allFinitePosition(const float * values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) &&
         std::isfinite(values[2]);
}

inline bool allFiniteQuaternion(const float * values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) &&
         std::isfinite(values[2]) && std::isfinite(values[3]);
}

inline tf2::Quaternion nedToEnuRotation()
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, M_PI_2);
  return q;
}

inline tf2::Quaternion aircraftToBaselinkRotation()
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  return q;
}

inline tf2::Vector3 nedPositionToEnu(const float * position)
{
  return tf2::Vector3(position[1], position[0], -position[2]);
}

inline tf2::Vector3 nedVelocityToEnu(const float * velocity)
{
  return tf2::Vector3(velocity[1], velocity[0], -velocity[2]);
}

inline tf2::Quaternion px4AttitudeToRos(const float * q)
{
  const tf2::Quaternion aircraft_to_ned(q[1], q[2], q[3], q[0]);
  tf2::Quaternion base_to_enu =
    nedToEnuRotation() * aircraft_to_ned * aircraftToBaselinkRotation();
  base_to_enu.normalize();
  return base_to_enu;
}

inline tf2::Vector3 enuVelocityToBodyFlu(
  const tf2::Vector3 & velocity_enu, const tf2::Quaternion & base_to_enu)
{
  return tf2::quatRotate(base_to_enu.inverse(), velocity_enu);
}

inline tf2::Vector3 nedVelocityToBodyFlu(
  const float * velocity, const tf2::Quaternion & base_to_enu)
{
  return enuVelocityToBodyFlu(nedVelocityToEnu(velocity), base_to_enu);
}

inline tf2::Vector3 bodyFrdVectorToFlu(const float * values)
{
  return tf2::Vector3(values[0], -values[1], -values[2]);
}

}  // namespace px4_frame_conversions
}  // namespace ros2_px4_waypoints

#endif  // ROS2_PX4_WAYPOINTS__FRAME_CONVERSIONS_HPP_
