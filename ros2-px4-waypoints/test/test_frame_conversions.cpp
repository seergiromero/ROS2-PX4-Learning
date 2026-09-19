#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>

#include "ros2-px4-waypoints/frame_conversions.hpp"

namespace
{

using ros2_px4_waypoints::px4_frame_conversions::aircraftToBaselinkRotation;
using ros2_px4_waypoints::px4_frame_conversions::bodyFrdVectorToFlu;
using ros2_px4_waypoints::px4_frame_conversions::nedPositionToEnu;
using ros2_px4_waypoints::px4_frame_conversions::nedToEnuRotation;
using ros2_px4_waypoints::px4_frame_conversions::nedVelocityToBodyFlu;
using ros2_px4_waypoints::px4_frame_conversions::nedVelocityToEnu;
using ros2_px4_waypoints::px4_frame_conversions::px4AttitudeToRos;

constexpr double kEps = 1e-9;

bool vectorsClose(
  const tf2::Vector3 & a, const tf2::Vector3 & b, double eps = kEps)
{
  return std::abs(a.x() - b.x()) < eps &&
         std::abs(a.y() - b.y()) < eps &&
         std::abs(a.z() - b.z()) < eps;
}

TEST(NedToEnuPosition, SwapsXyAndFlipsZ)
{
  // PX4 NED: 1 m north, 2 m east, -3 m up.
  const float ned[3] = {1.0f, 2.0f, -3.0f};
  const tf2::Vector3 enu = nedPositionToEnu(ned);

  // ROS ENU: 2 m east (x), 1 m north (y), 3 m up (z).
  EXPECT_TRUE(vectorsClose(enu, tf2::Vector3(2.0, 1.0, 3.0)));
}

TEST(NedToEnuPosition, OriginStaysOrigin)
{
  const float ned[3] = {0.0f, 0.0f, 0.0f};
  EXPECT_TRUE(vectorsClose(nedPositionToEnu(ned), tf2::Vector3(0.0, 0.0, 0.0)));
}

TEST(NedToEnuPosition, NegativeAltitudeBecomesPositiveZ)
{
  const float ned[3] = {0.0f, 0.0f, -10.0f};
  const tf2::Vector3 enu = nedPositionToEnu(ned);
  EXPECT_NEAR(enu.z(), 10.0, kEps);
}

TEST(NedToEnuRotation, MapsUnitVectorsToEnuBasis)
{
  const tf2::Quaternion q = nedToEnuRotation();

  // NED north axis -> ENU y.
  EXPECT_TRUE(vectorsClose(tf2::quatRotate(q, tf2::Vector3(1, 0, 0)),
    tf2::Vector3(0, 1, 0)));
  // NED east axis -> ENU x.
  EXPECT_TRUE(vectorsClose(tf2::quatRotate(q, tf2::Vector3(0, 1, 0)),
    tf2::Vector3(1, 0, 0)));
  // NED down axis -> ENU -z.
  EXPECT_TRUE(vectorsClose(tf2::quatRotate(q, tf2::Vector3(0, 0, 1)),
    tf2::Vector3(0, 0, -1)));
}

TEST(AircraftToBaselinkRotation, MapsFrdToFlu)
{
  const tf2::Quaternion q = aircraftToBaselinkRotation();

  // Forward stays forward.
  EXPECT_TRUE(vectorsClose(tf2::quatRotate(q, tf2::Vector3(1, 0, 0)),
    tf2::Vector3(1, 0, 0)));
  // FRD right (+y) has FLU coordinates (0, -1, 0): same physical direction,
  // expressed in a frame whose y axis is "left".
  EXPECT_TRUE(vectorsClose(tf2::quatRotate(q, tf2::Vector3(0, 1, 0)),
    tf2::Vector3(0, -1, 0)));
  // FRD down (+z) has FLU coordinates (0, 0, -1): same physical direction,
  // expressed in a frame whose z axis is "up".
  EXPECT_TRUE(vectorsClose(tf2::quatRotate(q, tf2::Vector3(0, 0, 1)),
    tf2::Vector3(0, 0, -1)));
}

TEST(Px4AttitudeToRos, IdentityQuaternionMeansHeadingNorth)
{
  // PX4 identity attitude: aircraft FRD aligned with NED -> heading north.
  const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  const tf2::Quaternion base_to_enu = px4AttitudeToRos(q);

  // Forward (FLU x) must point towards ENU +y (north).
  const tf2::Vector3 forward = tf2::quatRotate(base_to_enu, tf2::Vector3(1, 0, 0));
  EXPECT_TRUE(vectorsClose(forward, tf2::Vector3(0, 1, 0), 1e-6));
  // Up (FLU z) must point towards ENU +z.
  const tf2::Vector3 up = tf2::quatRotate(base_to_enu, tf2::Vector3(0, 0, 1));
  EXPECT_TRUE(vectorsClose(up, tf2::Vector3(0, 0, 1), 1e-6));
}

TEST(Px4AttitudeToRos, EastHeadingTurnsForwardToEnuX)
{
  // PX4 yaw of +90 degrees rotates north towards east about the NED down
  // axis: q = [cos(45deg), 0, 0, sin(45deg)], body -> NED convention.
  const float q[4] = {
    static_cast<float>(std::cos(M_PI_4)), 0.0f, 0.0f,
    static_cast<float>(std::sin(M_PI_4))};
  const tf2::Quaternion base_to_enu = px4AttitudeToRos(q);

  const tf2::Vector3 forward = tf2::quatRotate(base_to_enu, tf2::Vector3(1, 0, 0));
  EXPECT_TRUE(vectorsClose(forward, tf2::Vector3(1, 0, 0), 1e-6));
}

TEST(NedVelocityToBodyFlu, NorthVelocityOnNorthFacingVehicleIsForward)
{
  const float velocity_ned[3] = {1.0f, 0.0f, 0.0f};  // 1 m/s north.
  const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  const tf2::Quaternion base_to_enu = px4AttitudeToRos(identity);

  const tf2::Vector3 body = nedVelocityToBodyFlu(velocity_ned, base_to_enu);
  EXPECT_TRUE(vectorsClose(body, tf2::Vector3(1, 0, 0)));
}

TEST(NedVelocityToBodyFlu, EastVelocityOnNorthFacingVehicleIsLeftward)
{
  const float velocity_ned[3] = {0.0f, 1.0f, 0.0f};  // 1 m/s east.
  const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  const tf2::Quaternion base_to_enu = px4AttitudeToRos(identity);

  const tf2::Vector3 body = nedVelocityToBodyFlu(velocity_ned, base_to_enu);
  EXPECT_TRUE(vectorsClose(body, tf2::Vector3(0, -1, 0)));
}

TEST(BodyFrdVectorToFlu, RightAndDownSignsFlip)
{
  const float frd[3] = {1.0f, 2.0f, 3.0f};
  const tf2::Vector3 flu = bodyFrdVectorToFlu(frd);
  EXPECT_TRUE(vectorsClose(flu, tf2::Vector3(1.0, -2.0, -3.0)));
}

TEST(NedVelocityToEnu, SwapsXyAndFlipsZ)
{
  const float ned[3] = {0.5f, -1.5f, 2.0f};
  const tf2::Vector3 enu = nedVelocityToEnu(ned);
  EXPECT_TRUE(vectorsClose(enu, tf2::Vector3(-1.5, 0.5, -2.0)));
}

}  // namespace
