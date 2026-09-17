#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Waypoint
{
  float x;
  float y;
  float z;
};

class OffboardWaypoints : public rclcpp::Node
{
public:
  OffboardWaypoints() : Node("offboard_waypoints")
  {
    offboard_control_mode_pub_ =
      create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ =
      create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ =
      create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    local_position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleLocalPosition::SharedPtr msg) {
        local_position_ = *msg;
      });

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", rclcpp::SensorDataQoS(),
      [this](const VehicleStatus::SharedPtr msg) {
        status_ = *msg;
      });

    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleCommandAck::SharedPtr msg) {
        RCLCPP_INFO(
          get_logger(), "ACK command=%u result=%u",
          msg->command, msg->result);
      });

    timer_ = create_wall_timer(
      50ms, std::bind(&OffboardWaypoints::timer_callback, this));
  }

private:
  enum class MissionState
  {
    STREAMING,
    WAIT_OFFBOARD,
    WAIT_ARMED,
    TAKEOFF,
    WAYPOINTS,
    RETURN_HOME,
    LAND,
    FINISHED,
    ERROR
  };

  void timer_callback()
  {
    if (state_ == MissionState::ERROR) {
      return;
    }

    // OffboardControlMode must be paired with a trajectory setpoint,
    // and it must be published continuously. This is the heartbeat.
    publish_offboard_control_mode();

    switch (state_) {
      case MissionState::STREAMING:
        // Keep streaming an initial (safe) setpoint until PX4 knows us.
        publish_trajectory_setpoint(0.0f, 0.0f, -2.0f);
        streamed_setpoints_++;
        if (streamed_setpoints_ >= 10) {
          // 1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 = PX4 offboard mode
          publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          RCLCPP_INFO(get_logger(), "Offboard mode requested (command 176)");
          state_ = MissionState::WAIT_OFFBOARD;
        }
        break;

      case MissionState::WAIT_OFFBOARD:
        // Keep streaming; wait until PX4 actually accepts offboard.
        publish_trajectory_setpoint(0.0f, 0.0f, -2.0f);
        if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          RCLCPP_INFO(get_logger(), "Offboard accepted, arm command sent");
          state_ = MissionState::WAIT_ARMED;
        }
        break;

      case MissionState::WAIT_ARMED:
        publish_trajectory_setpoint(0.0f, 0.0f, -2.0f);
        if (status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(get_logger(), "Armed, taking off");
          state_ = MissionState::TAKEOFF;
        }
        break;

      case MissionState::TAKEOFF:
        publish_trajectory_setpoint(0.0f, 0.0f, takeoff_z_);
        if (at_target(0.0f, 0.0f, takeoff_z_)) {
          RCLCPP_INFO(get_logger(), "Takeoff complete, flying waypoints");
          waypoint_index_ = 0;
          state_ = MissionState::WAYPOINTS;
        }
        break;

      case MissionState::WAYPOINTS:
        if (waypoint_index_ >= waypoints_.size()) {
          state_ = MissionState::RETURN_HOME;
          RCLCPP_INFO(get_logger(), "All waypoints done, returning home");
          break;
        }
        {
          const Waypoint & wp = waypoints_[waypoint_index_];
          publish_trajectory_setpoint(wp.x, wp.y, wp.z);
          if (at_target(wp.x, wp.y, wp.z)) {
            RCLCPP_INFO(get_logger(), "Reached waypoint %zu", waypoint_index_);
            waypoint_index_++;
          }
        }
        break;

      case MissionState::RETURN_HOME:
        publish_trajectory_setpoint(0.0f, 0.0f, takeoff_z_);
        if (at_target(0.0f, 0.0f, takeoff_z_)) {
          RCLCPP_INFO(get_logger(), "Back home, landing");
          state_ = MissionState::LAND;
        }
        break;

      case MissionState::LAND:
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
        state_ = MissionState::FINISHED;
        break;

      case MissionState::FINISHED:
        // Keep the heartbeat alive but stop commanding movement.
        publish_trajectory_setpoint(0.0f, 0.0f, takeoff_z_);
        break;

      case MissionState::ERROR:
        break;
    }
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = timestamp_now();
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint(float x, float y, float z)
  {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.yaw = 0.0f;
    msg.yawspeed = NAN;
    msg.timestamp = timestamp_now();
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = timestamp_now();
    vehicle_command_pub_->publish(msg);
  }

  uint64_t timestamp_now()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  bool at_target(float x, float y, float z)
  {
    if (!local_position_.xy_valid || !local_position_.z_valid) {
      return false;
    }
    const float dx = local_position_.x - x;
    const float dy = local_position_.y - y;
    const float dz = local_position_.z - z;
    const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist < tolerance_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;

  VehicleLocalPosition local_position_{};
  VehicleStatus status_{};

  const float takeoff_z_ = -5.0f;
  const float tolerance_ = 1.0f;
  uint64_t streamed_setpoints_ = 0;
  size_t waypoint_index_ = 0;

  const std::vector<Waypoint> waypoints_ = {
    {5.0f, 0.0f, -5.0f},
    {5.0f, 5.0f, -5.0f},
    {0.0f, 5.0f, -5.0f},
    {-5.0f, 5.0f, -5.0f},
    {-5.0f, 0.0f, -5.0f},
  };

  MissionState state_ = MissionState::STREAMING;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardWaypoints>());
  rclcpp::shutdown();
  return 0;
}