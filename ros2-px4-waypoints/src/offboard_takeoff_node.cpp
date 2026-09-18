#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardTakeoff : public rclcpp::Node
{
public:
  OffboardTakeoff()
  : Node("offboard_takeoff")
  {
    declare_parameter<double>("takeoff_height_m", 2.0);
    declare_parameter<double>("position_tolerance_m", 0.25);

    takeoff_height_m_ = get_parameter("takeoff_height_m").as_double();
    tolerance_m_ = get_parameter("position_tolerance_m").as_double();
    takeoff_z_ = static_cast<float>(-std::abs(takeoff_height_m_));

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    trajectory_pub_ = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    local_position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleLocalPosition::SharedPtr msg) {
        local_position_ = *msg;
        if (msg->xy_valid && !origin_captured_) {
          target_x_ = msg->x;
          target_y_ = msg->y;
          origin_captured_ = true;
          RCLCPP_INFO(get_logger(), "Takeoff origin captured: x=%.2f y=%.2f",
            target_x_, target_y_);
        }
      });

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", rclcpp::SensorDataQoS(),
      [this](const VehicleStatus::SharedPtr msg) { status_ = *msg; });

    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleCommandAck::SharedPtr msg) {
        if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
          msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
        {
          RCLCPP_INFO(get_logger(), "PX4 ACK command=%u result=%u",
            msg->command, msg->result);
        }
      });

    timer_ = create_wall_timer(50ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Takeoff target: %.2f m above origin",
      std::abs(takeoff_z_));
  }

private:
  enum class State { STREAMING, WAIT_OFFBOARD, WAIT_ARMED, TAKEOFF, HOLD };

  void tick()
  {
    publish_offboard_mode();

    switch (state_) {
      case State::STREAMING:
        publish_setpoint();
        if (++setpoint_count_ >= 20) {
          publish_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          state_ = State::WAIT_OFFBOARD;
          RCLCPP_INFO(get_logger(), "Offboard mode requested");
        }
        break;

      case State::WAIT_OFFBOARD:
        publish_setpoint();
        if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          publish_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          state_ = State::WAIT_ARMED;
          RCLCPP_INFO(get_logger(), "Offboard accepted; arm requested");
        }
        break;

      case State::WAIT_ARMED:
        publish_setpoint();
        if (status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
          state_ = State::TAKEOFF;
          RCLCPP_INFO(get_logger(), "Armed; taking off");
        }
        break;

      case State::TAKEOFF:
      case State::HOLD:
        publish_setpoint();
        if (state_ == State::TAKEOFF && at_target()) {
          state_ = State::HOLD;
          RCLCPP_INFO(get_logger(), "Takeoff complete; holding position");
        }
        break;
    }
  }

  void publish_offboard_mode()
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
    offboard_mode_pub_->publish(msg);
  }

  void publish_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = {target_x_, target_y_, takeoff_z_};
    msg.velocity = {nan(), nan(), nan()};
    msg.acceleration = {nan(), nan(), nan()};
    msg.yaw = 0.0f;
    msg.yawspeed = nan();
    msg.timestamp = timestamp_now();
    trajectory_pub_->publish(msg);
  }

  void publish_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
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
    command_pub_->publish(msg);
  }

  bool at_target() const
  {
    if (!origin_captured_ || !local_position_.z_valid) {
      return false;
    }

    const float dx = local_position_.x - target_x_;
    const float dy = local_position_.y - target_y_;
    const float dz = local_position_.z - takeoff_z_;
    return std::sqrt(dx * dx + dy * dy + dz * dz) < tolerance_m_;
  }

  static float nan()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  uint64_t timestamp_now() const
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;

  VehicleLocalPosition local_position_{};
  VehicleStatus status_{};
  State state_{State::STREAMING};
  uint32_t setpoint_count_{0};
  bool origin_captured_{false};
  float target_x_{0.0f};
  float target_y_{0.0f};
  float takeoff_z_{-2.0f};
  double takeoff_height_m_{2.0};
  double tolerance_m_{0.25};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardTakeoff>());
  rclcpp::shutdown();
  return 0;
}
