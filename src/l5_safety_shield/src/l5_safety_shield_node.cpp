#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3 & a)
{
  return std::sqrt(dot(a, a));
}

double norm_xy(const Vec3 & a)
{
  return std::sqrt(a.x * a.x + a.y * a.y);
}

Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator*(const Vec3 & a, double s)
{
  return {a.x * s, a.y * s, a.z * s};
}

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

Vec3 normalize(const Vec3 & v)
{
  const double n = norm(v);
  if (n < 1e-6) {
    return {};
  }
  return {v.x / n, v.y / n, v.z / n};
}

std::string join_reasons(const std::vector<std::string> & reasons)
{
  if (reasons.empty()) {
    return "pass";
  }
  std::ostringstream out;
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << reasons[i];
  }
  return out.str();
}

class L5SafetyShieldNode : public rclcpp::Node
{
public:
  L5SafetyShieldNode() : Node("l5_safety_shield_node")
  {
    raw_cmd_topic_ = declare_parameter<std::string>("raw_cmd_topic", "/l5/raw_cmd_vel");
    safe_cmd_topic_ = declare_parameter<std::string>("safe_cmd_topic", "/mavros/setpoint_velocity/cmd_vel");
    nearest_distance_topic_ = declare_parameter<std::string>("nearest_distance_topic", "/l4/nearest_distance");
    nearest_normal_topic_ = declare_parameter<std::string>("nearest_normal_topic", "/l4/nearest_normal");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavros/local_position/local");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    map_timeout_s_ = declare_parameter<double>("map_timeout_s", 1.0);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);
    body_radius_ = declare_parameter<double>("body_radius", 0.35);
    safety_margin_ = declare_parameter<double>("safety_margin", 0.15);
    stop_d_eff_ = declare_parameter<double>("stop_d_eff", 0.08);
    caution_d_eff_ = declare_parameter<double>("caution_d_eff", 0.85);
    toward_speed_gain_ = declare_parameter<double>("toward_speed_gain", 0.8);
    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.35);
    max_z_speed_ = declare_parameter<double>("max_z_speed", 0.22);
    brake_accel_ = declare_parameter<double>("brake_accel", 0.70);
    control_delay_s_ = declare_parameter<double>("control_delay_s", 0.25);
    min_altitude_ = declare_parameter<double>("min_altitude", 1.2);
    max_altitude_ = declare_parameter<double>("max_altitude", 2.6);
    altitude_recovery_z_speed_ = declare_parameter<double>("altitude_recovery_z_speed", 0.12);

    publish_rate_hz_ = std::max(publish_rate_hz_, 5.0);

    auto velocity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    raw_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      raw_cmd_topic_, velocity_qos,
      std::bind(&L5SafetyShieldNode::raw_cmd_cb, this, std::placeholders::_1));
    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(safe_cmd_topic_, velocity_qos);

    nearest_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      nearest_distance_topic_, 10,
      std::bind(&L5SafetyShieldNode::nearest_distance_cb, this, std::placeholders::_1));
    nearest_normal_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      nearest_normal_topic_, 10,
      std::bind(&L5SafetyShieldNode::nearest_normal_cb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&L5SafetyShieldNode::odom_cb, this, std::placeholders::_1));

    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.reliable().transient_local();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, state_qos,
      std::bind(&L5SafetyShieldNode::state_cb, this, std::placeholders::_1));

    status_pub_ = create_publisher<std_msgs::msg::String>("/l5/shield_status", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/l5/shield_active", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&L5SafetyShieldNode::timer_cb, this));

    last_status_log_time_ = now() - rclcpp::Duration::from_seconds(10.0);
    RCLCPP_INFO(
      get_logger(),
      "L5 safety shield started: raw=%s safe=%s nearest=%s normal=%s",
      raw_cmd_topic_.c_str(), safe_cmd_topic_.c_str(),
      nearest_distance_topic_.c_str(), nearest_normal_topic_.c_str());
  }

private:
  void raw_cmd_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    raw_cmd_ = *msg;
    have_raw_cmd_ = true;
    last_cmd_time_ = now();
  }

  void nearest_distance_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    nearest_distance_ = static_cast<double>(msg->data);
    have_nearest_distance_ = std::isfinite(nearest_distance_);
    last_map_time_ = now();
  }

  void nearest_normal_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    nearest_normal_ = normalize({msg->vector.x, msg->vector.y, msg->vector.z});
    have_nearest_normal_ = norm(nearest_normal_) > 1e-6;
    last_map_time_ = now();
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_ = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    current_vel_ = {
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z};
    have_odom_ = true;
  }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
    have_state_ = true;
  }

  bool stale(const rclcpp::Time & stamp, double timeout_s) const
  {
    if (timeout_s <= 0.0) {
      return false;
    }
    return (now() - stamp).seconds() > timeout_s;
  }

  double effective_distance() const
  {
    if (!have_nearest_distance_) {
      return -std::numeric_limits<double>::infinity();
    }
    double d_eff = nearest_distance_ - body_radius_ - safety_margin_;
    if (have_nearest_normal_ && have_odom_) {
      const double v_toward = std::max(0.0, -dot(current_vel_, nearest_normal_));
      const double d_brake = (v_toward * v_toward) / std::max(2.0 * brake_accel_, 1e-3);
      const double d_delay = v_toward * control_delay_s_;
      d_eff -= d_brake + d_delay;
    }
    return d_eff;
  }

  geometry_msgs::msg::TwistStamped make_hover_cmd() const
  {
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = "map";
    return out;
  }

  void limit_speed(Vec3 & cmd, std::vector<std::string> & reasons) const
  {
    const double xy = norm_xy(cmd);
    if (xy > max_xy_speed_ && xy > 1e-6) {
      cmd.x *= max_xy_speed_ / xy;
      cmd.y *= max_xy_speed_ / xy;
      reasons.push_back("xy_limit");
    }
    if (std::abs(cmd.z) > max_z_speed_) {
      cmd.z = clamp(cmd.z, -max_z_speed_, max_z_speed_);
      reasons.push_back("z_limit");
    }
  }

  void apply_obstacle_shield(Vec3 & cmd, std::vector<std::string> & reasons) const
  {
    if (!have_nearest_distance_ || !have_nearest_normal_) {
      return;
    }

    const double d_eff = effective_distance();
    if (d_eff < caution_d_eff_) {
      reasons.push_back("near_obstacle");
    }

    const double toward = std::max(0.0, -dot(cmd, nearest_normal_));
    if (toward <= 1e-6) {
      return;
    }

    const double allowed_toward =
      toward_speed_gain_ * std::max(0.0, d_eff - stop_d_eff_);
    if (toward > allowed_toward) {
      cmd = cmd + nearest_normal_ * (toward - allowed_toward);
      reasons.push_back(d_eff < stop_d_eff_ ? "stop_toward_obstacle" : "limit_toward_obstacle");
    }
  }

  void apply_altitude_shield(Vec3 & cmd, std::vector<std::string> & reasons) const
  {
    if (!have_odom_) {
      return;
    }
    if (current_pos_.z < min_altitude_ && cmd.z < altitude_recovery_z_speed_) {
      cmd.z = altitude_recovery_z_speed_;
      reasons.push_back("low_altitude");
    }
    if (current_pos_.z > max_altitude_ && cmd.z > -altitude_recovery_z_speed_) {
      cmd.z = -altitude_recovery_z_speed_;
      reasons.push_back("high_altitude");
    }
  }

  void publish_status(const std::vector<std::string> & reasons, double d_eff)
  {
    std_msgs::msg::Bool active_msg;
    active_msg.data = !reasons.empty();
    active_pub_->publish(active_msg);

    std_msgs::msg::String status_msg;
    std::ostringstream out;
    out << join_reasons(reasons);
    out << " d_eff=" << d_eff;
    out << " nearest=" << nearest_distance_;
    out << " mode=" << current_state_.mode;
    out << " connected=" << (current_state_.connected ? "true" : "false");
    status_msg.data = out.str();
    status_pub_->publish(status_msg);

    if ((now() - last_status_log_time_).seconds() >= 1.0) {
      last_status_log_time_ = now();
      if (!reasons.empty()) {
        RCLCPP_WARN(get_logger(), "Shield active: %s", status_msg.data.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Shield pass: %s", status_msg.data.c_str());
      }
    }
  }

  void timer_cb()
  {
    if (!have_raw_cmd_) {
      return;
    }

    std::vector<std::string> reasons;
    geometry_msgs::msg::TwistStamped out = raw_cmd_;
    out.header.stamp = now();
    out.header.frame_id = raw_cmd_.header.frame_id.empty() ? "map" : raw_cmd_.header.frame_id;

    const bool cmd_stale = stale(last_cmd_time_, cmd_timeout_s_);
    const bool map_stale =
      !have_nearest_distance_ || !have_nearest_normal_ || stale(last_map_time_, map_timeout_s_);

    if (cmd_stale) {
      out = make_hover_cmd();
      reasons.push_back("cmd_timeout");
    } else if (!have_state_ || !current_state_.connected) {
      publish_status({"fcu_disconnected"}, effective_distance());
      return;
    } else if (current_state_.mode != "GUIDED") {
      out = make_hover_cmd();
      reasons.push_back("not_guided");
    } else if (map_stale) {
      out = make_hover_cmd();
      reasons.push_back("map_timeout");
    } else {
      Vec3 cmd{
        raw_cmd_.twist.linear.x,
        raw_cmd_.twist.linear.y,
        raw_cmd_.twist.linear.z};
      limit_speed(cmd, reasons);
      apply_obstacle_shield(cmd, reasons);
      apply_altitude_shield(cmd, reasons);
      limit_speed(cmd, reasons);
      out.twist.linear.x = cmd.x;
      out.twist.linear.y = cmd.y;
      out.twist.linear.z = cmd.z;
      out.twist.angular.z = raw_cmd_.twist.angular.z;
    }

    safe_cmd_pub_->publish(out);
    publish_status(reasons, effective_distance());
  }

  std::string raw_cmd_topic_{"/l5/raw_cmd_vel"};
  std::string safe_cmd_topic_{"/mavros/setpoint_velocity/cmd_vel"};
  std::string nearest_distance_topic_{"/l4/nearest_distance"};
  std::string nearest_normal_topic_{"/l4/nearest_normal"};
  std::string odom_topic_{"/mavros/local_position/local"};
  std::string state_topic_{"/mavros/state"};

  double publish_rate_hz_{20.0};
  double map_timeout_s_{1.0};
  double cmd_timeout_s_{0.5};
  double body_radius_{0.35};
  double safety_margin_{0.15};
  double stop_d_eff_{0.08};
  double caution_d_eff_{0.85};
  double toward_speed_gain_{0.8};
  double max_xy_speed_{0.35};
  double max_z_speed_{0.22};
  double brake_accel_{0.70};
  double control_delay_s_{0.25};
  double min_altitude_{1.2};
  double max_altitude_{2.6};
  double altitude_recovery_z_speed_{0.12};

  geometry_msgs::msg::TwistStamped raw_cmd_;
  mavros_msgs::msg::State current_state_;
  Vec3 current_pos_;
  Vec3 current_vel_;
  Vec3 nearest_normal_{1.0, 0.0, 0.0};
  double nearest_distance_{std::numeric_limits<double>::infinity()};

  bool have_raw_cmd_{false};
  bool have_state_{false};
  bool have_odom_{false};
  bool have_nearest_distance_{false};
  bool have_nearest_normal_{false};

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr raw_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr nearest_distance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr nearest_normal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<L5SafetyShieldNode>());
  rclcpp::shutdown();
  return 0;
}
