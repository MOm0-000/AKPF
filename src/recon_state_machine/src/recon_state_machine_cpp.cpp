#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

enum class MissionState {
  WAITING_FCU,
  SETTING_GUIDED,
  ARMING,
  TAKEOFF,
  SCANNING,
  RETURN_HOME,
  LANDING,
  DONE
};

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Segment {
  Point3 start;
  Point3 end;
  double duration_s = 1.0;
  rclcpp::Time start_time;
};

class ReconStateMachineCpp : public rclcpp::Node {
public:
  ReconStateMachineCpp() : Node("recon_state_machine_cpp") {
    declare_parameter<double>("takeoff_alt", 2.0);
    declare_parameter<double>("waypoint_accept_radius", 0.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 300.0);
    declare_parameter<double>("trajectory_speed", 2.0);
    declare_parameter<double>("min_segment_time_s", 3.0);
    declare_parameter<std::vector<double>>(
      "scan_waypoints_xy",
      std::vector<double>{
        0.0, 0.0,
        -4.0, 52.5,
        4.0, 52.5,
        4.0, 57.5,
        -4.0, 57.5,
        0.0, 55.0});

    takeoff_alt_ = get_parameter("takeoff_alt").as_double();
    accept_radius_ = get_parameter("waypoint_accept_radius").as_double();
    takeoff_timeout_s_ = get_parameter("takeoff_timeout_s").as_double();
    mission_timeout_s_ = get_parameter("mission_timeout_s").as_double();
    trajectory_speed_ = std::max(0.1, get_parameter("trajectory_speed").as_double());
    min_segment_time_s_ = std::max(0.5, get_parameter("min_segment_time_s").as_double());
    scan_waypoints_ = parse_waypoints(get_parameter("scan_waypoints_xy").as_double_array());

    auto best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", best_effort,
      std::bind(&ReconStateMachineCpp::state_cb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", best_effort,
      std::bind(&ReconStateMachineCpp::odom_cb, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());

    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_cli_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    state_enter_time_ = now();
    mission_start_time_ = now();
    last_state_log_ = now();

    timer_ = create_wall_timer(50ms, std::bind(&ReconStateMachineCpp::control_loop, this));

    RCLCPP_INFO(get_logger(), "C++ reconnaissance state machine started");
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    current_position_.z = msg->pose.pose.position.z;

    if (!home_.has_value() && mission_state_ == MissionState::WAITING_FCU) {
      home_ = Point3{current_position_.x, current_position_.y, 0.0};
      target_ = Point3{current_position_.x, current_position_.y, 0.0};
      RCLCPP_INFO(
        get_logger(), "Home locked at local ENU (%.2f, %.2f)", home_->x, home_->y);
    }
  }

  void control_loop() {
    check_service_results();

    if (publish_setpoints_) {
      publish_setpoint(target_);
    }

    const bool mission_timeout_enabled =
      mission_state_ == MissionState::TAKEOFF ||
      mission_state_ == MissionState::SCANNING;

    if (mission_timeout_enabled && seconds_since(mission_start_time_) > mission_timeout_s_) {
      RCLCPP_WARN(get_logger(), "Mission timeout, returning home");
      start_return_home();
    }

    if (seconds_since(last_state_log_) > 5.0) {
      last_state_log_ = now();
      RCLCPP_INFO(
        get_logger(), "state=%s, pos=(%.1f, %.1f, %.1f), target=(%.1f, %.1f, %.1f)",
        state_name(mission_state_).c_str(),
        current_position_.x, current_position_.y, current_position_.z,
        target_.x, target_.y, target_.z);
    }

    switch (mission_state_) {
      case MissionState::WAITING_FCU:
        if (current_state_.connected && home_.has_value()) {
          enter_state(MissionState::SETTING_GUIDED);
        }
        break;

      case MissionState::SETTING_GUIDED:
        if (current_state_.mode == "GUIDED") {
          enter_state(MissionState::ARMING);
        } else if (!pending_set_mode_future_.valid()) {
          call_set_mode("GUIDED");
        }
        break;

      case MissionState::ARMING:
        if (current_state_.armed) {
          enter_state(MissionState::TAKEOFF);
        } else if (!pending_arm_future_.valid()) {
          call_arm(true);
        }
        break;

      case MissionState::TAKEOFF:
        publish_setpoints_ = false;
        if (!takeoff_sent_ && !pending_takeoff_future_.valid()) {
          takeoff_sent_ = call_takeoff(takeoff_alt_);
          if (takeoff_sent_) {
            state_enter_time_ = now();
          }
        }

        if (current_position_.z >= takeoff_alt_ * 0.95) {
          publish_setpoints_ = true;
          start_scan();
        } else if (takeoff_sent_ && seconds_since(state_enter_time_) > takeoff_timeout_s_) {
          RCLCPP_WARN(get_logger(), "Takeoff timeout, starting scan from current position");
          publish_setpoints_ = true;
          start_scan();
        }
        break;

      case MissionState::SCANNING:
        update_scan_trajectory();
        break;

      case MissionState::RETURN_HOME:
        update_return_trajectory();
        break;

      case MissionState::LANDING:
        publish_setpoints_ = false;
        if (!land_sent_ && !pending_land_future_.valid()) {
          land_sent_ = call_land();
        }
        if (current_position_.z < 0.2) {
          enter_state(MissionState::DONE);
        }
        break;

      case MissionState::DONE:
        RCLCPP_INFO(get_logger(), "Mission complete");
        rclcpp::shutdown();
        break;
    }
  }

  void start_scan() {
    scan_index_ = 0;
    start_segment(current_position_at_altitude(), scan_waypoints_.front());
    enter_state(MissionState::SCANNING);
  }

  void update_scan_trajectory() {
    target_ = sample_segment();
    if (!segment_finished()) {
      return;
    }

    ++scan_index_;
    if (scan_index_ >= scan_waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "Scan trajectory finished, returning home");
      start_return_home();
      return;
    }

    start_segment(active_segment_.end, scan_waypoints_[scan_index_]);
  }

  void start_return_home() {
    const auto home = home_.value_or(Point3{0.0, 0.0, 0.0});
    const Point3 return_point{home.x, home.y, takeoff_alt_};
    start_segment(current_position_at_altitude(), return_point);
    enter_state(MissionState::RETURN_HOME);
  }

  void update_return_trajectory() {
    target_ = sample_segment();
    if (
      segment_finished() ||
      distance_xy(current_position_, active_segment_.end) <= accept_radius_)
    {
      enter_state(MissionState::LANDING);
    }
  }

  void start_segment(const Point3 & start, const Point3 & end) {
    const double dist = distance_xyz(start, end);
    const double duration = std::max(min_segment_time_s_, dist / trajectory_speed_);
    active_segment_ = Segment{start, end, duration, now()};
    target_ = start;

    RCLCPP_INFO(
      get_logger(),
      "New min-snap segment: (%.1f, %.1f, %.1f) -> (%.1f, %.1f, %.1f), %.1f s",
      start.x, start.y, start.z, end.x, end.y, end.z, duration);
  }

  Point3 sample_segment() const {
    const double elapsed = std::clamp(
      seconds_since(active_segment_.start_time), 0.0, active_segment_.duration_s);
    const double tau = elapsed / active_segment_.duration_s;
    const double s = min_snap_blend(tau);
    return interpolate(active_segment_.start, active_segment_.end, s);
  }

  bool segment_finished() const {
    return seconds_since(active_segment_.start_time) >= active_segment_.duration_s;
  }

  Point3 current_position_at_altitude() const {
    return Point3{current_position_.x, current_position_.y, takeoff_alt_};
  }

  std::vector<Point3> parse_waypoints(const std::vector<double> & xy_values) const {
    std::vector<Point3> waypoints;

    if (xy_values.size() % 2 != 0) {
      RCLCPP_WARN(get_logger(), "scan_waypoints_xy has odd length; ignoring last value");
    }

    for (std::size_t i = 0; i + 1 < xy_values.size(); i += 2) {
      waypoints.push_back(Point3{xy_values[i], xy_values[i + 1], takeoff_alt_});
    }

    if (waypoints.empty()) {
      waypoints.push_back(Point3{0.0, 0.0, takeoff_alt_});
    }

    return waypoints;
  }

  void publish_setpoint(const Point3 & point) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = point.x;
    msg.pose.position.y = point.y;
    msg.pose.position.z = point.z;
    msg.pose.orientation.w = 1.0;
    setpoint_pub_->publish(msg);
  }

  void call_set_mode(const std::string & mode) {
    if (!set_mode_cli_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Set-mode service is not ready");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    pending_set_mode_future_ = set_mode_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Requested mode: %s", mode.c_str());
  }

  void call_arm(bool arm) {
    if (!arming_cli_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Arming service is not ready");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    pending_arm_future_ = arming_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Requested arming: %s", arm ? "true" : "false");
  }

  bool call_takeoff(double altitude) {
    if (!takeoff_cli_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Takeoff service is not ready");
      return false;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = static_cast<float>(altitude);
    pending_takeoff_future_ = takeoff_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Takeoff command sent: %.1f m", altitude);
    return true;
  }

  bool call_land() {
    if (!land_cli_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Land service is not ready");
      return false;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    pending_land_future_ = land_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Land command sent");
    return true;
  }

  void check_service_results() {
    if (
      pending_set_mode_future_.valid() &&
      pending_set_mode_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_set_mode_future_.get();
      if (!result->mode_sent) {
        RCLCPP_WARN(get_logger(), "Set mode failed; will retry");
      }
      pending_set_mode_future_ = {};
    }

    if (
      pending_arm_future_.valid() &&
      pending_arm_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_arm_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Arming failed; will retry");
      }
      pending_arm_future_ = {};
    }

    if (
      pending_takeoff_future_.valid() &&
      pending_takeoff_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_takeoff_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Takeoff command rejected; will retry");
        takeoff_sent_ = false;
      }
      pending_takeoff_future_ = {};
    }

    if (
      pending_land_future_.valid() &&
      pending_land_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_land_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Land command rejected; will retry");
        land_sent_ = false;
      }
      pending_land_future_ = {};
    }
  }

  void enter_state(MissionState next_state) {
    if (mission_state_ == next_state) {
      return;
    }

    mission_state_ = next_state;
    state_enter_time_ = now();

    if (next_state == MissionState::TAKEOFF) {
      mission_start_time_ = now();
      takeoff_sent_ = false;
    }

    if (next_state == MissionState::LANDING) {
      land_sent_ = false;
    }

    RCLCPP_INFO(get_logger(), "Enter %s", state_name(next_state).c_str());
  }

  double seconds_since(const rclcpp::Time & start) const {
    return (now() - start).seconds();
  }

  static double min_snap_blend(double tau) {
    tau = std::clamp(tau, 0.0, 1.0);
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    const double t4 = t3 * tau;
    const double t5 = t4 * tau;
    const double t6 = t5 * tau;
    const double t7 = t6 * tau;
    return 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7;
  }

  static Point3 interpolate(const Point3 & a, const Point3 & b, double s) {
    return Point3{
      a.x + (b.x - a.x) * s,
      a.y + (b.y - a.y) * s,
      a.z + (b.z - a.z) * s};
  }

  static double distance_xy(const Point3 & a, const Point3 & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double distance_xyz(const Point3 & a, const Point3 & b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static std::string state_name(MissionState state) {
    switch (state) {
      case MissionState::WAITING_FCU: return "WAITING_FCU";
      case MissionState::SETTING_GUIDED: return "SETTING_GUIDED";
      case MissionState::ARMING: return "ARMING";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::SCANNING: return "SCANNING";
      case MissionState::RETURN_HOME: return "RETURN_HOME";
      case MissionState::LANDING: return "LANDING";
      case MissionState::DONE: return "DONE";
    }
    return "UNKNOWN";
  }

  double takeoff_alt_ = 2.0;
  double accept_radius_ = 0.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 300.0;
  double trajectory_speed_ = 2.0;
  double min_segment_time_s_ = 3.0;

  mavros_msgs::msg::State current_state_;
  Point3 current_position_;
  std::optional<Point3> home_;
  Point3 target_;
  std::vector<Point3> scan_waypoints_;
  std::size_t scan_index_ = 0;
  Segment active_segment_;

  MissionState mission_state_ = MissionState::WAITING_FCU;
  bool publish_setpoints_ = false;
  bool takeoff_sent_ = false;
  bool land_sent_ = false;

  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_state_log_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>>
    pending_set_mode_future_;
  std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>>
    pending_arm_future_;
  std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandTOL::Response>>
    pending_takeoff_future_;
  std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandTOL::Response>>
    pending_land_future_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReconStateMachineCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
