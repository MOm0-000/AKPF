#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vec3 operator+(const Vec3 & a, const Vec3 & b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 & a, const Vec3 & b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 & a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}

Vec3 operator/(const Vec3 & a, double s) {
  return {a.x / s, a.y / s, a.z / s};
}

double dot(const Vec3 & a, const Vec3 & b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3 & a) {
  return std::sqrt(dot(a, a));
}

double norm_xy(const Vec3 & a) {
  return std::sqrt(a.x * a.x + a.y * a.y);
}

Vec3 normalize(const Vec3 & a) {
  const double n = norm(a);
  if (n < 1e-6) {
    return {};
  }
  return a / n;
}

bool finite_vec(const Vec3 & a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

const sensor_msgs::msg::PointField * find_cloud_field(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & name) {
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

bool read_cloud_field(
    const sensor_msgs::msg::PointCloud2 & msg,
    const sensor_msgs::msg::PointField & field,
    std::size_t base,
    double & value) {
  if (base + field.offset >= msg.data.size()) {
    return false;
  }

  const auto * ptr = msg.data.data() + base + field.offset;
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    if (base + field.offset + sizeof(float) > msg.data.size()) {
      return false;
    }
    float raw{0.0F};
    std::memcpy(&raw, ptr, sizeof(float));
    value = static_cast<double>(raw);
    return true;
  }

  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    if (base + field.offset + sizeof(double) > msg.data.size()) {
      return false;
    }
    double raw{0.0};
    std::memcpy(&raw, ptr, sizeof(double));
    value = raw;
    return true;
  }

  return false;
}

struct BoxObstacle {
  std::string name;
  Vec3 center;
  Vec3 size;
};

struct DistanceQuery {
  std::string obstacle_name;
  double signed_distance{std::numeric_limits<double>::infinity()};
  Vec3 normal{1.0, 0.0, 0.0};
};

struct ScenarioGeometry {
  std::string name;
  Vec3 goal;
  double t_max_s{120.0};
  double min_altitude{1.2};
  double max_altitude{2.6};
  std::vector<BoxObstacle> obstacles;
};

enum class MissionState {
  WAITING_DEPS,
  WAITING_FCU,
  SETTING_GUIDED,
  ARMING,
  TAKEOFF,
  NAVIGATING,
  HOVER,
  LANDING,
  DONE,
  FAILSAFE
};

class L3AkpfNode : public rclcpp::Node {
public:
  L3AkpfNode() : Node("l3_akpf_node") {
    scenario_name_ = declare_parameter<std::string>("scenario", "S1_single_front_obstacle");
    takeoff_alt_ = declare_parameter<double>("takeoff_alt", 2.0);
    mission_timeout_s_ = declare_parameter<double>("mission_timeout_s", 0.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    use_stamped_cmd_vel_ = declare_parameter<bool>("use_stamped_cmd_vel", true);
    distance_source_ = declare_parameter<std::string>("distance_source", "truth_geometry");
    perception_cloud_topic_ = declare_parameter<std::string>("perception_cloud_topic", "/l4/local_cloud");
    perception_stale_timeout_s_ = declare_parameter<double>("perception_stale_timeout_s", 1.0);
    perception_fallback_to_truth_ = declare_parameter<bool>("perception_fallback_to_truth", true);
    perception_min_points_ = declare_parameter<int>("perception_min_points", 10);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavros/local_position/local");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");

    enable_repulsion_ = declare_parameter<bool>("enable_repulsion", true);
    enable_kinodynamic_ = declare_parameter<bool>("enable_kinodynamic", true);
    enable_curl_ = declare_parameter<bool>("enable_curl", true);
    enable_aero_ = declare_parameter<bool>("enable_aero", true);
    enable_local_target_ = declare_parameter<bool>("enable_local_target", true);

    k_att_ = declare_parameter<double>("k_att", 0.45);
    k_repulse_ = declare_parameter<double>("k_repulse", 0.55);
    k_curl_ = declare_parameter<double>("k_curl", 0.75);
    k_aero_ = declare_parameter<double>("k_aero", 0.20);
    max_xy_speed_ = declare_parameter<double>("max_xy_speed", 0.35);
    max_z_speed_ = declare_parameter<double>("max_z_speed", 0.22);
    max_accel_ = declare_parameter<double>("max_accel", 0.55);
    goal_radius_ = declare_parameter<double>("goal_radius", 0.50);
    slow_radius_ = declare_parameter<double>("slow_radius", 1.20);
    terminal_radius_ = declare_parameter<double>("terminal_radius", 1.40);
    terminal_goal_boost_ = declare_parameter<double>("terminal_goal_boost", 0.24);
    terminal_repulsion_relief_ = declare_parameter<double>("terminal_repulsion_relief", 0.65);
    anti_retreat_margin_ = declare_parameter<double>("anti_retreat_margin", 0.18);
    anti_retreat_progress_ = declare_parameter<double>("anti_retreat_progress", -0.05);
    anti_retreat_penalty_ = declare_parameter<double>("anti_retreat_penalty", 8.0);
    recovery_progress_floor_ = declare_parameter<double>("recovery_progress_floor", 0.05);
    candidate_safe_margin_ = declare_parameter<double>("candidate_safe_margin", 0.15);
    candidate_comfort_margin_ = declare_parameter<double>("candidate_comfort_margin", 0.50);
    local_target_clearance_ = declare_parameter<double>("local_target_clearance", 0.15);
    local_target_inflate_extra_ = declare_parameter<double>("local_target_inflate_extra", 0.25);
    local_target_hold_radius_ = declare_parameter<double>("local_target_hold_radius", 0.65);
    local_target_min_advance_ = declare_parameter<double>("local_target_min_advance", 0.85);
    local_target_repulsion_scale_ = declare_parameter<double>("local_target_repulsion_scale", 0.0);
    local_target_progress_weight_ = declare_parameter<double>("local_target_progress_weight", 0.55);
    local_target_sample_step_ = declare_parameter<double>("local_target_sample_step", 0.20);
    progress_stall_timeout_s_ = declare_parameter<double>("progress_stall_timeout_s", 8.0);
    progress_stall_epsilon_ = declare_parameter<double>("progress_stall_epsilon", 0.20);
    repulse_influence_ = declare_parameter<double>("repulse_influence", 1.20);
    aero_influence_ = declare_parameter<double>("aero_influence", 0.70);
    body_radius_ = declare_parameter<double>("body_radius", 0.35);
    safety_margin_ = declare_parameter<double>("safety_margin", 0.15);
    brake_accel_ = declare_parameter<double>("brake_accel", 0.70);
    control_delay_s_ = declare_parameter<double>("control_delay_s", 0.25);
    critical_d_eff_ = declare_parameter<double>("critical_d_eff", 0.08);
    emergency_d_eff_ = declare_parameter<double>("emergency_d_eff", 0.35);
    recovery_xy_speed_ = declare_parameter<double>("recovery_xy_speed", 0.22);
    recovery_climb_speed_ = declare_parameter<double>("recovery_climb_speed", 0.12);
    recovery_exit_d_eff_ = declare_parameter<double>("recovery_exit_d_eff", 0.85);
    recovery_exit_path_margin_ = declare_parameter<double>("recovery_exit_path_margin", 0.15);
    recovery_min_duration_s_ = declare_parameter<double>("recovery_min_duration_s", 3.0);

    if (recovery_exit_d_eff_ < emergency_d_eff_) {
      RCLCPP_WARN(
          get_logger(),
          "recovery_exit_d_eff %.2f is below emergency_d_eff %.2f; clamping exit threshold",
          recovery_exit_d_eff_, emergency_d_eff_);
      recovery_exit_d_eff_ = emergency_d_eff_;
    }
    recovery_min_duration_s_ = std::max(0.0, recovery_min_duration_s_);

    if (publish_rate_hz_ < 10.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz %.1f is low; clamping to 10 Hz", publish_rate_hz_);
      publish_rate_hz_ = 10.0;
    }

    scenario_ = make_scenario(scenario_name_);
    if (mission_timeout_s_ <= 0.0) {
      mission_timeout_s_ = scenario_.t_max_s;
    }
    takeoff_alt_ = clamp(takeoff_alt_, scenario_.min_altitude, scenario_.max_altitude);
    scenario_.goal.z = clamp(scenario_.goal.z, scenario_.min_altitude, scenario_.max_altitude);

    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.reliable().transient_local();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&L3AkpfNode::state_cb, this, std::placeholders::_1));

    local_pos_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&L3AkpfNode::local_pos_cb, this, std::placeholders::_1));
    local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&L3AkpfNode::local_pose_cb, this, std::placeholders::_1));

    const auto perception_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    perception_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        perception_cloud_topic_, perception_qos,
        std::bind(&L3AkpfNode::perception_cloud_cb, this, std::placeholders::_1));

    auto velocity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    stamped_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", velocity_qos);
    unstamped_velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", velocity_qos);

    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_cli_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    mission_start_time_ = now();
    state_entry_time_ = mission_start_time_;
    last_request_time_ = mission_start_time_ - rclcpp::Duration::from_seconds(10.0);
    last_diag_time_ = mission_start_time_ - rclcpp::Duration::from_seconds(10.0);
    last_command_time_ = mission_start_time_;
    last_progress_time_ = mission_start_time_;

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&L3AkpfNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "L3 AKPF node started");
    RCLCPP_INFO(get_logger(), "Scenario: %s, goal=(%.2f, %.2f, %.2f), obstacles=%zu",
                scenario_.name.c_str(), scenario_.goal.x, scenario_.goal.y, scenario_.goal.z,
                scenario_.obstacles.size());
    RCLCPP_INFO(get_logger(), "AKPF terms: repulsion=%s kino=%s curl=%s aero=%s",
                yes_no(enable_repulsion_), yes_no(enable_kinodynamic_),
                yes_no(enable_curl_), yes_no(enable_aero_));
    RCLCPP_INFO(get_logger(), "Distance source: %s (perception cloud: %s, fallback_to_truth=%s)",
                distance_source_.c_str(), perception_cloud_topic_.c_str(),
                yes_no(perception_fallback_to_truth_));
    RCLCPP_INFO(get_logger(), "MAVROS position topics: odom=%s pose=%s", odom_topic_.c_str(), pose_topic_.c_str());
  }

private:
  ScenarioGeometry make_scenario(const std::string & name) {
    if (name == "S1_single_front_obstacle") {
      return with_room_boundaries(
          {name, {4.20, 0.00, 2.00}, 90.0, 1.2, 2.6,
           {{"front_block", {3.00, 0.00, 1.25}, {0.70, 1.60, 2.50}}}});
    }
    if (name == "S2_narrow_gate") {
      return with_room_boundaries(
          {name, {4.20, 0.00, 2.00}, 100.0, 1.2, 2.6,
           {{"gate_left_pillar", {2.50, 0.90, 1.30}, {0.45, 0.45, 2.60}},
            {"gate_right_pillar", {2.50, -0.90, 1.30}, {0.45, 0.45, 2.60}},
            {"gate_top_reference", {2.50, 0.00, 2.65}, {0.50, 2.25, 0.12}}}});
    }
    if (name == "S3_corridor") {
      return with_room_boundaries(
          {name, {4.20, 0.00, 2.00}, 110.0, 1.2, 2.6,
           {{"corridor_left_wall", {2.20, 1.20, 1.35}, {5.80, 0.16, 2.70}},
            {"corridor_right_wall", {2.20, -1.20, 1.35}, {5.80, 0.16, 2.70}}}});
    }
    if (name == "S4_table_or_low_obstacle") {
      return with_room_boundaries(
          {name, {4.20, 0.00, 2.00}, 100.0, 1.2, 2.6,
           {{"low_table_top", {2.60, 0.00, 0.80}, {1.70, 1.10, 0.16}},
            {"low_table_leg_1", {1.90, 0.45, 0.40}, {0.12, 0.12, 0.80}},
            {"low_table_leg_2", {3.30, 0.45, 0.40}, {0.12, 0.12, 0.80}},
            {"low_table_leg_3", {1.90, -0.45, 0.40}, {0.12, 0.12, 0.80}},
            {"low_table_leg_4", {3.30, -0.45, 0.40}, {0.12, 0.12, 0.80}}}});
    }
    if (name == "S5_corner") {
      return with_room_boundaries(
          {name, {3.80, 2.60, 2.00}, 120.0, 1.2, 2.6,
           {{"corner_vertical_wall", {2.00, -0.70, 1.35}, {0.18, 2.60, 2.70}},
            {"corner_horizontal_wall", {3.05, 1.30, 1.35}, {2.30, 0.18, 2.70}},
            {"corner_inner_block", {2.55, 0.55, 1.25}, {0.55, 0.55, 2.50}}}});
    }


    if (name == "S6_cluttered_boxes") {
      return with_room_boundaries(
          {name, {4.35, 0.85, 2.00}, 150.0, 1.2, 2.6,
           {{"clutter_box_1", {1.60, 1.35, 1.25}, {0.50, 0.60, 2.50}},
            {"clutter_box_2", {2.40, -1.35, 1.25}, {0.55, 0.60, 2.50}},
            {"clutter_box_3", {3.05, 1.35, 1.25}, {0.50, 0.60, 2.50}},
            {"clutter_box_4", {3.65, -0.85, 1.25}, {0.45, 0.55, 2.50}},
            {"clutter_box_5", {3.90, 1.95, 1.25}, {0.40, 0.50, 2.50}}}});
    }
    if (name == "S8_vertical_constraint") {
      return with_room_boundaries(
          {name, {4.20, 0.00, 1.55}, 140.0, 1.1, 2.6,
           {{"low_ceiling_slab", {2.65, 0.00, 2.45}, {2.50, 2.30, 0.30}},
            {"ceiling_left_support", {2.05, 1.20, 1.25}, {0.18, 0.18, 2.50}},
            {"ceiling_right_support", {3.25, -1.20, 1.25}, {0.18, 0.18, 2.50}}}});
    }
    if (name == "S9_multi_corner") {
      return with_room_boundaries(
          {name, {4.40, 2.65, 2.00}, 160.0, 1.2, 2.6,
           {{"multi_corner_wall_1", {1.35, -0.95, 1.35}, {0.18, 2.20, 2.70}},
            {"multi_corner_wall_2", {2.60, 1.05, 1.35}, {2.20, 0.18, 2.70}},
            {"multi_corner_wall_3", {3.45, 2.85, 1.35}, {0.18, 0.95, 2.70}},
            {"multi_corner_block", {2.55, -0.05, 1.25}, {0.55, 0.55, 2.50}}}});
    }

    RCLCPP_WARN(get_logger(), "Unknown scenario '%s'; falling back to S1_single_front_obstacle",
                name.c_str());
    return make_scenario("S1_single_front_obstacle");
  }

  ScenarioGeometry with_room_boundaries(ScenarioGeometry scenario) const {
    scenario.obstacles.push_back({"l2_wall_x_positive", {5.00, 0.00, 1.50}, {0.12, 8.00, 3.00}});
    scenario.obstacles.push_back({"l2_wall_x_negative", {-5.00, 0.00, 1.50}, {0.12, 8.00, 3.00}});
    scenario.obstacles.push_back({"l2_wall_y_positive", {0.00, 4.00, 1.50}, {10.00, 0.12, 3.00}});
    scenario.obstacles.push_back({"l2_wall_y_negative", {0.00, -4.00, 1.50}, {10.00, 0.12, 3.00}});
    return scenario;
  }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    state_received_ = true;
  }

  void local_pos_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z};
    current_vel_ = {
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z};
    odom_received_ = true;
  }

  void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const Vec3 next_pos{
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z};
    const rclcpp::Time stamp =
        msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ? now() : rclcpp::Time(msg->header.stamp);

    if (odom_received_) {
      const double dt = (stamp - last_pose_time_).seconds();
      if (dt > 1e-3 && dt < 1.0) {
        current_vel_ = {
            (next_pos.x - current_pos_.x) / dt,
            (next_pos.y - current_pos_.y) / dt,
            (next_pos.z - current_pos_.z) / dt};
      }
    }

    current_pos_ = next_pos;
    last_pose_time_ = stamp;
    odom_received_ = true;
  }

  void perception_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->is_bigendian) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring big-endian perception PointCloud2.");
      return;
    }

    const auto * x_field = find_cloud_field(*msg, "x");
    const auto * y_field = find_cloud_field(*msg, "y");
    const auto * z_field = find_cloud_field(*msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring perception cloud without x/y/z fields.");
      return;
    }

    std::vector<Vec3> points;
    points.reserve(static_cast<std::size_t>(msg->width) * std::max<std::uint32_t>(msg->height, 1));
    const std::size_t rows = std::max<std::uint32_t>(msg->height, 1);
    const std::size_t cols = msg->width;
    for (std::size_t row = 0; row < rows; ++row) {
      for (std::size_t col = 0; col < cols; ++col) {
        const std::size_t base = row * msg->row_step + col * msg->point_step;
        Vec3 p;
        if (!read_cloud_field(*msg, *x_field, base, p.x) ||
            !read_cloud_field(*msg, *y_field, base, p.y) ||
            !read_cloud_field(*msg, *z_field, base, p.z) ||
            !finite_vec(p)) {
          continue;
        }
        points.push_back(p);
      }
    }

    perception_points_ = std::move(points);
    last_perception_cloud_time_ = now();
    has_perception_cloud_ = true;
  }

  void control_loop() {
    if ((now() - mission_start_time_).seconds() > mission_timeout_s_ &&
        mission_state_ != MissionState::DONE && mission_state_ != MissionState::FAILSAFE) {
      RCLCPP_ERROR(get_logger(), "Mission timeout %.1f s, entering failsafe landing", mission_timeout_s_);
      enter_state(MissionState::FAILSAFE);
    }

    handle_futures();

    switch (mission_state_) {
      case MissionState::WAITING_DEPS:
        if (deps_ready()) {
          RCLCPP_INFO(get_logger(), "Dependencies ready: state, odom, arming, set_mode, takeoff, land, perception");
          enter_state(MissionState::WAITING_FCU);
        } else {
          log_waiting_deps();
        }
        break;

      case MissionState::WAITING_FCU:
        if (current_state_.connected) {
          RCLCPP_INFO(get_logger(), "FCU connected");
          enter_state(MissionState::SETTING_GUIDED);
        } else {
          log_periodic("Waiting FCU heartbeat on /mavros/state");
        }
        break;

      case MissionState::SETTING_GUIDED:
        if (current_state_.mode == "GUIDED") {
          RCLCPP_INFO(get_logger(), "GUIDED already active");
          enter_state(MissionState::ARMING);
        } else if (request_due() && !pending_set_mode_future_.valid()) {
          call_set_mode("GUIDED");
        }
        break;

      case MissionState::ARMING:
        if (current_state_.armed) {
          RCLCPP_INFO(get_logger(), "Vehicle armed");
          enter_state(MissionState::TAKEOFF);
        } else if (request_due() && !pending_arm_future_.valid()) {
          call_arm(true);
        }
        break;

      case MissionState::TAKEOFF:
        if (!takeoff_sent_) {
          call_takeoff(takeoff_alt_);
          takeoff_sent_ = true;
          takeoff_request_time_ = now();
        }
        if (current_pos_.z >= takeoff_alt_ * 0.92) {
          RCLCPP_INFO(get_logger(), "Takeoff reached %.2f m, starting AKPF navigation", current_pos_.z);
          enter_state(MissionState::NAVIGATING);
        } else if ((now() - takeoff_request_time_).seconds() > 60.0) {
          RCLCPP_ERROR(get_logger(), "Takeoff timeout, entering failsafe landing");
          enter_state(MissionState::FAILSAFE);
        }
        break;

      case MissionState::NAVIGATING:
        run_akpf_navigation();
        break;

      case MissionState::HOVER:
        publish_velocity(0.0, 0.0, 0.0);
        if ((now() - state_entry_time_).seconds() >= 3.0) {
          enter_state(MissionState::LANDING);
        }
        break;

      case MissionState::LANDING:
        publish_velocity(0.0, 0.0, 0.0);
        if (!land_sent_) {
          call_land();
          land_sent_ = true;
        }
        if (current_pos_.z < 0.20 || !current_state_.armed) {
          RCLCPP_INFO(get_logger(), "Landed or disarmed, mission complete");
          enter_state(MissionState::DONE);
        }
        break;

      case MissionState::DONE:
        RCLCPP_INFO(get_logger(), "L3 AKPF mission done, shutting down");
        rclcpp::shutdown();
        break;

      case MissionState::FAILSAFE:
        publish_velocity(0.0, 0.0, 0.0);
        if (!land_sent_ && land_cli_->service_is_ready()) {
          call_land();
          land_sent_ = true;
        }
        if (current_pos_.z < 0.20 || !current_state_.armed) {
          RCLCPP_INFO(get_logger(), "Failsafe landing complete, shutting down");
          rclcpp::shutdown();
        }
        break;
    }
  }

  bool deps_ready() {
    refresh_perception_cloud_state();
    return state_received_ && odom_received_ &&
           arming_cli_->service_is_ready() &&
           set_mode_cli_->service_is_ready() &&
           takeoff_cli_->service_is_ready() &&
           land_cli_->service_is_ready() &&
           (perception_fallback_to_truth_ || !using_perception_source() || perception_cloud_current_);
  }

  void run_akpf_navigation() {
    refresh_perception_cloud_state();
    if (using_perception_source() && !perception_cloud_current_ && !perception_fallback_to_truth_) {
      RCLCPP_ERROR(get_logger(), "Perception distance source is stale or unavailable; entering failsafe");
      enter_state(MissionState::FAILSAFE);
      return;
    }

    const Vec3 goal_error = scenario_.goal - current_pos_;
    const double distance_to_goal = norm(goal_error);
    const DistanceQuery nearest = nearest_obstacle(current_pos_);
    last_nearest_distance_ = nearest.signed_distance;
    last_d_eff_ = effective_distance(nearest);

    if (current_pos_.z < scenario_.min_altitude - 0.25 ||
        current_pos_.z > scenario_.max_altitude + 0.25 ||
        nearest.signed_distance < -0.03 ||
        last_d_eff_ < -0.08) {
      RCLCPP_ERROR(get_logger(),
                   "Safety violation: z=%.2f, nearest=%s %.2f m d_eff=%.2f; entering failsafe",
                   current_pos_.z, nearest.obstacle_name.c_str(), nearest.signed_distance, last_d_eff_);
      enter_state(MissionState::FAILSAFE);
      return;
    }

    if (distance_to_goal <= goal_radius_) {
      RCLCPP_INFO(get_logger(),
                  "Goal reached: pos=(%.2f, %.2f, %.2f), goal_dist=%.2f, min_clearance=%.2f",
                  current_pos_.x, current_pos_.y, current_pos_.z, distance_to_goal,
                  min_nearest_distance_);
      enter_state(MissionState::HOVER);
      return;
    }

    if (!std::isfinite(best_goal_distance_) ||
        distance_to_goal < best_goal_distance_ - progress_stall_epsilon_) {
      best_goal_distance_ = distance_to_goal;
      last_progress_time_ = now();
    }

    const bool force_local_target =
        enable_local_target_ && distance_to_goal > terminal_radius_ &&
        (now() - last_progress_time_).seconds() >= progress_stall_timeout_s_;

    const Vec3 nav_target = choose_navigation_target(scenario_.goal, force_local_target);
    const Vec3 nav_error = nav_target - current_pos_;

    const bool perception_recovery_available =
        using_perception_source() && std::isfinite(last_d_eff_);
    if (perception_recovery_available && !recovery_active_ && last_d_eff_ < emergency_d_eff_) {
      recovery_active_ = true;
      recovery_start_time_ = now();
      RCLCPP_WARN(
          get_logger(),
          "Entering perception recovery: nearest=%s %.2f m d_eff=%.2f",
          nearest.obstacle_name.c_str(), nearest.signed_distance, last_d_eff_);
    }
    if (recovery_active_) {
      const double recovery_elapsed = (now() - recovery_start_time_).seconds();
      const Vec3 recovery_exit_dir = normalize_xy_vec({nav_error.x, nav_error.y, 0.0});
      const double recovery_exit_margin = norm_xy(recovery_exit_dir) > 1e-6 ?
          predicted_clearance_margin(recovery_exit_dir, norm(nav_error)) :
          std::numeric_limits<double>::infinity();
      if (!perception_recovery_available) {
        recovery_active_ = false;
      } else if (last_d_eff_ > recovery_exit_d_eff_ &&
                 recovery_elapsed >= recovery_min_duration_s_ &&
                 recovery_exit_margin >= recovery_exit_path_margin_) {
        recovery_active_ = false;
        RCLCPP_INFO(
            get_logger(),
            "Leaving perception recovery: nearest=%s %.2f m d_eff=%.2f path_margin=%.2f elapsed=%.1fs",
            nearest.obstacle_name.c_str(), nearest.signed_distance, last_d_eff_,
            recovery_exit_margin, recovery_elapsed);
      }
    }

    Vec3 cmd;
    const bool emergency_recovery = perception_recovery_available && recovery_active_;
    if (emergency_recovery) {
      cmd = compute_recovery_velocity(nearest, nav_error);
    } else {
      cmd = compute_akpf_velocity(nav_error, nearest);
      cmd = select_local_candidate_velocity(cmd, nav_error);
    }
    cmd = limit_speed(cmd, distance_to_goal);
    if (!emergency_recovery) {
      cmd = limit_accel(cmd);
    } else {
      last_command_time_ = now();
    }

    if (current_pos_.z < scenario_.min_altitude && cmd.z < 0.0) {
      cmd.z = 0.0;
    }
    if (current_pos_.z > scenario_.max_altitude && cmd.z > 0.0) {
      cmd.z = 0.0;
    }

    publish_velocity(cmd.x, cmd.y, cmd.z);
    last_cmd_ = cmd;
    min_nearest_distance_ = std::min(min_nearest_distance_, nearest.signed_distance);
    min_d_eff_ = std::min(min_d_eff_, last_d_eff_);

    if ((now() - last_diag_time_).seconds() >= 1.0) {
      last_diag_time_ = now();
      RCLCPP_INFO(get_logger(),
                  "AKPF pos=(%.2f, %.2f, %.2f) goal=%.2f nav=%s(%.2f, %.2f) nearest=%s %.2f d_eff=%.2f mode=%s cmd=(%.2f, %.2f, %.2f)",
                  current_pos_.x, current_pos_.y, current_pos_.z, distance_to_goal,
                  using_local_target_ ? "local" : "global", nav_target.x, nav_target.y,
                  nearest.obstacle_name.c_str(), nearest.signed_distance, last_d_eff_,
                  emergency_recovery ? "recover" : "nav",
                  cmd.x, cmd.y, cmd.z);
    }
  }

  Vec3 normalize_xy_vec(const Vec3 & v) const {
    const double n = norm_xy(v);
    if (n < 1e-6) {
      return {};
    }
    return {v.x / n, v.y / n, 0.0};
  }

  Vec3 rotate_xy(const Vec3 & v, double degrees) const {
    constexpr double kPi = 3.14159265358979323846;
    const double rad = degrees * kPi / 180.0;
    const double c = std::cos(rad);
    const double sn = std::sin(rad);
    return {v.x * c - v.y * sn, v.x * sn + v.y * c, 0.0};
  }

  bool is_room_boundary(const BoxObstacle & obstacle) const {
    return obstacle.name.rfind("l2_wall_", 0) == 0;
  }

  double point_clearance_margin(const Vec3 & point) const {
    return nearest_obstacle(point).signed_distance - body_radius_ - safety_margin_;
  }

  double segment_clearance_margin(const Vec3 & from, const Vec3 & to) const {
    const Vec3 delta = to - from;
    const double length = norm_xy(delta);
    const int steps = std::max(2, static_cast<int>(std::ceil(length / std::max(local_target_sample_step_, 0.05))));
    double min_margin = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      Vec3 sample = from + delta * t;
      sample.z = current_pos_.z;
      min_margin = std::min(min_margin, point_clearance_margin(sample));
    }
    return min_margin;
  }

  bool segment_is_clear(const Vec3 & from, const Vec3 & to) const {
    return segment_clearance_margin(from, to) >= local_target_clearance_;
  }

  Vec3 choose_navigation_target(const Vec3 & global_goal, bool force_local_target = false) {
    if (!enable_local_target_) {
      has_active_local_target_ = false;
      using_local_target_ = false;
      return global_goal;
    }

    if (has_active_local_target_) {
      active_local_target_.z = global_goal.z;
      const bool target_not_reached = norm_xy(active_local_target_ - current_pos_) > local_target_hold_radius_;
      const bool target_still_progresses =
        norm_xy(global_goal - active_local_target_) + 0.15 < norm_xy(global_goal - current_pos_);
      if (target_not_reached &&
          target_still_progresses &&
          point_clearance_margin(active_local_target_) >= local_target_clearance_) {
        using_local_target_ = true;
        return active_local_target_;
      }
      has_active_local_target_ = false;
      using_local_target_ = false;
    }

    if (!force_local_target && segment_is_clear(current_pos_, global_goal)) {
      using_local_target_ = false;
      return global_goal;
    }

    const double inflate = body_radius_ + safety_margin_ + local_target_inflate_extra_;
    const Vec3 global_goal_dir = normalize_xy_vec({global_goal.x - current_pos_.x, global_goal.y - current_pos_.y, 0.0});
    Vec3 best = global_goal;
    double best_cost = std::numeric_limits<double>::infinity();
    bool found_goal_visible = false;
    bool found_reachable = false;

    for (const auto & obstacle : scenario_.obstacles) {
      if (is_room_boundary(obstacle)) {
        continue;
      }
      for (const double sx : {-1.0, 1.0}) {
        for (const double sy : {-1.0, 1.0}) {
          Vec3 candidate{
              obstacle.center.x + sx * (0.5 * obstacle.size.x + inflate),
              obstacle.center.y + sy * (0.5 * obstacle.size.y + inflate),
              global_goal.z};

          if (point_clearance_margin(candidate) < local_target_clearance_) {
            continue;
          }
          if (!segment_is_clear(current_pos_, candidate)) {
            continue;
          }

          const bool goal_visible = segment_is_clear(candidate, global_goal);
          const Vec3 candidate_dir = normalize_xy_vec({candidate.x - current_pos_.x, candidate.y - current_pos_.y, 0.0});
          const double progress = dot(candidate_dir, global_goal_dir);
          const double progress_distance = dot(candidate - current_pos_, global_goal_dir);
          if (progress_distance < local_target_min_advance_) {
            continue;
          }
          const double cost = norm(candidate - current_pos_) + norm(global_goal - candidate) -
            0.35 * progress - local_target_progress_weight_ * progress_distance;

          if ((goal_visible && !found_goal_visible) ||
              (goal_visible == found_goal_visible && cost < best_cost) ||
              (!found_goal_visible && !found_reachable && cost < best_cost)) {
            best = candidate;
            best_cost = cost;
            found_goal_visible = goal_visible;
            found_reachable = true;
          }
        }
      }
    }

    if (found_reachable) {
      active_local_target_ = best;
      has_active_local_target_ = true;
      using_local_target_ = true;
      return active_local_target_;
    }
    using_local_target_ = false;
    return global_goal;
  }

  Vec3 choose_tangent(const DistanceQuery & nearest, const Vec3 & goal_error) const {
    Vec3 left{-nearest.normal.y, nearest.normal.x, 0.0};
    left = normalize_xy_vec(left);
    Vec3 right{nearest.normal.y, -nearest.normal.x, 0.0};
    right = normalize_xy_vec(right);
    const Vec3 goal_dir = normalize_xy_vec({goal_error.x, goal_error.y, 0.0});
    const Vec3 base_dir = norm_xy(goal_dir) > 1e-6 ? goal_dir : left;

    const double left_score = score_candidate_direction(left, goal_dir, base_dir);
    const double right_score = score_candidate_direction(right, goal_dir, base_dir);
    return left_score >= right_score ? left : right;
  }

  Vec3 select_local_candidate_velocity(const Vec3 & field, const Vec3 & goal_error) const {
    const Vec3 goal_dir = normalize_xy_vec({goal_error.x, goal_error.y, 0.0});
    if (norm_xy(goal_dir) < 1e-6) {
      return field;
    }

    const Vec3 field_dir = normalize_xy_vec({field.x, field.y, 0.0});
    const Vec3 base_dir = norm_xy(field_dir) > 1e-6 ? field_dir : goal_dir;
    const double goal_distance = norm(goal_error);
    const double candidate_speed = clamp(std::max(norm_xy(field), 0.22), 0.18, max_xy_speed_);

    if (using_local_target_ && norm_xy(field_dir) > 1e-6 &&
        predicted_clearance_margin(field_dir, goal_distance) >= 0.0) {
      return {field_dir.x * candidate_speed, field_dir.y * candidate_speed, field.z};
    }

    const std::vector<double> angles_deg{
        -180.0, -150.0, -120.0, -90.0, -60.0, -35.0, -15.0,
        0.0, 15.0, 35.0, 60.0, 90.0, 120.0, 150.0, 180.0};

    struct Candidate {
      Vec3 dir;
      double score{0.0};
      double progress{0.0};
      double margin{0.0};
    };

    std::vector<Candidate> candidates;
    candidates.reserve(angles_deg.size() + 1);

    auto add_candidate = [&](Vec3 dir) {
      dir = normalize_xy_vec(dir);
      if (norm_xy(dir) < 1e-6) {
        return;
      }
      candidates.push_back({
          dir,
          score_candidate_direction(dir, goal_dir, base_dir, goal_distance),
          dot(dir, goal_dir),
          predicted_clearance_margin(dir, goal_distance)});
    };

    for (const double angle : angles_deg) {
      add_candidate(rotate_xy(goal_dir, angle));
    }
    add_candidate(base_dir);

    const double current_margin = nearest_obstacle(current_pos_).signed_distance - body_radius_ - safety_margin_;
    const double progress_floor = current_margin < anti_retreat_margin_ ?
        recovery_progress_floor_ : anti_retreat_progress_;

    bool has_safe_progress = false;
    bool has_noncolliding_progress = false;
    for (const auto & candidate : candidates) {
      if (candidate.progress >= progress_floor && candidate.margin >= candidate_safe_margin_) {
        has_safe_progress = true;
      }
      if (candidate.progress >= progress_floor && candidate.margin >= 0.0) {
        has_noncolliding_progress = true;
      }
    }

    Vec3 best_dir = base_dir;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const auto & candidate : candidates) {
      if (has_safe_progress &&
          (candidate.progress < progress_floor || candidate.margin < candidate_safe_margin_)) {
        continue;
      }
      if (!has_safe_progress && has_noncolliding_progress &&
          (candidate.progress < progress_floor || candidate.margin < 0.0)) {
        continue;
      }
      if (candidate.score > best_score) {
        best_score = candidate.score;
        best_dir = candidate.dir;
      }
    }

    return {best_dir.x * candidate_speed, best_dir.y * candidate_speed, field.z};
  }


  double predicted_clearance_margin(const Vec3 & dir, double goal_distance) const {
    double min_clearance = std::numeric_limits<double>::infinity();
    for (const double horizon : {0.45, 0.90, 1.35, 1.80, 2.40, 3.00}) {
      double sample_distance = horizon;
      if (std::isfinite(goal_distance)) {
        // Do not punish a direction for what would happen after the vehicle has already reached the goal.
        sample_distance = std::min(horizon, std::max(0.25, goal_distance));
      }
      Vec3 probe = current_pos_ + dir * sample_distance;
      probe.z = current_pos_.z;
      min_clearance = std::min(min_clearance, nearest_obstacle(probe).signed_distance);
    }
    return min_clearance - body_radius_ - safety_margin_;
  }

  double score_candidate_direction(
      const Vec3 & dir, const Vec3 & goal_dir, const Vec3 & base_dir,
      double goal_distance = std::numeric_limits<double>::infinity()) const {
    const double progress = dot(dir, goal_dir);
    const double base_alignment = dot(dir, base_dir);
    const double terminal = std::isfinite(goal_distance) ?
        clamp((terminal_radius_ - goal_distance) / std::max(terminal_radius_ - goal_radius_, 1e-3), 0.0, 1.0) :
        0.0;

    double smoothness = 0.0;
    const Vec3 last_dir = normalize_xy_vec({last_cmd_.x, last_cmd_.y, 0.0});
    if (norm_xy(last_dir) > 1e-6) {
      smoothness = dot(dir, last_dir);
    }

    const double margin = predicted_clearance_margin(dir, goal_distance);
    const double safety_weight = 1.10 - 0.30 * terminal;
    const double progress_weight = 2.20 + 2.00 * terminal;
    const double safety_score = margin >= 0.0 ?
        safety_weight * std::min(margin, candidate_comfort_margin_) :
        -20.0 + 20.0 * margin;

    double retreat_penalty = 0.0;
    const double current_margin = nearest_obstacle(current_pos_).signed_distance - body_radius_ - safety_margin_;
    const bool recovery_needed = current_margin < anti_retreat_margin_;
    const bool terminal_goal_region = std::isfinite(goal_distance) && goal_distance <= terminal_radius_;
    if (!recovery_needed && !terminal_goal_region && progress < anti_retreat_progress_) {
      const double retreat_depth = anti_retreat_progress_ - progress;
      retreat_penalty = anti_retreat_penalty_ * retreat_depth * (1.0 + std::min(std::max(current_margin, 0.0), 1.0));
    }

    return safety_score + progress_weight * progress + 0.25 * base_alignment + 0.25 * smoothness - retreat_penalty;
  }

  Vec3 compute_akpf_velocity(const Vec3 & goal_error, const DistanceQuery & nearest) const {
    const double goal_distance = norm(goal_error);
    const double terminal = clamp(
        (terminal_radius_ - goal_distance) / std::max(terminal_radius_ - goal_radius_, 1e-3), 0.0, 1.0);
    const Vec3 goal_dir = normalize(goal_error);
    Vec3 field = goal_error * k_att_ + goal_dir * (terminal_goal_boost_ * terminal);

    if (enable_repulsion_) {
      const double d_eff = effective_distance(nearest);
      if (d_eff < repulse_influence_) {
        const double risk = clamp((repulse_influence_ - d_eff) / repulse_influence_, 0.0, 1.0);
        const double local_scale = using_local_target_ ? local_target_repulsion_scale_ : 1.0;
        const double repulsion_scale = local_scale * (1.0 - terminal_repulsion_relief_ * terminal);
        const double gain = k_repulse_ * risk * risk * repulsion_scale;
        field = field + nearest.normal * gain;

        if (d_eff < critical_d_eff_) {
          field = field + nearest.normal * (k_repulse_ * 0.8);
        }

        if (enable_curl_) {
          const double obstacle_ahead = dot(goal_dir, nearest.normal * -1.0);
          if (obstacle_ahead > 0.20) {
            Vec3 tangent = choose_tangent(nearest, goal_error);

            const double opposing_tangent_component = dot(field, tangent);
            if (opposing_tangent_component < 0.0) {
              field = field - tangent * (0.80 * opposing_tangent_component);
            }

            field = field + tangent * (k_curl_ * (0.35 + risk) * repulsion_scale);
          }
        }
      }
    }

    if (enable_aero_) {
      if (nearest.signed_distance < aero_influence_) {
        const double risk = clamp((aero_influence_ - nearest.signed_distance) / aero_influence_, 0.0, 1.0);
        field = field + nearest.normal * (k_aero_ * risk * risk * (1.0 - 0.50 * terminal));
      }
      const double ground_risk_height = scenario_.min_altitude + 0.20;
      if (current_pos_.z < ground_risk_height) {
        const double risk = clamp((ground_risk_height - current_pos_.z) / 0.40, 0.0, 1.0);
        field.z += k_aero_ * risk;
      }
    }

    return field;
  }

  Vec3 compute_recovery_velocity(const DistanceQuery & nearest, const Vec3 & nav_error) const {
    Vec3 away = normalize_xy_vec({nearest.normal.x, nearest.normal.y, 0.0});
    if (norm_xy(away) < 1e-6) {
      away = normalize_xy_vec({-last_cmd_.x, -last_cmd_.y, 0.0});
    }
    if (norm_xy(away) < 1e-6) {
      away = {-1.0, 0.0, 0.0};
    }

    const Vec3 goal_dir = normalize_xy_vec({nav_error.x, nav_error.y, 0.0});
    Vec3 tangent_left{-away.y, away.x, 0.0};
    Vec3 tangent_right{away.y, -away.x, 0.0};
    tangent_left = normalize_xy_vec(tangent_left);
    tangent_right = normalize_xy_vec(tangent_right);
    Vec3 tangent = dot(tangent_left, goal_dir) >= dot(tangent_right, goal_dir) ?
        tangent_left : tangent_right;
    if (norm_xy(goal_dir) < 1e-6 || norm_xy(tangent) < 1e-6) {
      tangent = normalize_xy_vec({last_cmd_.x, last_cmd_.y, 0.0});
    }
    if (norm_xy(tangent) < 1e-6) {
      tangent = tangent_left;
    }

    const double risk = clamp((emergency_d_eff_ - last_d_eff_) / std::max(emergency_d_eff_, 1e-3), 0.0, 1.0);
    const double normal_weight = 0.35 + 0.45 * risk;
    const double tangent_weight = 0.95 - 0.30 * risk;
    Vec3 recovery_dir = normalize_xy_vec(away * normal_weight + tangent * tangent_weight);
    if (norm_xy(recovery_dir) < 1e-6) {
      recovery_dir = away;
    }

    double vz = 0.0;
    if (current_pos_.z < scenario_.goal.z) {
      vz = std::min(recovery_climb_speed_, scenario_.goal.z - current_pos_.z);
    }
    if (current_pos_.z < scenario_.min_altitude + 0.25) {
      vz = std::max(vz, recovery_climb_speed_);
    }

    return {
      recovery_dir.x * recovery_xy_speed_ * (0.80 + 0.20 * risk),
      recovery_dir.y * recovery_xy_speed_ * (0.80 + 0.20 * risk),
      vz};
  }

  Vec3 limit_speed(Vec3 cmd, double distance_to_goal) const {
    double max_xy = max_xy_speed_;
    if (distance_to_goal < slow_radius_) {
      max_xy *= clamp(distance_to_goal / slow_radius_, 0.30, 1.0);
    }

    const double xy = norm_xy(cmd);
    if (xy > max_xy && xy > 1e-6) {
      cmd.x *= max_xy / xy;
      cmd.y *= max_xy / xy;
    }
    cmd.z = clamp(cmd.z, -max_z_speed_, max_z_speed_);
    return cmd;
  }

  Vec3 limit_accel(Vec3 cmd) {
    const double dt = std::max(1.0 / publish_rate_hz_, (now() - last_command_time_).seconds());
    last_command_time_ = now();
    const double max_delta = max_accel_ * dt;

    Vec3 delta = cmd - last_cmd_;
    const double delta_norm = norm(delta);
    if (delta_norm > max_delta && delta_norm > 1e-6) {
      cmd = last_cmd_ + delta * (max_delta / delta_norm);
    }
    return cmd;
  }

  DistanceQuery nearest_obstacle(const Vec3 & p) const {
    if (using_perception_source() &&
        perception_cloud_current_ &&
        perception_points_.size() >= static_cast<std::size_t>(std::max(perception_min_points_, 1))) {
      return query_perception_cloud(p);
    }

    DistanceQuery best;
    for (const auto & obstacle : scenario_.obstacles) {
      DistanceQuery q = query_box(p, obstacle);
      if (q.signed_distance < best.signed_distance) {
        best = q;
      }
    }
    return best;
  }

  DistanceQuery query_perception_cloud(const Vec3 & p) const {
    DistanceQuery best;
    best.obstacle_name = "perception_map";
    Vec3 nearest_point;

    for (const auto & point : perception_points_) {
      const double d = norm(point - p);
      if (d < best.signed_distance) {
        best.signed_distance = d;
        nearest_point = point;
      }
    }

    if (!std::isfinite(best.signed_distance)) {
      best.normal = {1.0, 0.0, 0.0};
      return best;
    }

    best.normal = normalize(p - nearest_point);
    if (norm(best.normal) < 1e-6) {
      best.normal = {1.0, 0.0, 0.0};
    }
    return best;
  }

  DistanceQuery query_box(const Vec3 & p, const BoxObstacle & box) const {
    const Vec3 half = box.size * 0.5;
    const Vec3 d{p.x - box.center.x, p.y - box.center.y, p.z - box.center.z};
    const Vec3 ad{std::abs(d.x), std::abs(d.y), std::abs(d.z)};
    const Vec3 q{ad.x - half.x, ad.y - half.y, ad.z - half.z};
    const Vec3 outside{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)};
    const double outside_dist = norm(outside);

    DistanceQuery result;
    result.obstacle_name = box.name;

    if (outside_dist > 1e-6) {
      const Vec3 closest{
          clamp(p.x, box.center.x - half.x, box.center.x + half.x),
          clamp(p.y, box.center.y - half.y, box.center.y + half.y),
          clamp(p.z, box.center.z - half.z, box.center.z + half.z)};
      result.normal = normalize(p - closest);
      result.signed_distance = outside_dist;
      return result;
    }

    const double px = half.x - ad.x;
    const double py = half.y - ad.y;
    const double pz = half.z - ad.z;
    result.signed_distance = -std::min({px, py, pz});

    if (px <= py && px <= pz) {
      result.normal = {d.x >= 0.0 ? 1.0 : -1.0, 0.0, 0.0};
    } else if (py <= px && py <= pz) {
      result.normal = {0.0, d.y >= 0.0 ? 1.0 : -1.0, 0.0};
    } else {
      result.normal = {0.0, 0.0, d.z >= 0.0 ? 1.0 : -1.0};
    }
    return result;
  }

  double effective_distance(const DistanceQuery & q) const {
    double d_eff = q.signed_distance - body_radius_ - safety_margin_;
    if (enable_kinodynamic_) {
      const double v_toward = std::max(0.0, -dot(current_vel_, q.normal));
      const double d_brake = (v_toward * v_toward) / std::max(2.0 * brake_accel_, 1e-3);
      const double d_delay = v_toward * control_delay_s_;
      d_eff -= d_brake + d_delay;
    }
    return d_eff;
  }

  bool request_due() const {
    return (now() - last_request_time_).seconds() >= 2.0;
  }

  void enter_state(MissionState next_state) {
    mission_state_ = next_state;
    state_entry_time_ = now();
    last_diag_time_ = state_entry_time_ - rclcpp::Duration::from_seconds(10.0);
    if (next_state == MissionState::NAVIGATING) {
      best_goal_distance_ = std::numeric_limits<double>::infinity();
      last_progress_time_ = state_entry_time_;
      has_active_local_target_ = false;
      using_local_target_ = false;
      recovery_active_ = false;
    }
    RCLCPP_INFO(get_logger(), "State -> %s", state_name(next_state));
  }

  const char * state_name(MissionState state) const {
    switch (state) {
      case MissionState::WAITING_DEPS: return "WAITING_DEPS";
      case MissionState::WAITING_FCU: return "WAITING_FCU";
      case MissionState::SETTING_GUIDED: return "SETTING_GUIDED";
      case MissionState::ARMING: return "ARMING";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::NAVIGATING: return "NAVIGATING";
      case MissionState::HOVER: return "HOVER";
      case MissionState::LANDING: return "LANDING";
      case MissionState::DONE: return "DONE";
      case MissionState::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  void log_waiting_deps() {
    refresh_perception_cloud_state();
    if ((now() - last_diag_time_).seconds() < 2.0) {
      return;
    }
    last_diag_time_ = now();
    RCLCPP_INFO(get_logger(),
                "Waiting deps: state=%s odom=%s arming=%s set_mode=%s takeoff=%s land=%s perception=%s",
                yes_no(state_received_), yes_no(odom_received_),
                yes_no(arming_cli_->service_is_ready()),
                yes_no(set_mode_cli_->service_is_ready()),
                yes_no(takeoff_cli_->service_is_ready()),
                yes_no(land_cli_->service_is_ready()),
                yes_no(!using_perception_source() || perception_cloud_current_ || perception_fallback_to_truth_));
  }

  void log_periodic(const std::string & text) {
    if ((now() - last_diag_time_).seconds() >= 2.0) {
      last_diag_time_ = now();
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
  }

  const char * yes_no(bool value) const {
    return value ? "yes" : "no";
  }

  bool using_perception_source() const {
    return distance_source_ == "perception_map";
  }

  void refresh_perception_cloud_state() {
    if (!using_perception_source()) {
      perception_cloud_current_ = true;
      return;
    }
    if (!has_perception_cloud_ ||
        perception_points_.size() < static_cast<std::size_t>(std::max(perception_min_points_, 1))) {
      perception_cloud_current_ = false;
      return;
    }
    if (perception_stale_timeout_s_ <= 0.0) {
      perception_cloud_current_ = true;
      return;
    }
    perception_cloud_current_ =
        (now() - last_perception_cloud_time_).seconds() <= perception_stale_timeout_s_;
  }

  void handle_futures() {
    if (pending_set_mode_future_.valid() &&
        pending_set_mode_future_.wait_for(0s) == std::future_status::ready) {
      auto result = pending_set_mode_future_.get();
      RCLCPP_INFO(get_logger(), "Set mode response: mode_sent=%s", yes_no(result->mode_sent));
      pending_set_mode_future_ = {};
    }

    if (pending_arm_future_.valid() &&
        pending_arm_future_.wait_for(0s) == std::future_status::ready) {
      auto result = pending_arm_future_.get();
      RCLCPP_INFO(get_logger(), "Arm response: success=%s", yes_no(result->success));
      pending_arm_future_ = {};
    }

    if (pending_takeoff_future_.valid() &&
        pending_takeoff_future_.wait_for(0s) == std::future_status::ready) {
      auto result = pending_takeoff_future_.get();
      RCLCPP_INFO(get_logger(), "Takeoff response: success=%s", yes_no(result->success));
      pending_takeoff_future_ = {};
    }
  }

  void call_set_mode(const std::string & mode) {
    if (!set_mode_cli_->service_is_ready()) {
      log_periodic("Set mode service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    auto future = set_mode_cli_->async_send_request(req);
    pending_set_mode_future_ = future.future.share();
    last_request_time_ = now();
    RCLCPP_INFO(get_logger(), "Requested mode: %s", mode.c_str());
  }

  void call_arm(bool arm) {
    if (!arming_cli_->service_is_ready()) {
      log_periodic("Arming service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    auto future = arming_cli_->async_send_request(req);
    pending_arm_future_ = future.future.share();
    last_request_time_ = now();
    RCLCPP_INFO(get_logger(), "Requested arming: %s", arm ? "true" : "false");
  }

  void call_takeoff(double alt) {
    if (!takeoff_cli_->service_is_ready()) {
      log_periodic("Takeoff service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = static_cast<float>(alt);
    auto future = takeoff_cli_->async_send_request(req);
    pending_takeoff_future_ = future.future.share();
    last_request_time_ = now();
    RCLCPP_INFO(get_logger(), "Takeoff command sent to %.2f m", alt);
  }

  void call_land() {
    if (!land_cli_->service_is_ready()) {
      log_periodic("Land service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_cli_->async_send_request(req);
    last_request_time_ = now();
    RCLCPP_INFO(get_logger(), "Land command sent");
  }

  void publish_velocity(double vx, double vy, double vz) {
    if (use_stamped_cmd_vel_) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = "map";
      msg.twist.linear.x = vx;
      msg.twist.linear.y = vy;
      msg.twist.linear.z = vz;
      msg.twist.angular.z = 0.0;
      stamped_velocity_pub_->publish(msg);
    } else {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = vx;
      msg.linear.y = vy;
      msg.linear.z = vz;
      msg.angular.z = 0.0;
      unstamped_velocity_pub_->publish(msg);
    }
  }

  std::string scenario_name_;
  ScenarioGeometry scenario_;
  std::string distance_source_{"truth_geometry"};
  std::string perception_cloud_topic_{"/l4/local_cloud"};
  std::string odom_topic_{"/mavros/local_position/local"};
  std::string pose_topic_{"/mavros/local_position/pose"};
  double perception_stale_timeout_s_{1.0};
  bool perception_fallback_to_truth_{true};
  int perception_min_points_{10};

  double takeoff_alt_{2.0};
  double mission_timeout_s_{120.0};
  double publish_rate_hz_{20.0};
  bool use_stamped_cmd_vel_{true};
  bool enable_repulsion_{true};
  bool enable_kinodynamic_{true};
  bool enable_curl_{true};
  bool enable_aero_{true};
  bool enable_local_target_{true};

  double k_att_{0.45};
  double k_repulse_{0.55};
  double k_curl_{0.75};
  double k_aero_{0.20};
  double max_xy_speed_{0.35};
  double max_z_speed_{0.22};
  double max_accel_{0.55};
  double goal_radius_{0.50};
  double slow_radius_{1.20};
  double terminal_radius_{1.40};
  double terminal_goal_boost_{0.24};
  double terminal_repulsion_relief_{0.65};
  double anti_retreat_margin_{0.18};
  double anti_retreat_progress_{-0.05};
  double anti_retreat_penalty_{8.0};
  double recovery_progress_floor_{0.05};
  double candidate_safe_margin_{0.15};
  double candidate_comfort_margin_{0.50};
  double local_target_clearance_{0.15};
  double local_target_inflate_extra_{0.25};
  double local_target_hold_radius_{0.65};
  double local_target_min_advance_{0.85};
  double local_target_repulsion_scale_{0.0};
  double local_target_progress_weight_{0.55};
  double local_target_sample_step_{0.20};
  double progress_stall_timeout_s_{8.0};
  double progress_stall_epsilon_{0.20};
  double repulse_influence_{1.20};
  double aero_influence_{0.70};
  double body_radius_{0.35};
  double safety_margin_{0.15};
  double brake_accel_{0.70};
  double control_delay_s_{0.25};
  double critical_d_eff_{0.08};
  double emergency_d_eff_{0.35};
  double recovery_xy_speed_{0.22};
  double recovery_climb_speed_{0.12};
  double recovery_exit_d_eff_{0.85};
  double recovery_exit_path_margin_{0.15};
  double recovery_min_duration_s_{3.0};

  mavros_msgs::msg::State current_state_;
  Vec3 current_pos_;
  Vec3 current_vel_;
  Vec3 last_cmd_;
  Vec3 active_local_target_;
  bool has_active_local_target_{false};
  bool using_local_target_{false};
  std::vector<Vec3> perception_points_;

  bool state_received_{false};
  bool odom_received_{false};
  bool takeoff_sent_{false};
  bool land_sent_{false};
  bool has_perception_cloud_{false};
  bool perception_cloud_current_{false};
  bool recovery_active_{false};

  double last_nearest_distance_{std::numeric_limits<double>::infinity()};
  double last_d_eff_{std::numeric_limits<double>::infinity()};
  double min_nearest_distance_{std::numeric_limits<double>::infinity()};
  double min_d_eff_{std::numeric_limits<double>::infinity()};
  double best_goal_distance_{std::numeric_limits<double>::infinity()};

  MissionState mission_state_{MissionState::WAITING_DEPS};
  rclcpp::Time mission_start_time_;
  rclcpp::Time state_entry_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_diag_time_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_progress_time_;
  rclcpp::Time takeoff_request_time_;
  rclcpp::Time recovery_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_perception_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr perception_cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr unstamped_velocity_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture pending_set_mode_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture pending_arm_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture pending_takeoff_future_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<L3AkpfNode>());
  rclcpp::shutdown();
  return 0;
}
