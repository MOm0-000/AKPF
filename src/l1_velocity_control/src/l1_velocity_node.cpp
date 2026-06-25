#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

enum class MissionState {
  WAITING_DEPS,
  WAITING_FCU,
  SETTING_GUIDED,
  ARMING,
  TAKEOFF,
  VELOCITY_TEST,
  HOVER,
  LANDING,
  DONE,
  FAILSAFE
};

struct VelocityStep {
  std::string name;
  double vx;
  double vy;
  double vz;
  double duration_s;
};

class L1VelocityNode : public rclcpp::Node {
public:
  L1VelocityNode() : Node("l1_velocity_node") {
    takeoff_alt_ = this->declare_parameter<double>("takeoff_alt", 2.0);
    v_xy_ = this->declare_parameter<double>("v_xy", 0.35);
    v_z_ = this->declare_parameter<double>("v_z", 0.20);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);
    use_stamped_cmd_vel_ = this->declare_parameter<bool>("use_stamped_cmd_vel", true);
    mission_timeout_s_ = this->declare_parameter<double>("mission_timeout_s", 180.0);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/mavros/local_position/local");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");

    if (publish_rate_hz_ < 10.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz %.1f is low; clamping to 10 Hz", publish_rate_hz_);
      publish_rate_hz_ = 10.0;
    }

    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.reliable().transient_local();

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&L1VelocityNode::state_cb, this, std::placeholders::_1));

    local_pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&L1VelocityNode::local_pos_cb, this, std::placeholders::_1));
    local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&L1VelocityNode::local_pose_cb, this, std::placeholders::_1));

    auto velocity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    stamped_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", velocity_qos);

    unstamped_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", velocity_qos);

    arming_cli_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_cli_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_cli_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    velocity_steps_ = {
        {"forward +X", v_xy_, 0.0, 0.0, 4.0},
        {"hover", 0.0, 0.0, 0.0, 2.0},
        {"backward -X", -v_xy_, 0.0, 0.0, 4.0},
        {"hover", 0.0, 0.0, 0.0, 2.0},
        {"left +Y", 0.0, v_xy_, 0.0, 4.0},
        {"hover", 0.0, 0.0, 0.0, 2.0},
        {"right -Y", 0.0, -v_xy_, 0.0, 4.0},
        {"hover", 0.0, 0.0, 0.0, 2.0},
        {"up +Z", 0.0, 0.0, v_z_, 3.0},
        {"hover", 0.0, 0.0, 0.0, 2.0},
        {"down -Z", 0.0, 0.0, -v_z_, 3.0},
        {"hover", 0.0, 0.0, 0.0, 3.0},
    };

    mission_start_time_ = this->now();
    state_entry_time_ = mission_start_time_;
    last_request_time_ = mission_start_time_ - rclcpp::Duration::from_seconds(10.0);
    last_diag_time_ = mission_start_time_ - rclcpp::Duration::from_seconds(10.0);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&L1VelocityNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "L1 velocity node started");
    RCLCPP_INFO(get_logger(), "Velocity interface: %s",
                use_stamped_cmd_vel_ ? "/mavros/setpoint_velocity/cmd_vel (TwistStamped)"
                                     : "/mavros/setpoint_velocity/cmd_vel_unstamped (Twist)");
    RCLCPP_INFO(get_logger(), "MAVROS position topics: odom=%s pose=%s", odom_topic_.c_str(), pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Limits: v_xy=%.2f m/s, v_z=%.2f m/s, publish_rate=%.1f Hz",
                v_xy_, v_z_, publish_rate_hz_);
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    state_received_ = true;
  }

  void local_pos_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_z_ = msg->pose.position.z;
    odom_received_ = true;
  }

  void control_loop() {
    if ((this->now() - mission_start_time_).seconds() > mission_timeout_s_ &&
        mission_state_ != MissionState::DONE && mission_state_ != MissionState::FAILSAFE) {
      RCLCPP_ERROR(get_logger(), "Mission timeout %.1f s, entering failsafe landing", mission_timeout_s_);
      enter_state(MissionState::FAILSAFE);
    }

    handle_futures();

    switch (mission_state_) {
      case MissionState::WAITING_DEPS:
        publish_zero_velocity_if_active(false);
        if (deps_ready()) {
          RCLCPP_INFO(get_logger(), "Dependencies ready: state, odom, arming, set_mode, takeoff, land");
          enter_state(MissionState::WAITING_FCU);
        } else {
          log_waiting_deps();
        }
        break;

      case MissionState::WAITING_FCU:
        publish_zero_velocity_if_active(false);
        if (current_state_.connected) {
          RCLCPP_INFO(get_logger(), "FCU connected");
          enter_state(MissionState::SETTING_GUIDED);
        } else {
          log_periodic("Waiting FCU heartbeat on /mavros/state");
        }
        break;

      case MissionState::SETTING_GUIDED:
        publish_zero_velocity_if_active(false);
        if (current_state_.mode == "GUIDED") {
          RCLCPP_INFO(get_logger(), "GUIDED already active");
          enter_state(MissionState::ARMING);
        } else if (request_due() && !pending_set_mode_future_.valid()) {
          call_set_mode("GUIDED");
        }
        break;

      case MissionState::ARMING:
        publish_zero_velocity_if_active(false);
        if (current_state_.armed) {
          RCLCPP_INFO(get_logger(), "Vehicle armed");
          enter_state(MissionState::TAKEOFF);
        } else if (request_due() && !pending_arm_future_.valid()) {
          call_arm(true);
        }
        break;

      case MissionState::TAKEOFF:
        publish_zero_velocity_if_active(false);
        if (!takeoff_sent_) {
          call_takeoff(takeoff_alt_);
          takeoff_sent_ = true;
          takeoff_request_time_ = this->now();
        }
        if (current_z_ >= takeoff_alt_ * 0.92) {
          RCLCPP_INFO(get_logger(), "Takeoff reached %.2f m, starting velocity test", current_z_);
          active_step_index_ = 0;
          enter_state(MissionState::VELOCITY_TEST);
        } else if ((this->now() - takeoff_request_time_).seconds() > 60.0) {
          RCLCPP_ERROR(get_logger(), "Takeoff timeout, entering failsafe landing");
          enter_state(MissionState::FAILSAFE);
        }
        break;

      case MissionState::VELOCITY_TEST:
        run_velocity_steps();
        break;

      case MissionState::HOVER:
        publish_velocity(0.0, 0.0, 0.0);
        if ((this->now() - state_entry_time_).seconds() >= 3.0) {
          enter_state(MissionState::LANDING);
        }
        break;

      case MissionState::LANDING:
        publish_velocity(0.0, 0.0, 0.0);
        if (!land_sent_) {
          call_land();
          land_sent_ = true;
        }
        if (current_z_ < 0.20 || !current_state_.armed) {
          RCLCPP_INFO(get_logger(), "Landed or disarmed, mission complete");
          enter_state(MissionState::DONE);
        }
        break;

      case MissionState::DONE:
        RCLCPP_INFO(get_logger(), "L1 velocity mission done, shutting down");
        rclcpp::shutdown();
        break;

      case MissionState::FAILSAFE:
        publish_velocity(0.0, 0.0, 0.0);
        if (!land_sent_ && land_cli_->service_is_ready()) {
          call_land();
          land_sent_ = true;
        }
        if (current_z_ < 0.20 || !current_state_.armed) {
          RCLCPP_INFO(get_logger(), "Failsafe landing complete, shutting down");
          rclcpp::shutdown();
        }
        break;
    }
  }

  bool deps_ready() const {
    return state_received_ && odom_received_ &&
           arming_cli_->service_is_ready() &&
           set_mode_cli_->service_is_ready() &&
           takeoff_cli_->service_is_ready() &&
           land_cli_->service_is_ready();
  }

  void log_waiting_deps() {
    if ((this->now() - last_diag_time_).seconds() < 2.0) {
      return;
    }
    last_diag_time_ = this->now();
    RCLCPP_INFO(get_logger(),
                "Waiting deps: state=%s odom=%s arming=%s set_mode=%s takeoff=%s land=%s",
                yes_no(state_received_), yes_no(odom_received_),
                yes_no(arming_cli_->service_is_ready()),
                yes_no(set_mode_cli_->service_is_ready()),
                yes_no(takeoff_cli_->service_is_ready()),
                yes_no(land_cli_->service_is_ready()));
  }

  void log_periodic(const std::string & text) {
    if ((this->now() - last_diag_time_).seconds() >= 2.0) {
      last_diag_time_ = this->now();
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
  }

  const char * yes_no(bool value) const {
    return value ? "yes" : "no";
  }

  bool request_due() const {
    return (this->now() - last_request_time_).seconds() >= 2.0;
  }

  void enter_state(MissionState next_state) {
    mission_state_ = next_state;
    state_entry_time_ = this->now();
    last_diag_time_ = state_entry_time_ - rclcpp::Duration::from_seconds(10.0);
    RCLCPP_INFO(get_logger(), "State -> %s", state_name(next_state));
  }

  const char * state_name(MissionState state) const {
    switch (state) {
      case MissionState::WAITING_DEPS: return "WAITING_DEPS";
      case MissionState::WAITING_FCU: return "WAITING_FCU";
      case MissionState::SETTING_GUIDED: return "SETTING_GUIDED";
      case MissionState::ARMING: return "ARMING";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::VELOCITY_TEST: return "VELOCITY_TEST";
      case MissionState::HOVER: return "HOVER";
      case MissionState::LANDING: return "LANDING";
      case MissionState::DONE: return "DONE";
      case MissionState::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
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
    last_request_time_ = this->now();
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
    last_request_time_ = this->now();
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
    last_request_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Takeoff command sent to %.2f m", alt);
  }

  void call_land() {
    if (!land_cli_->service_is_ready()) {
      log_periodic("Land service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_cli_->async_send_request(req);
    last_request_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Land command sent");
  }

  void run_velocity_steps() {
    if (active_step_index_ >= velocity_steps_.size()) {
      RCLCPP_INFO(get_logger(), "Velocity test sequence complete");
      enter_state(MissionState::HOVER);
      return;
    }

    const auto & step = velocity_steps_[active_step_index_];
    if (!step_announced_) {
      RCLCPP_INFO(get_logger(), "Velocity step %zu/%zu: %s, cmd=(%.2f, %.2f, %.2f), pos=(%.2f, %.2f, %.2f)",
                  active_step_index_ + 1, velocity_steps_.size(), step.name.c_str(),
                  step.vx, step.vy, step.vz, current_x_, current_y_, current_z_);
      step_announced_ = true;
    }

    double vz = step.vz;
    if (current_z_ < 1.0 && vz < 0.0) {
      vz = 0.0;
      log_periodic("Down velocity suppressed because altitude is below 1.0 m");
    }

    publish_velocity(step.vx, step.vy, vz);

    if ((this->now() - state_entry_time_).seconds() >= step.duration_s) {
      RCLCPP_INFO(get_logger(), "Velocity step complete: %s, pos=(%.2f, %.2f, %.2f)",
                  step.name.c_str(), current_x_, current_y_, current_z_);
      active_step_index_++;
      step_announced_ = false;
      state_entry_time_ = this->now();
    }
  }

  void publish_zero_velocity_if_active(bool force) {
    if (force) {
      publish_velocity(0.0, 0.0, 0.0);
    }
  }

  void publish_velocity(double vx, double vy, double vz) {
    if (use_stamped_cmd_vel_) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = this->now();
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

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr unstamped_velocity_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State current_state_;
  std::string odom_topic_{"/mavros/local_position/local"};
  std::string pose_topic_{"/mavros/local_position/pose"};
  bool state_received_ = false;
  bool odom_received_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_z_ = 0.0;

  MissionState mission_state_ = MissionState::WAITING_DEPS;
  rclcpp::Time mission_start_time_;
  rclcpp::Time state_entry_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_diag_time_;
  rclcpp::Time takeoff_request_time_;

  double takeoff_alt_;
  double v_xy_;
  double v_z_;
  double publish_rate_hz_;
  bool use_stamped_cmd_vel_;
  double mission_timeout_s_;

  bool takeoff_sent_ = false;
  bool land_sent_ = false;
  size_t active_step_index_ = 0;
  bool step_announced_ = false;
  std::vector<VelocityStep> velocity_steps_;

  std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> pending_set_mode_future_;
  std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> pending_arm_future_;
  std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandTOL::Response>> pending_takeoff_future_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<L1VelocityNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
