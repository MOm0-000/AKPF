#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<int>{}(key.x);
    const auto hy = std::hash<int>{}(key.y);
    const auto hz = std::hash<int>{}(key.z);
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

struct VoxelEntry
{
  Point3 point;
  rclcpp::Time stamp;
};

double norm(const Point3 & p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

double dot(const Point3 & a, const Point3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3 operator-(const Point3 & a, const Point3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Point3 operator+(const Point3 & a, const Point3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

bool finite_point(const Point3 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

Quaternion normalize_quat(Quaternion q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-9) {
    return {};
  }
  q.x /= n;
  q.y /= n;
  q.z /= n;
  q.w /= n;
  return q;
}

Point3 rotate_by_quat(const Point3 & p, const Quaternion & q_in)
{
  const Quaternion q = normalize_quat(q_in);
  const Point3 u{q.x, q.y, q.z};
  const double s = q.w;
  const double dot_uv = u.x * p.x + u.y * p.y + u.z * p.z;
  const Point3 cross{
    u.y * p.z - u.z * p.y,
    u.z * p.x - u.x * p.z,
    u.x * p.y - u.y * p.x};
  return {
    2.0 * dot_uv * u.x + (s * s - dot(u, u)) * p.x + 2.0 * s * cross.x,
    2.0 * dot_uv * u.y + (s * s - dot(u, u)) * p.y + 2.0 * s * cross.y,
    2.0 * dot_uv * u.z + (s * s - dot(u, u)) * p.z + 2.0 * s * cross.z};
}

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

bool read_field_value(
  const sensor_msgs::msg::PointCloud2 & msg,
  const sensor_msgs::msg::PointField & field,
  std::size_t base,
  double & value)
{
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

sensor_msgs::msg::PointCloud2 make_cloud_msg(
  const std::vector<Point3> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.point_step = 3 * sizeof(float);
  msg.row_step = msg.point_step * msg.width;

  sensor_msgs::msg::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::msg::PointField y_field = x_field;
  y_field.name = "y";
  y_field.offset = sizeof(float);

  sensor_msgs::msg::PointField z_field = x_field;
  z_field.name = "z";
  z_field.offset = 2 * sizeof(float);

  msg.fields = {x_field, y_field, z_field};
  msg.data.resize(static_cast<std::size_t>(msg.row_step));

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::array<float, 3> xyz{
      static_cast<float>(points[i].x),
      static_cast<float>(points[i].y),
      static_cast<float>(points[i].z)};
    std::memcpy(msg.data.data() + i * msg.point_step, xyz.data(), msg.point_step);
  }
  return msg;
}

}  // namespace

class L4PointcloudMapper : public rclcpp::Node
{
public:
  L4PointcloudMapper()
  : Node("l4_pointcloud_mapper")
  {
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/l4/points");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/mavros/local_position/local");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");
    use_odom_ = declare_parameter<bool>("use_odom", true);
    query_position_.x = declare_parameter<double>("query_x", 0.0);
    query_position_.y = declare_parameter<double>("query_y", 0.0);
    query_position_.z = declare_parameter<double>("query_z", 2.0);
    local_radius_m_ = declare_parameter<double>("local_radius_m", 6.0);
    voxel_size_m_ = declare_parameter<double>("voxel_size_m", 0.15);
    max_input_points_ = declare_parameter<int>("max_input_points", 200000);
    map_memory_s_ = declare_parameter<double>("map_memory_s", 90.0);
    max_map_voxels_ = declare_parameter<int>("max_map_voxels", 80000);
    stale_cloud_timeout_s_ = declare_parameter<double>("stale_cloud_timeout_s", 2.0);
    query_rate_hz_ = declare_parameter<double>("query_rate_hz", 10.0);
    log_period_s_ = declare_parameter<double>("log_period_s", 1.0);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    transform_to_odom_frame_ = declare_parameter<bool>("transform_to_odom_frame", false);
    camera_offset_.x = declare_parameter<double>("camera_offset_x", 0.22);
    camera_offset_.y = declare_parameter<double>("camera_offset_y", 0.0);
    camera_offset_.z = declare_parameter<double>("camera_offset_z", 0.06);

    if (voxel_size_m_ <= 0.0) {
      voxel_size_m_ = 0.15;
    }
    if (local_radius_m_ <= 0.0) {
      local_radius_m_ = 6.0;
    }
    if (query_rate_hz_ <= 0.0) {
      query_rate_hz_ = 10.0;
    }
    if (map_memory_s_ < 0.0) {
      map_memory_s_ = 0.0;
    }
    if (max_map_voxels_ < 100) {
      max_map_voxels_ = 100;
    }

    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, cloud_qos,
      std::bind(&L4PointcloudMapper::pointcloud_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&L4PointcloudMapper::odom_callback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&L4PointcloudMapper::pose_callback, this, std::placeholders::_1));

    nearest_distance_pub_ = create_publisher<std_msgs::msg::Float32>("/l4/nearest_distance", 10);
    nearest_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/l4/nearest_point", 10);
    nearest_normal_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/l4/nearest_normal", 10);
    local_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/l4/local_cloud", cloud_qos);

    const auto period = std::chrono::duration<double>(1.0 / query_rate_hz_);
    query_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&L4PointcloudMapper::query_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "L4 pointcloud mapper started: cloud=%s odom=%s pose=%s use_odom=%s transform=%s voxel=%.2f local_radius=%.2f memory=%.1fs",
      pointcloud_topic_.c_str(), odom_topic_.c_str(), pose_topic_.c_str(), use_odom_ ? "true" : "false",
      transform_to_odom_frame_ ? "odom_frame" : "input_frame", voxel_size_m_, local_radius_m_, map_memory_s_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_position_.x = msg->pose.pose.position.x;
    odom_position_.y = msg->pose.pose.position.y;
    odom_position_.z = msg->pose.pose.position.z;
    odom_orientation_.x = msg->pose.pose.orientation.x;
    odom_orientation_.y = msg->pose.pose.orientation.y;
    odom_orientation_.z = msg->pose.pose.orientation.z;
    odom_orientation_.w = msg->pose.pose.orientation.w;
    has_odom_ = true;
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    odom_position_.x = msg->pose.position.x;
    odom_position_.y = msg->pose.position.y;
    odom_position_.z = msg->pose.position.z;
    odom_orientation_.x = msg->pose.orientation.x;
    odom_orientation_.y = msg->pose.orientation.y;
    odom_orientation_.z = msg->pose.orientation.z;
    odom_orientation_.w = msg->pose.orientation.w;
    has_odom_ = true;
  }

  Point3 current_query_position() const
  {
    if (use_odom_ && has_odom_) {
      return odom_position_;
    }
    return query_position_;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (transform_to_odom_frame_ && !has_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for odometry before transforming camera cloud to %s.",
        output_frame_id_.c_str());
      return;
    }

    if (msg->is_bigendian) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Big-endian PointCloud2 is not supported by this minimal L4 mapper.");
      return;
    }

    const auto * x_field = find_field(*msg, "x");
    const auto * y_field = find_field(*msg, "y");
    const auto * z_field = find_field(*msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 must contain x/y/z fields.");
      return;
    }

    const Point3 query = current_query_position();
    const double radius2 = local_radius_m_ * local_radius_m_;
    const auto stamp = now();

    std::size_t parsed = 0;
    std::size_t accepted = 0;
    const std::size_t rows = std::max<std::uint32_t>(msg->height, 1);
    const std::size_t cols = msg->width;
    const std::size_t max_points = static_cast<std::size_t>(std::max(max_input_points_, 1));

    for (std::size_t row = 0; row < rows && parsed < max_points; ++row) {
      for (std::size_t col = 0; col < cols && parsed < max_points; ++col) {
        ++parsed;
        const std::size_t base = row * msg->row_step + col * msg->point_step;
        double x{0.0};
        double y{0.0};
        double z{0.0};
        if (!read_field_value(*msg, *x_field, base, x) ||
            !read_field_value(*msg, *y_field, base, y) ||
            !read_field_value(*msg, *z_field, base, z)) {
          continue;
        }
        const Point3 point{x, y, z};
        if (!finite_point(point)) {
          continue;
        }
        const Point3 query_point = transform_input_point(point);

        const Point3 delta = query_point - query;
        const double dist2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
        if (dist2 > radius2) {
          continue;
        }

        VoxelKey key{
          static_cast<int>(std::floor(query_point.x / voxel_size_m_)),
          static_cast<int>(std::floor(query_point.y / voxel_size_m_)),
          static_cast<int>(std::floor(query_point.z / voxel_size_m_))};

        auto [it, inserted] = voxel_map_.insert({key, {query_point, stamp}});
        if (!inserted) {
          it->second.point = query_point;
          it->second.stamp = stamp;
        }
        ++accepted;
      }
    }

    rebuild_local_points(query, stamp);

    cloud_header_ = msg->header;
    if (transform_to_odom_frame_) {
      cloud_header_.frame_id = output_frame_id_;
    }
    if (cloud_header_.frame_id.empty()) {
      cloud_header_.frame_id = output_frame_id_;
    }
    last_cloud_time_ = stamp;
    has_cloud_ = true;
    input_point_count_ = parsed;
    accepted_point_count_ = accepted;

    local_cloud_pub_->publish(make_cloud_msg(local_points_, cloud_header_));
  }

  void rebuild_local_points(const Point3 & query, const rclcpp::Time & stamp)
  {
    const double radius2 = local_radius_m_ * local_radius_m_;

    for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
      const double age = (stamp - it->second.stamp).seconds();
      const Point3 delta = it->second.point - query;
      const double dist2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
      if ((map_memory_s_ > 0.0 && age > map_memory_s_) || dist2 > radius2) {
        it = voxel_map_.erase(it);
      } else {
        ++it;
      }
    }

    if (voxel_map_.size() > static_cast<std::size_t>(max_map_voxels_)) {
      const std::size_t target = static_cast<std::size_t>(max_map_voxels_);
      while (voxel_map_.size() > target) {
        auto oldest = voxel_map_.begin();
        for (auto it = voxel_map_.begin(); it != voxel_map_.end(); ++it) {
          if (it->second.stamp < oldest->second.stamp) {
            oldest = it;
          }
        }
        voxel_map_.erase(oldest);
      }
    }

    local_points_.clear();
    local_points_.reserve(voxel_map_.size());
    for (const auto & entry : voxel_map_) {
      local_points_.push_back(entry.second.point);
    }
  }

  Point3 transform_input_point(const Point3 & point) const
  {
    if (!transform_to_odom_frame_) {
      return point;
    }
    if (!has_odom_) {
      return point;
    }
    const Point3 body_point = point + camera_offset_;
    return odom_position_ + rotate_by_quat(body_point, odom_orientation_);
  }

  void query_timer_callback()
  {
    const auto now_time = now();
    if (!has_cloud_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for PointCloud2 on %s", pointcloud_topic_.c_str());
      return;
    }

    const double cloud_age = (now_time - last_cloud_time_).seconds();
    if (cloud_age > stale_cloud_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Point cloud stale: age=%.2fs > %.2fs", cloud_age, stale_cloud_timeout_s_);
      return;
    }

    const Point3 query = current_query_position();
    if (use_odom_ && !has_odom_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for odometry on %s, using fallback query=(%.2f, %.2f, %.2f)",
        odom_topic_.c_str(), query.x, query.y, query.z);
    }

    if (local_points_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No local points after radius/voxel filtering.");
      return;
    }

    Point3 nearest = local_points_.front();
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (const auto & point : local_points_) {
      const double d = norm(point - query);
      if (d < nearest_dist) {
        nearest_dist = d;
        nearest = point;
      }
    }

    Point3 normal = query - nearest;
    const double normal_norm = norm(normal);
    if (normal_norm > 1e-6) {
      normal.x /= normal_norm;
      normal.y /= normal_norm;
      normal.z /= normal_norm;
    } else {
      normal = {0.0, 0.0, 1.0};
    }

    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = static_cast<float>(nearest_dist);
    nearest_distance_pub_->publish(distance_msg);

    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = cloud_header_;
    point_msg.header.stamp = now_time;
    point_msg.point.x = nearest.x;
    point_msg.point.y = nearest.y;
    point_msg.point.z = nearest.z;
    nearest_point_pub_->publish(point_msg);

    geometry_msgs::msg::Vector3Stamped normal_msg;
    normal_msg.header = point_msg.header;
    normal_msg.vector.x = normal.x;
    normal_msg.vector.y = normal.y;
    normal_msg.vector.z = normal.z;
    nearest_normal_pub_->publish(normal_msg);

    if ((now_time - last_log_time_).seconds() >= log_period_s_) {
      RCLCPP_INFO(
        get_logger(),
        "L4 query pos=(%.2f, %.2f, %.2f) input=%zu local=%zu nearest=%.2f point=(%.2f, %.2f, %.2f) normal=(%.2f, %.2f, %.2f)",
        query.x, query.y, query.z, input_point_count_, local_points_.size(), nearest_dist,
        nearest.x, nearest.y, nearest.z, normal.x, normal.y, normal.z);
      last_log_time_ = now_time;
    }
  }

  std::string pointcloud_topic_;
  std::string odom_topic_;
  std::string pose_topic_;
  bool use_odom_{true};
  Point3 query_position_{};
  Point3 odom_position_{};
  Quaternion odom_orientation_{};
  bool has_odom_{false};
  double local_radius_m_{6.0};
  double voxel_size_m_{0.15};
  int max_input_points_{200000};
  double map_memory_s_{90.0};
  int max_map_voxels_{80000};
  double stale_cloud_timeout_s_{2.0};
  double query_rate_hz_{10.0};
  double log_period_s_{1.0};
  std::string output_frame_id_{"map"};
  bool transform_to_odom_frame_{false};
  Point3 camera_offset_{0.22, 0.0, 0.06};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr nearest_distance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr nearest_normal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_pub_;
  rclcpp::TimerBase::SharedPtr query_timer_;

  std::vector<Point3> local_points_;
  std::unordered_map<VoxelKey, VoxelEntry, VoxelHash> voxel_map_;
  std_msgs::msg::Header cloud_header_;
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
  bool has_cloud_{false};
  std::size_t input_point_count_{0};
  std::size_t accepted_point_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<L4PointcloudMapper>());
  rclcpp::shutdown();
  return 0;
}
