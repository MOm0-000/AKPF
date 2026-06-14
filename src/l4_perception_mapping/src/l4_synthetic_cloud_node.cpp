#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace {

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Box
{
  std::string name;
  Point3 center;
  Point3 size;
};

std::vector<Box> room_boundaries()
{
  return {
    {"l2_wall_x_positive", {5.0, 0.0, 1.5}, {0.12, 8.0, 3.0}},
    {"l2_wall_x_negative", {-5.0, 0.0, 1.5}, {0.12, 8.0, 3.0}},
    {"l2_wall_y_positive", {0.0, 4.0, 1.5}, {10.0, 0.12, 3.0}},
    {"l2_wall_y_negative", {0.0, -4.0, 1.5}, {10.0, 0.12, 3.0}},
  };
}

std::vector<Box> scenario_boxes(const std::string & scenario, bool include_boundaries)
{
  std::vector<Box> boxes;
  if (include_boundaries) {
    boxes = room_boundaries();
  }

  if (scenario == "S1_single_front_obstacle") {
    boxes.push_back({"front_block", {3.00, 0.00, 1.25}, {0.70, 1.60, 2.50}});
  } else if (scenario == "S2_narrow_gate") {
    boxes.push_back({"gate_left_pillar", {2.50, 0.90, 1.30}, {0.45, 0.45, 2.60}});
    boxes.push_back({"gate_right_pillar", {2.50, -0.90, 1.30}, {0.45, 0.45, 2.60}});
    boxes.push_back({"gate_top_reference", {2.50, 0.00, 2.65}, {0.50, 2.25, 0.12}});
  } else if (scenario == "S3_corridor") {
    boxes.push_back({"corridor_left_wall", {2.20, 1.20, 1.35}, {5.80, 0.16, 2.70}});
    boxes.push_back({"corridor_right_wall", {2.20, -1.20, 1.35}, {5.80, 0.16, 2.70}});
  } else if (scenario == "S4_table_or_low_obstacle") {
    boxes.push_back({"low_table_top", {2.60, 0.00, 0.80}, {1.70, 1.10, 0.16}});
    boxes.push_back({"low_table_leg_1", {1.90, 0.45, 0.40}, {0.12, 0.12, 0.80}});
    boxes.push_back({"low_table_leg_2", {3.30, 0.45, 0.40}, {0.12, 0.12, 0.80}});
    boxes.push_back({"low_table_leg_3", {1.90, -0.45, 0.40}, {0.12, 0.12, 0.80}});
    boxes.push_back({"low_table_leg_4", {3.30, -0.45, 0.40}, {0.12, 0.12, 0.80}});
  } else if (scenario == "S5_corner") {
    boxes.push_back({"corner_vertical_wall", {2.00, -0.70, 1.35}, {0.18, 2.60, 2.70}});
    boxes.push_back({"corner_horizontal_wall", {3.05, 1.30, 1.35}, {2.30, 0.18, 2.70}});
    boxes.push_back({"corner_inner_block", {2.55, 0.55, 1.25}, {0.55, 0.55, 2.50}});
  } else if (scenario == "S6_cluttered_boxes") {
    boxes.push_back({"clutter_box_1", {1.60, 1.35, 1.25}, {0.50, 0.60, 2.50}});
    boxes.push_back({"clutter_box_2", {2.40, -1.35, 1.25}, {0.55, 0.60, 2.50}});
    boxes.push_back({"clutter_box_3", {3.05, 1.35, 1.25}, {0.50, 0.60, 2.50}});
    boxes.push_back({"clutter_box_4", {3.65, -0.85, 1.25}, {0.45, 0.55, 2.50}});
    boxes.push_back({"clutter_box_5", {3.90, 1.95, 1.25}, {0.40, 0.50, 2.50}});
  } else if (scenario == "S8_vertical_constraint") {
    boxes.push_back({"low_ceiling_slab", {2.65, 0.00, 2.45}, {2.50, 2.30, 0.30}});
    boxes.push_back({"ceiling_left_support", {2.05, 1.20, 1.25}, {0.18, 0.18, 2.50}});
    boxes.push_back({"ceiling_right_support", {3.25, -1.20, 1.25}, {0.18, 0.18, 2.50}});
  } else if (scenario == "S9_multi_corner") {
    boxes.push_back({"multi_corner_wall_1", {1.35, -0.95, 1.35}, {0.18, 2.20, 2.70}});
    boxes.push_back({"multi_corner_wall_2", {2.60, 1.05, 1.35}, {2.20, 0.18, 2.70}});
    boxes.push_back({"multi_corner_wall_3", {3.45, 2.85, 1.35}, {0.18, 0.95, 2.70}});
    boxes.push_back({"multi_corner_block", {2.55, -0.05, 1.25}, {0.55, 0.55, 2.50}});
  }

  return boxes;
}

void append_grid_face(
  std::vector<Point3> & points,
  const Box & box,
  int fixed_axis,
  double fixed_value,
  int axis_a,
  int axis_b,
  double step)
{
  const std::array<double, 3> center{box.center.x, box.center.y, box.center.z};
  const std::array<double, 3> size{box.size.x, box.size.y, box.size.z};
  const int n_a = std::max(1, static_cast<int>(std::ceil(size[axis_a] / step)));
  const int n_b = std::max(1, static_cast<int>(std::ceil(size[axis_b] / step)));

  for (int i = 0; i <= n_a; ++i) {
    for (int j = 0; j <= n_b; ++j) {
      std::array<double, 3> xyz{center[0], center[1], center[2]};
      xyz[fixed_axis] = center[fixed_axis] + fixed_value;
      xyz[axis_a] = center[axis_a] - 0.5 * size[axis_a] + size[axis_a] * static_cast<double>(i) / n_a;
      xyz[axis_b] = center[axis_b] - 0.5 * size[axis_b] + size[axis_b] * static_cast<double>(j) / n_b;
      points.push_back({xyz[0], xyz[1], xyz[2]});
    }
  }
}

std::vector<Point3> sample_box_surfaces(const std::vector<Box> & boxes, double step)
{
  std::vector<Point3> points;
  for (const auto & box : boxes) {
    append_grid_face(points, box, 0, -0.5 * box.size.x, 1, 2, step);
    append_grid_face(points, box, 0, 0.5 * box.size.x, 1, 2, step);
    append_grid_face(points, box, 1, -0.5 * box.size.y, 0, 2, step);
    append_grid_face(points, box, 1, 0.5 * box.size.y, 0, 2, step);
    append_grid_face(points, box, 2, -0.5 * box.size.z, 0, 1, step);
    append_grid_face(points, box, 2, 0.5 * box.size.z, 0, 1, step);
  }
  return points;
}

sensor_msgs::msg::PointCloud2 make_cloud_msg(
  const std::vector<Point3> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
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

class L4SyntheticCloud : public rclcpp::Node
{
public:
  L4SyntheticCloud()
  : Node("l4_synthetic_cloud")
  {
    scenario_ = declare_parameter<std::string>("scenario", "S1_single_front_obstacle");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    sample_step_m_ = declare_parameter<double>("sample_step_m", 0.18);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    include_boundaries_ = declare_parameter<bool>("include_boundaries", true);

    if (sample_step_m_ <= 0.0) {
      sample_step_m_ = 0.18;
    }
    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 5.0;
    }

    boxes_ = scenario_boxes(scenario_, include_boundaries_);
    if (boxes_.empty()) {
      RCLCPP_ERROR(get_logger(), "Unknown scenario or empty obstacle set: %s", scenario_.c_str());
      throw std::runtime_error("empty synthetic scenario");
    }

    points_ = sample_box_surfaces(boxes_, sample_step_m_);
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/l4/points", cloud_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&L4SyntheticCloud::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "L4 synthetic cloud started: scenario=%s boxes=%zu points=%zu step=%.2f frame=%s",
      scenario_.c_str(), boxes_.size(), points_.size(), sample_step_m_, frame_id_.c_str());
  }

private:
  void timer_callback()
  {
    publisher_->publish(make_cloud_msg(points_, frame_id_, now()));
  }

  std::string scenario_;
  std::string frame_id_;
  double sample_step_m_{0.18};
  double publish_rate_hz_{5.0};
  bool include_boundaries_{true};
  std::vector<Box> boxes_;
  std::vector<Point3> points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<L4SyntheticCloud>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("l4_synthetic_cloud"), "Fatal error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
