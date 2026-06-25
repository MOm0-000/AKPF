#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

std::string headerValue(
  const gz::msgs::Header & header,
  const std::string & key)
{
  for (const auto & item : header.data()) {
    if (item.key() == key && item.value_size() > 0) {
      return item.value(0);
    }
  }
  return {};
}

uint8_t toRosDatatype(const gz::msgs::PointCloudPacked_Field & field)
{
  const int gz_type = static_cast<int>(field.datatype());
  if (gz_type < static_cast<int>(gz::msgs::PointCloudPacked_Field::INT8) ||
    gz_type > static_cast<int>(gz::msgs::PointCloudPacked_Field::FLOAT64))
  {
    return 0;
  }

  return static_cast<uint8_t>(gz_type + 1);
}

const gz::msgs::PointCloudPacked_Field * findField(
  const gz::msgs::PointCloudPacked & msg,
  const std::string & name)
{
  for (const auto & field : msg.field()) {
    if (field.name() == name) {
      return &field;
    }
  }
  return nullptr;
}

bool readFloat32(
  const gz::msgs::PointCloudPacked & msg,
  const gz::msgs::PointCloudPacked_Field & field,
  const size_t base,
  float & value)
{
  if (field.datatype() != gz::msgs::PointCloudPacked_Field::FLOAT32) {
    return false;
  }
  const size_t offset = base + field.offset();
  if (offset + sizeof(float) > msg.data().size()) {
    return false;
  }
  std::memcpy(&value, msg.data().data() + offset, sizeof(float));
  return std::isfinite(value);
}

void appendFloat(std::vector<uint8_t> & data, const float value)
{
  const auto * raw = reinterpret_cast<const uint8_t *>(&value);
  data.insert(data.end(), raw, raw + sizeof(float));
}

}  // namespace

class GzPointCloudBridgeNode final : public rclcpp::Node
{
public:
  GzPointCloudBridgeNode()
  : Node("l4_gz_pointcloud_bridge")
  {
    gz_topic_ = declare_parameter<std::string>("gz_topic", "/l4/depth_camera/points");
    ros_topic_ = declare_parameter<std::string>("ros_topic", "/l4/depth_camera/points");
    frame_id_override_ = declare_parameter<std::string>("frame_id_override", "");
    repack_xyz_ = declare_parameter<bool>("repack_xyz", true);
    sample_step_ = std::max<int>(1, declare_parameter<int>("sample_step", 4));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ros_topic_,
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

    if (!gz_node_.Subscribe(gz_topic_, &GzPointCloudBridgeNode::onCloud, this)) {
      throw std::runtime_error("failed to subscribe to Gazebo topic " + gz_topic_);
    }

    RCLCPP_INFO(
      get_logger(),
      "L4 Gazebo pointcloud bridge started: %s -> %s",
      gz_topic_.c_str(),
      ros_topic_.c_str());
  }

private:
  void onCloud(const gz::msgs::PointCloudPacked & gz_msg)
  {
    sensor_msgs::msg::PointCloud2 ros_msg;

    ros_msg.header.stamp.sec = gz_msg.header().stamp().sec();
    ros_msg.header.stamp.nanosec = gz_msg.header().stamp().nsec();
    ros_msg.header.frame_id = frame_id_override_.empty()
      ? headerValue(gz_msg.header(), "frame_id")
      : frame_id_override_;

    if (repack_xyz_) {
      if (!repackXyz(gz_msg, ros_msg)) {
        return;
      }
    } else {
      copyPacked(gz_msg, ros_msg);
    }

    pub_->publish(std::move(ros_msg));

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Bridged PointCloud2: width=%u height=%u point_step=%u bytes=%zu frame=%s",
      ros_msg.width,
      ros_msg.height,
      ros_msg.point_step,
      ros_msg.data.size(),
      ros_msg.header.frame_id.c_str());
  }

  bool repackXyz(
    const gz::msgs::PointCloudPacked & gz_msg,
    sensor_msgs::msg::PointCloud2 & ros_msg)
  {
    const auto * x_field = findField(gz_msg, "x");
    const auto * y_field = findField(gz_msg, "y");
    const auto * z_field = findField(gz_msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Gazebo PointCloudPacked must contain x/y/z fields.");
      return false;
    }

    ros_msg.fields.resize(3);
    ros_msg.fields[0].name = "x";
    ros_msg.fields[0].offset = 0;
    ros_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_msg.fields[0].count = 1;
    ros_msg.fields[1].name = "y";
    ros_msg.fields[1].offset = 4;
    ros_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_msg.fields[1].count = 1;
    ros_msg.fields[2].name = "z";
    ros_msg.fields[2].offset = 8;
    ros_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_msg.fields[2].count = 1;

    ros_msg.height = 1;
    ros_msg.is_bigendian = false;
    ros_msg.point_step = 12;
    ros_msg.is_dense = false;
    ros_msg.data.clear();

    for (uint32_t row = 0; row < gz_msg.height(); row += static_cast<uint32_t>(sample_step_)) {
      for (uint32_t col = 0; col < gz_msg.width(); col += static_cast<uint32_t>(sample_step_)) {
        const size_t base =
          static_cast<size_t>(row) * gz_msg.row_step() +
          static_cast<size_t>(col) * gz_msg.point_step();
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
        if (!readFloat32(gz_msg, *x_field, base, x) ||
          !readFloat32(gz_msg, *y_field, base, y) ||
          !readFloat32(gz_msg, *z_field, base, z))
        {
          continue;
        }
        appendFloat(ros_msg.data, x);
        appendFloat(ros_msg.data, y);
        appendFloat(ros_msg.data, z);
      }
    }

    ros_msg.width = static_cast<uint32_t>(ros_msg.data.size() / ros_msg.point_step);
    ros_msg.row_step = ros_msg.width * ros_msg.point_step;
    return ros_msg.width > 0;
  }

  bool copyPacked(
    const gz::msgs::PointCloudPacked & gz_msg,
    sensor_msgs::msg::PointCloud2 & ros_msg)
  {
    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();
    ros_msg.is_bigendian = gz_msg.is_bigendian();
    ros_msg.point_step = gz_msg.point_step();
    ros_msg.row_step = gz_msg.row_step();
    ros_msg.is_dense = gz_msg.is_dense();

    ros_msg.fields.reserve(static_cast<size_t>(gz_msg.field_size()));
    for (const auto & gz_field : gz_msg.field()) {
      sensor_msgs::msg::PointField ros_field;
      ros_field.name = gz_field.name();
      ros_field.offset = gz_field.offset();
      ros_field.datatype = toRosDatatype(gz_field);
      ros_field.count = std::max<uint32_t>(1, gz_field.count());
      if (ros_field.datatype == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Skipping point cloud with unsupported Gazebo field datatype %d for field %s",
          static_cast<int>(gz_field.datatype()),
          gz_field.name().c_str());
        return false;
      }
      ros_msg.fields.push_back(std::move(ros_field));
    }

    const auto & data = gz_msg.data();
    ros_msg.data.assign(data.begin(), data.end());
    return true;
  }

  std::string gz_topic_;
  std::string ros_topic_;
  std::string frame_id_override_;
  bool repack_xyz_{true};
  int sample_step_{4};
  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GzPointCloudBridgeNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("l4_gz_pointcloud_bridge"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
