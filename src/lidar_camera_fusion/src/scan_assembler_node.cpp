#include "lidar_camera_fusion/scan_assembler_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace std::chrono_literals;

namespace lidar_camera_fusion
{

ScanAssemblerNode::ScanAssemblerNode(const rclcpp::NodeOptions & options)
: Node("scan_assembler_node", options)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  declare_parameter("target_frame", "base_link");
  declare_parameter("scan_topic",   "/scan");
  declare_parameter("max_points",   500000);
  declare_parameter("publish_rate", 2.0);

  target_frame_ = get_parameter("target_frame").as_string();
  scan_topic_   = get_parameter("scan_topic").as_string();
  max_points_   = get_parameter("max_points").as_int();
  publish_rate_ = get_parameter("publish_rate").as_double();

  // ── TF2 ───────────────────────────────────────────────────────────────────
  // spin_thread=true gives the TransformListener its own dedicated thread so
  // it can receive and buffer /tf messages even while scan_callback is
  // blocked inside lookupTransform(... 100 ms timeout).  Without this,
  // both the listener and the lookup share the single executor thread and
  // the lookup deadlocks — it waits for TF data that can never arrive
  // because the thread is already occupied waiting.
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  // ── Pre-allocate circular buffer ──────────────────────────────────────────
  point_buffer_.resize(static_cast<std::size_t>(max_points_));

  // ── Subscription ─────────────────────────────────────────────────────────
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanAssemblerNode::scan_callback, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────────────────
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/scanner/assembled_cloud", rclcpp::SensorDataQoS());

  // ── Clear service ─────────────────────────────────────────────────────────
  clear_srv_ = create_service<std_srvs::srv::Trigger>(
    "/scanner/clear_cloud",
    std::bind(
      &ScanAssemblerNode::clear_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // ── Publish timer ─────────────────────────────────────────────────────────
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  publish_timer_ = create_wall_timer(
    period, std::bind(&ScanAssemblerNode::publish_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "scan_assembler_node: '%s' → '%s', max_points=%d, rate=%.1f Hz",
    scan_topic_.c_str(), target_frame_.c_str(), max_points_, publish_rate_);
}

// ── scan_callback ──────────────────────────────────────────────────────────

void ScanAssemblerNode::scan_callback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Step 1: project 2D scan into a PointCloud2 in the scan's own frame.
  // projectLaser never needs TF — it always succeeds immediately.
  sensor_msgs::msg::PointCloud2 cloud_laser;
  laser_projection_.projectLaser(
    *msg, cloud_laser, -1.0, laser_geometry::channel_option::None);

  // Step 2: look up the transform at the scan's exact timestamp.
  // We wait up to 100 ms for the TF data to arrive — this handles the
  // typical robot_state_publisher latency (~7–80 ms) that causes the
  // "extrapolation into the future" error when using transformLaserScanToPointCloud.
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      target_frame_,
      msg->header.frame_id,
      rclcpp::Time(msg->header.stamp),
      rclcpp::Duration(0, 100'000'000));  // 100 ms timeout
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF transform failed: %s", ex.what());
    return;
  }

  // Step 3: apply the transform to move the cloud into target_frame.
  sensor_msgs::msg::PointCloud2 cloud_out;
  tf2::doTransform(cloud_laser, cloud_out, tf_stamped);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  append_cloud(cloud_out);
}

// ── append_cloud ───────────────────────────────────────────────────────────

void ScanAssemblerNode::append_cloud(const sensor_msgs::msg::PointCloud2 & cloud_in)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

  const std::size_t capacity = static_cast<std::size_t>(max_points_);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      continue;
    }
    point_buffer_[write_index_].x = *iter_x;
    point_buffer_[write_index_].y = *iter_y;
    point_buffer_[write_index_].z = *iter_z;
    write_index_ = (write_index_ + 1) % capacity;
    if (active_count_ < capacity) {
      ++active_count_;
    }
    // When active_count_ == capacity the write head wraps and overwrites oldest
  }
}

// ── publish_callback ───────────────────────────────────────────────────────

void ScanAssemblerNode::publish_callback()
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (active_count_ == 0) {
    return;
  }

  const std::size_t capacity = static_cast<std::size_t>(max_points_);

  sensor_msgs::msg::PointCloud2 out_msg;
  sensor_msgs::PointCloud2Modifier modifier(out_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(active_count_);

  out_msg.header.stamp    = now();
  out_msg.header.frame_id = target_frame_;
  out_msg.is_dense        = false;

  sensor_msgs::PointCloud2Iterator<float> out_x(out_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(out_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(out_msg, "z");

  if (active_count_ < capacity) {
    // Buffer not yet full: points are contiguous from index 0
    for (std::size_t i = 0; i < active_count_; ++i, ++out_x, ++out_y, ++out_z) {
      *out_x = point_buffer_[i].x;
      *out_y = point_buffer_[i].y;
      *out_z = point_buffer_[i].z;
    }
  } else {
    // Buffer full: oldest point is at write_index_
    for (std::size_t k = 0; k < capacity; ++k, ++out_x, ++out_y, ++out_z) {
      const std::size_t idx = (write_index_ + k) % capacity;
      *out_x = point_buffer_[idx].x;
      *out_y = point_buffer_[idx].y;
      *out_z = point_buffer_[idx].z;
    }
  }

  cloud_pub_->publish(out_msg);
}

// ── clear_callback ─────────────────────────────────────────────────────────

void ScanAssemblerNode::clear_callback(
  std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  write_index_  = 0;
  active_count_ = 0;
  response->success = true;
  response->message = "Assembled cloud cleared";
  RCLCPP_INFO(get_logger(), "Assembled cloud cleared");
}

}  // namespace lidar_camera_fusion

// ── main ───────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_camera_fusion::ScanAssemblerNode>());
  rclcpp::shutdown();
  return 0;
}
