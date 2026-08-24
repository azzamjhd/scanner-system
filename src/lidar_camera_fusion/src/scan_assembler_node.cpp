#include "lidar_camera_fusion/scan_assembler_node.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <functional>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
  declare_parameter("position_topic", "/current_position");
  declare_parameter("output_dir",   ".");
  declare_parameter("joint_name",   "gantry_joint");
  declare_parameter("max_points",   500000);
  declare_parameter("publish_rate", 2.0);

  target_frame_ = get_parameter("target_frame").as_string();
  scan_topic_   = get_parameter("scan_topic").as_string();
  position_topic_ = get_parameter("position_topic").as_string();
  output_dir_   = get_parameter("output_dir").as_string();
  joint_name_   = get_parameter("joint_name").as_string();
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

  // ── Subscriptions ────────────────────────────────────────────────────────
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanAssemblerNode::scan_callback, this, std::placeholders::_1));

  // Gantry bridge — converts configured geometry_msgs/Point position topic (x mm)
  // to /joint_states (m) so robot_state_publisher keeps the TF tree in sync.
  position_sub_ = create_subscription<geometry_msgs::msg::Point>(
    position_topic_, 10,
    std::bind(&ScanAssemblerNode::position_callback, this, std::placeholders::_1));

  // ── Publishers ─────────────────────────────────────────────────────────
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/scanner/assembled_cloud", rclcpp::SensorDataQoS());

  scan_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/scanner/scan_cloud", rclcpp::SensorDataQoS());

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // Latched Bool topic so cloud_colorizer_node always knows the current state
  // even if it starts after scan_assembler_node.
  scanning_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/scanner/is_scanning",
    rclcpp::QoS(1).transient_local());

  // ── Clear service ─────────────────────────────────────────────────────────
  clear_srv_ = create_service<std_srvs::srv::Trigger>(
    "/scanner/clear_cloud",
    std::bind(
      &ScanAssemblerNode::clear_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // ── Start / Stop services ──────────────────────────────────────────────────
  start_srv_ = create_service<std_srvs::srv::Trigger>(
    "/scanner/start",
    std::bind(
      &ScanAssemblerNode::start_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "/scanner/stop",
    std::bind(
      &ScanAssemblerNode::stop_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // ── Publish timer ─────────────────────────────────────────────────────────
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));
  publish_timer_ = create_wall_timer(
    period, std::bind(&ScanAssemblerNode::publish_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "scan_assembler_node: scan='%s' → '%s', position='%s', joint='%s', max_points=%d, rate=%.1f Hz, output_dir='%s'",
    scan_topic_.c_str(), target_frame_.c_str(), position_topic_.c_str(), joint_name_.c_str(),
    max_points_, publish_rate_, output_dir_.c_str());
}

// ── position_callback ───────────────────────────────────────────────────────
// Converts configured position_topic Point.x (mm) → /joint_states (m).
// robot_state_publisher consumes /joint_states to animate gantry_joint,
// which is the dynamic edge in the TF tree that gives each scan ring its
// correct 3-D position in base_link space.

void ScanAssemblerNode::position_callback(
  geometry_msgs::msg::Point::ConstSharedPtr msg)
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.name         = {joint_name_};
  js.position     = {static_cast<double>(msg->x) / 1000.0};   // mm → m
  joint_pub_->publish(js);
}

// ── scan_callback ──────────────────────────────────────────────────────────

void ScanAssemblerNode::scan_callback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Guard: discard scans when the scanner is not actively running.
  // is_scanning_ is std::atomic<bool>, so this check is lock-free.
  if (!is_scanning_) {
    return;
  }

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
  cloud_out.header.stamp = msg->header.stamp;
  cloud_out.header.frame_id = target_frame_;

  // Publish the per-scan cloud for colorization (one ring at a time).
  scan_cloud_pub_->publish(cloud_out);

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

// ── clear_buffer_locked ────────────────────────────────────────────────────
// Internal helper — caller MUST hold buffer_mutex_.

void ScanAssemblerNode::clear_buffer_locked()
{
  write_index_  = 0;
  active_count_ = 0;
}

// ── clear_callback ─────────────────────────────────────────────────────────

void ScanAssemblerNode::clear_callback(
  std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  clear_buffer_locked();
  response->success = true;
  response->message = "Assembled cloud cleared";
  RCLCPP_INFO(get_logger(), "Assembled cloud cleared");
}

// ── start_callback ─────────────────────────────────────────────────────────
// Clears the accumulation buffer first, then opens the gate so that every
// new scan session always starts with a completely empty 3-D space.

void ScanAssemblerNode::start_callback(
  std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    clear_buffer_locked();
  }
  is_scanning_ = true;
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    scanning_pub_->publish(msg);
  }
  response->success = true;
  response->message = "Scanner started; accumulation buffer cleared";
  RCLCPP_INFO(get_logger(), "Scan started — buffer cleared, accumulation active");
}

// ── save_pcd_locked ────────────────────────────────────────────────────────
// Caller MUST hold buffer_mutex_.

void ScanAssemblerNode::save_pcd_locked(const std::string & filepath)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.reserve(active_count_);

  const std::size_t capacity = static_cast<std::size_t>(max_points_);

  if (active_count_ < capacity) {
    // Buffer not yet full — points are contiguous from index 0
    for (std::size_t i = 0; i < active_count_; ++i) {
      pcl_cloud.emplace_back(
        point_buffer_[i].x, point_buffer_[i].y, point_buffer_[i].z);
    }
  } else {
    // Buffer full/wrapped — oldest point is at write_index_
    for (std::size_t k = 0; k < capacity; ++k) {
      const std::size_t idx = (write_index_ + k) % capacity;
      pcl_cloud.emplace_back(
        point_buffer_[idx].x, point_buffer_[idx].y, point_buffer_[idx].z);
    }
  }

  pcl_cloud.width    = static_cast<std::uint32_t>(pcl_cloud.size());
  pcl_cloud.height   = 1;
  pcl_cloud.is_dense = true;

  if (pcl::io::savePCDFileBinary(filepath, pcl_cloud) != 0) {
    throw std::runtime_error("pcl::io::savePCDFileBinary failed for: " + filepath);
  }
}

// ── stop_callback ──────────────────────────────────────────────────────────

void ScanAssemblerNode::stop_callback(
  std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  is_scanning_ = false;

  // ── Build timestamped output path ─────────────────────────────────────────
  const auto now_tp = std::chrono::system_clock::now();
  const auto now_tt = std::chrono::system_clock::to_time_t(now_tp);
  std::tm tm_buf{};
  localtime_r(&now_tt, &tm_buf);
  char time_str[32];
  std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm_buf);

  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
  if (ec) {
    const std::string msg =
      "Scanner stopped but could not create output dir '" +
      output_dir_ + "': " + ec.message();
    response->success = false;
    response->message = msg;
    RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
    return;
  }

  const std::string filepath =
    (std::filesystem::path(output_dir_) /
     ("scan_" + std::string(time_str) + ".pcd")).string();

  // ── Serialise buffer and write to disk (under lock) ───────────────────────
  std::size_t saved_count = 0;
  std::string save_error;

  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    saved_count = active_count_;
    if (saved_count > 0) {
      try {
        save_pcd_locked(filepath);
      } catch (const std::exception & ex) {
        save_error = ex.what();
      }
    }
  }

  // Notify cloud_colorizer_node (and any other listeners) that scanning stopped.
  // Publish before filling in the response so the colorizer starts saving in
  // parallel while we finish composing the service reply.
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    scanning_pub_->publish(msg);
  }

  if (saved_count == 0) {
    response->success = true;
    response->message = "Scanner stopped; buffer was empty — no .pcd written";
    RCLCPP_WARN(get_logger(), "Scan stopped — buffer is empty, no .pcd written");
  } else if (!save_error.empty()) {
    response->success = false;
    response->message = "Scanner stopped but failed to save PCD: " + save_error;
    RCLCPP_ERROR(
      get_logger(), "Scan stopped — failed to write '%s': %s",
      filepath.c_str(), save_error.c_str());
  } else {
    response->success = true;
    response->message =
      "Scanner stopped; " + std::to_string(saved_count) +
      " points saved to " + filepath;
    RCLCPP_INFO(
      get_logger(), "Scan stopped — %zu points saved → %s",
      saved_count, filepath.c_str());
  }
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
