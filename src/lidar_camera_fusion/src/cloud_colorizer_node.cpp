#include "lidar_camera_fusion/cloud_colorizer_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <functional>
#include <stdexcept>

#include <opencv2/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace lidar_camera_fusion
{

CloudColorizerNode::CloudColorizerNode(const rclcpp::NodeOptions & options)
: Node("cloud_colorizer_node", options)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  declare_parameter("optical_frame",    "camera_optical_frame");
  declare_parameter("output_dir",       ".");
  declare_parameter("image_topic",      "/image_rect_color");
  declare_parameter("camera_info_topic", "/camera_info");
  declare_parameter("queue_size",       10);
  declare_parameter("approx_time_slop", 0.1);

  optical_frame_     = get_parameter("optical_frame").as_string();
  output_dir_        = get_parameter("output_dir").as_string();
  image_topic_       = get_parameter("image_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  queue_size_        = get_parameter("queue_size").as_int();
  approx_time_slop_  = get_parameter("approx_time_slop").as_double();

  // ── TF2 ───────────────────────────────────────────────────────────────────
  // spin_thread=true: same reasoning as scan_assembler_node — the sync
  // callback can block in lookupTransform for up to 100 ms, so TF must
  // be received on a dedicated thread rather than the shared executor thread.
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  // ── message_filters subscriptions ─────────────────────────────────────────
  cloud_sub_.subscribe(this, "/scanner/scan_cloud", rmw_qos_profile_sensor_data);
  image_sub_.subscribe(this, image_topic_,       rmw_qos_profile_sensor_data);
  info_sub_ .subscribe(this, camera_info_topic_, rmw_qos_profile_default);

  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(static_cast<uint32_t>(queue_size_)),
    cloud_sub_, image_sub_, info_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approx_time_slop_));
  sync_->registerCallback(
    std::bind(
      &CloudColorizerNode::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // ── Publisher ─────────────────────────────────────────────────────────
  colored_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/scanner/colored_cloud", rclcpp::SensorDataQoS());

  // ── Scanning-state subscriber ────────────────────────────────────────
  // Receives true (start) / false (stop) from scan_assembler_node.
  // transient_local matches the publisher so a late-starting colorizer
  // immediately gets the current scanning state.
  is_scanning_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/scanner/is_scanning",
    rclcpp::QoS(1).transient_local(),
    std::bind(&CloudColorizerNode::is_scanning_callback, this,
              std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "cloud_colorizer_node: image='%s', camera_info='%s', projecting onto '%s', slop=%.2f s, queue=%d, output_dir='%s'",
    image_topic_.c_str(), camera_info_topic_.c_str(), optical_frame_.c_str(),
    approx_time_slop_, queue_size_, output_dir_.c_str());
}

// ── sync_callback ──────────────────────────────────────────────────────────

void CloudColorizerNode::sync_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr  cloud_msg,
  sensor_msgs::msg::Image::ConstSharedPtr        image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr   info_msg)
{
  // ── 1. TF lookup: cloud frame → optical frame (latest available) ───────────
  // We use rclcpp::Time(0) — "give me the latest transform you have" — instead
  // of the exact image timestamp.  The reason: the assembled cloud is already
  // committed to base_link coordinates at publish time; there is no per-point
  // timestamp that needs microsecond alignment with the image.  The gantry
  // moves at mm/s, so the latest available camera pose (typically < 5 ms old)
  // introduces negligible positional error (< 0.01 mm).  Using the image
  // timestamp instead requires the dynamic base_link→gantry_link TF edge to
  // have data that *brackets* the image stamp, which intermittently fails when
  // the encoder publish rate creates a tiny gap just after the image arrives.
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      optical_frame_,
      cloud_msg->header.frame_id,
      rclcpp::Time(0),                    // latest available — no extrapolation
      rclcpp::Duration(0, 100'000'000));  // 100 ms wait if TF tree not yet ready
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF lookup failed ('%s' → '%s'): %s",
      cloud_msg->header.frame_id.c_str(), optical_frame_.c_str(), ex.what());
    return;
  }

  // ── 2. Batch-transform entire cloud into the optical frame ────────────────
  sensor_msgs::msg::PointCloud2 cloud_optical;
  tf2::doTransform(*cloud_msg, cloud_optical, tf_stamped);

  // ── 3. Decode image ───────────────────────────────────────────────────────
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN(get_logger(), "cv_bridge failed: %s", ex.what());
    return;
  }
  const cv::Mat & img     = cv_ptr->image;
  const int       img_w   = img.cols;
  const int       img_h   = img.rows;

  // ── 4. Rectified camera intrinsics from CameraInfo.p (row-major 3×4) ─────
  // Option A: colorize using image_proc's rectified image.  Projection must
  // use the rectified projection matrix P, not raw K/D.  Do not apply D here;
  // image_proc already removed lens distortion from the image coordinates.
  // p = [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
  const double fx = info_msg->p[0];
  const double fy = info_msg->p[5];
  const double cx = info_msg->p[2];
  const double cy = info_msg->p[6];

  // ── 5. Build colored PCL cloud ────────────────────────────────────────────
  const std::size_t n_pts = cloud_msg->width * cloud_msg->height;

  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  colored_cloud.reserve(n_pts);  // single allocation, no per-point realloc

  // Iterate original cloud for XYZ (stored in output = base_link frame)
  sensor_msgs::PointCloud2ConstIterator<float> orig_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> orig_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> orig_z(*cloud_msg, "z");

  // Iterate transformed cloud for projection (optical frame)
  sensor_msgs::PointCloud2ConstIterator<float> opt_x(cloud_optical, "x");
  sensor_msgs::PointCloud2ConstIterator<float> opt_y(cloud_optical, "y");
  sensor_msgs::PointCloud2ConstIterator<float> opt_z(cloud_optical, "z");

  for (;
    orig_x != orig_x.end();
    ++orig_x, ++orig_y, ++orig_z, ++opt_x, ++opt_y, ++opt_z)
  {
    const float Z = *opt_z;

    // Discard points behind the camera or at origin
    if (Z <= 0.0f) {
      continue;
    }

    // Pinhole projection
    const int u = static_cast<int>(fx * (*opt_x) / Z + cx);
    const int v = static_cast<int>(fy * (*opt_y) / Z + cy);

    // Bounds check
    if (u < 0 || u >= img_w || v < 0 || v >= img_h) {
      continue;
    }

    // Sample BGR → RGB
    const cv::Vec3b bgr = img.at<cv::Vec3b>(v, u);

    pcl::PointXYZRGB pt;
    pt.x = *orig_x;   // original base_link frame
    pt.y = *orig_y;
    pt.z = *orig_z;
    pt.r = bgr[2];
    pt.g = bgr[1];
    pt.b = bgr[0];
    colored_cloud.push_back(pt);
  }

  colored_cloud.width = static_cast<std::uint32_t>(colored_cloud.size());
  colored_cloud.height = 1;
  colored_cloud.is_dense = false;

  if (colored_cloud.empty()) {
    return;
  }

  // ── 6. Append to cache and publish the full colored cloud ─────────────────
  sensor_msgs::msg::PointCloud2 out_msg;
  {
    std::lock_guard<std::mutex> lock(cloud_cache_mutex_);
    if (!has_cloud_cache_) {
      cloud_cache_ = colored_cloud;
      has_cloud_cache_ = true;
    } else {
      cloud_cache_.points.reserve(cloud_cache_.points.size() + colored_cloud.points.size());
      cloud_cache_.points.insert(
        cloud_cache_.points.end(),
        colored_cloud.points.begin(),
        colored_cloud.points.end());
      cloud_cache_.width = static_cast<std::uint32_t>(cloud_cache_.points.size());
      cloud_cache_.height = 1;
      cloud_cache_.is_dense = false;
    }
    pcl::toROSMsg(cloud_cache_, out_msg);
  }

  out_msg.header.stamp    = image_msg->header.stamp;
  out_msg.header.frame_id = cloud_msg->header.frame_id;  // preserve original frame

  colored_pub_->publish(out_msg);
}

// ── is_scanning_callback ────────────────────────────────────────────────────────

void CloudColorizerNode::is_scanning_callback(
  std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (msg->data) {
    // Scan started — discard any cloud from the previous session so we
    // never accidentally save stale data.
    std::lock_guard<std::mutex> lock(cloud_cache_mutex_);
    cloud_cache_.clear();
    cloud_cache_.width = 0;
    cloud_cache_.height = 1;
    cloud_cache_.is_dense = false;
    has_cloud_cache_ = false;
    RCLCPP_INFO(get_logger(), "Scan started — colored-cloud cache cleared");
    return;
  }

  // Scan stopped — save the cached colored cloud to disk.
  const auto now_tp = std::chrono::system_clock::now();
  const auto now_tt = std::chrono::system_clock::to_time_t(now_tp);
  std::tm tm_buf{};
  localtime_r(&now_tt, &tm_buf);
  char time_str[32];
  std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm_buf);

  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
  if (ec) {
    RCLCPP_ERROR(
      get_logger(), "Cannot create output dir '%s': %s",
      output_dir_.c_str(), ec.message().c_str());
    return;
  }

  const std::string filepath =
    (std::filesystem::path(output_dir_) /
     ("colored_" + std::string(time_str) + ".pcd")).string();

  std::size_t point_count = 0;
  std::string save_error;

  {
    std::lock_guard<std::mutex> lock(cloud_cache_mutex_);

    if (!has_cloud_cache_ || cloud_cache_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Scan stopped but colored-cloud cache is empty — "
        "is the camera connected and cloud_colorizer_node running?");
      return;
    }

    point_count = cloud_cache_.size();
    try {
      if (pcl::io::savePCDFileBinary(filepath, cloud_cache_) != 0) {
        throw std::runtime_error("savePCDFileBinary returned non-zero");
      }
    } catch (const std::exception & ex) {
      save_error = ex.what();
    }
  }

  if (!save_error.empty()) {
    RCLCPP_ERROR(
      get_logger(), "Failed to save colored PCD '%s': %s",
      filepath.c_str(), save_error.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "Scan stopped — %zu colored points saved → %s",
      point_count, filepath.c_str());
  }
}

}  // namespace lidar_camera_fusion

// ── main ───────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_camera_fusion::CloudColorizerNode>());
  rclcpp::shutdown();
  return 0;
}
