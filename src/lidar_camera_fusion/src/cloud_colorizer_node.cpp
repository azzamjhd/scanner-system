#include "lidar_camera_fusion/cloud_colorizer_node.hpp"

#include <cmath>
#include <functional>

#include <opencv2/core.hpp>

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
  declare_parameter("queue_size",       10);
  declare_parameter("approx_time_slop", 0.1);

  optical_frame_     = get_parameter("optical_frame").as_string();
  queue_size_        = get_parameter("queue_size").as_int();
  approx_time_slop_  = get_parameter("approx_time_slop").as_double();

  // ── TF2 ───────────────────────────────────────────────────────────────────
  // spin_thread=true: same reasoning as scan_assembler_node — the sync
  // callback can block in lookupTransform for up to 100 ms, so TF must
  // be received on a dedicated thread rather than the shared executor thread.
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  // ── message_filters subscriptions ─────────────────────────────────────────
  cloud_sub_.subscribe(this, "/scanner/assembled_cloud", rmw_qos_profile_sensor_data);
  image_sub_.subscribe(this, "/image_raw",               rmw_qos_profile_sensor_data);
  info_sub_ .subscribe(this, "/camera_info",             rmw_qos_profile_default);

  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(static_cast<uint32_t>(queue_size_)),
    cloud_sub_, image_sub_, info_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approx_time_slop_));
  sync_->registerCallback(
    std::bind(
      &CloudColorizerNode::sync_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // ── Publisher ─────────────────────────────────────────────────────────────
  colored_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/scanner/colored_cloud", rclcpp::SensorDataQoS());

  RCLCPP_INFO(
    get_logger(),
    "cloud_colorizer_node: projecting onto '%s', slop=%.2f s, queue=%d",
    optical_frame_.c_str(), approx_time_slop_, queue_size_);
}

// ── sync_callback ──────────────────────────────────────────────────────────

void CloudColorizerNode::sync_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr  cloud_msg,
  sensor_msgs::msg::Image::ConstSharedPtr        image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr   info_msg)
{
  // ── 1. TF lookup: cloud frame → optical frame at image timestamp ──────────
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      optical_frame_,
      cloud_msg->header.frame_id,
      rclcpp::Time(image_msg->header.stamp),
      rclcpp::Duration(0, 100'000'000));  // 100 ms timeout for TF latency
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

  // ── 4. Camera intrinsics from CameraInfo.k (row-major 3×3) ───────────────
  // k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  const double fx = info_msg->k[0];
  const double fy = info_msg->k[4];
  const double cx = info_msg->k[2];
  const double cy = info_msg->k[5];

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

  // ── 6. Serialize and publish ──────────────────────────────────────────────
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(colored_cloud, out_msg);
  out_msg.header.stamp    = image_msg->header.stamp;
  out_msg.header.frame_id = cloud_msg->header.frame_id;  // preserve original frame

  colored_pub_->publish(out_msg);
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
