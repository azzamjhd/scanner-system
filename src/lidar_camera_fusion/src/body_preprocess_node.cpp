#include "lidar_camera_fusion/body_preprocess_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_camera_fusion
{

BodyPreprocessNode::BodyPreprocessNode(const rclcpp::NodeOptions & options)
: Node("body_preprocess_node", options)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  declare_parameter("input_topic",            "/scanner/colored_cloud");
  declare_parameter("output_topic",           "/scanner/body_cloud");
  declare_parameter("output_dir",             ".");

  declare_parameter("z_min",                  -0.50);   // m, base_link
  declare_parameter("z_max",                   0.80);

  declare_parameter("plane_distance_thresh",  0.005);   // 5 mm
  declare_parameter("plane_max_iter",         200);
  declare_parameter("plane_eps_angle_deg",    15.0);    // expect bed ≈ horizontal

  declare_parameter("enable_cluster",         true);
  declare_parameter("cluster_tolerance",      0.02);    // 2 cm
  declare_parameter("cluster_min_size",       500);
  declare_parameter("cluster_max_size",       2'000'000);

  input_topic_           = get_parameter("input_topic").as_string();
  output_topic_          = get_parameter("output_topic").as_string();
  output_dir_            = get_parameter("output_dir").as_string();
  z_min_                 = get_parameter("z_min").as_double();
  z_max_                 = get_parameter("z_max").as_double();
  plane_distance_thresh_ = get_parameter("plane_distance_thresh").as_double();
  plane_max_iter_        = get_parameter("plane_max_iter").as_int();
  plane_eps_angle_deg_   = get_parameter("plane_eps_angle_deg").as_double();
  enable_cluster_        = get_parameter("enable_cluster").as_bool();
  cluster_tolerance_     = get_parameter("cluster_tolerance").as_double();
  cluster_min_size_      = get_parameter("cluster_min_size").as_int();
  cluster_max_size_      = get_parameter("cluster_max_size").as_int();

  // ── Subscriptions ────────────────────────────────────────────────────────
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&BodyPreprocessNode::colored_callback, this, std::placeholders::_1));

  is_scanning_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/scanner/is_scanning",
    rclcpp::QoS(1).transient_local(),
    std::bind(&BodyPreprocessNode::is_scanning_callback, this,
              std::placeholders::_1));

  // ── Publisher ────────────────────────────────────────────────────────────
  // RELIABLE + TRANSIENT_LOCAL: the body cloud is published exactly once per
  // scan (on stop). Transient-local durability means manual_segmentation_node
  // still receives the latched result even if it (re)connects after we publish.
  body_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::QoS(1).reliable().transient_local());

  RCLCPP_INFO(
    get_logger(),
    "body_preprocess_node: %s -> %s | z=[%.2f,%.2f] m | plane d=%.3f m, "
    "eps=%.1f deg | cluster=%s tol=%.3f m, min=%d",
    input_topic_.c_str(), output_topic_.c_str(),
    z_min_, z_max_,
    plane_distance_thresh_, plane_eps_angle_deg_,
    enable_cluster_ ? "on" : "off",
    cluster_tolerance_, cluster_min_size_);
}

// ── colored_callback ───────────────────────────────────────────────────────
// While scanning, just cache the latest accumulating snapshot. No processing
// here — RANSAC runs once on stop (see is_scanning_callback). /scanner/colored_cloud
// already contains every previous point, so the latest frame is the complete cloud.

void BodyPreprocessNode::colored_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(*msg, *cloud_in);
  if (cloud_in->empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(cache_mutex_);
  raw_cache_   = *cloud_in;
  last_header_ = msg->header;
  has_cache_   = true;
}

// ── process_cloud ──────────────────────────────────────────────────────────
// Full RANSAC bed-removal + optional largest-cluster extraction. Runs once.

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BodyPreprocessNode::process_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in) const
{
  auto empty = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (cloud_in->empty()) {
    return empty;
  }

  // 1. Crop to Z slab in base_link to focus RANSAC on the bed surface
  auto cloud_cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  {
    pcl::PassThrough<pcl::PointXYZRGB> pt;
    pt.setInputCloud(cloud_in);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(static_cast<float>(z_min_), static_cast<float>(z_max_));
    pt.filter(*cloud_cropped);
  }
  if (cloud_cropped->empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Z-crop [%.2f,%.2f] removed all %zu pts — check z_min/z_max",
      z_min_, z_max_, cloud_in->size());
    return empty;
  }

  // 2. RANSAC plane fit, normal constrained to ~+Z so we don't fit the body side.
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr      inliers(new pcl::PointIndices);
  {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_max_iter_);
    seg.setDistanceThreshold(plane_distance_thresh_);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(plane_eps_angle_deg_ * M_PI / 180.0);
    seg.setInputCloud(cloud_cropped);
    seg.segment(*inliers, *coeff);
  }

  auto cloud_no_bed = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (inliers->indices.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "RANSAC found no plane — forwarding cropped cloud unchanged (%zu pts)",
      cloud_cropped->size());
    *cloud_no_bed = *cloud_cropped;
  } else {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);   // keep everything that is NOT the bed
    extract.filter(*cloud_no_bed);
    RCLCPP_INFO(
      get_logger(),
      "RANSAC removed %zu bed inliers of %zu cropped pts -> %zu remaining",
      inliers->indices.size(), cloud_cropped->size(), cloud_no_bed->size());
  }

  if (cloud_no_bed->empty()) {
    return empty;
  }

  // 3. Optional largest-cluster extraction.
  auto cloud_body = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (enable_cluster_) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_no_bed);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_no_bed);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Euclidean clustering found no cluster >= %d pts — using non-bed cloud "
        "(%zu pts). Lower cluster_min_size or set enable_cluster:=false.",
        cluster_min_size_, cloud_no_bed->size());
      *cloud_body = *cloud_no_bed;
    } else {
      auto largest = std::max_element(
        cluster_indices.begin(), cluster_indices.end(),
        [](const pcl::PointIndices & a, const pcl::PointIndices & b) {
          return a.indices.size() < b.indices.size();
        });
      cloud_body->reserve(largest->indices.size());
      for (int idx : largest->indices) {
        cloud_body->push_back((*cloud_no_bed)[idx]);
      }
      RCLCPP_INFO(
        get_logger(),
        "Kept largest cluster: %zu pts (of %zu clusters)",
        cloud_body->size(), cluster_indices.size());
    }
  } else {
    *cloud_body = *cloud_no_bed;
  }

  cloud_body->width    = static_cast<std::uint32_t>(cloud_body->size());
  cloud_body->height   = 1;
  cloud_body->is_dense = false;
  return cloud_body;
}

// ── is_scanning_callback ───────────────────────────────────────────────────

void BodyPreprocessNode::is_scanning_callback(
  std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (msg->data) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raw_cache_.clear();
    has_cache_ = false;
    RCLCPP_INFO(get_logger(), "Scan started — raw cache cleared");
    return;
  }

  // ── Scan stopped: run the RANSAC pipeline ONCE on the final cached cloud ──
  auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  std_msgs::msg::Header header;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (!has_cache_ || raw_cache_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Scan stopped but raw cache is empty — nothing to process. "
        "(Is /scanner/colored_cloud publishing during the scan?)");
      return;
    }
    *raw   = raw_cache_;
    header = last_header_;
  }

  RCLCPP_INFO(
    get_logger(), "Scan stopped — processing %zu raw colored points",
    raw->size());

  auto cloud_body = process_cloud(raw);
  if (cloud_body->empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Body cloud is empty after processing — nothing published or saved.");
    return;
  }

  // Publish once (latched: RELIABLE + TRANSIENT_LOCAL).
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*cloud_body, out_msg);
  out_msg.header = header;
  body_pub_->publish(out_msg);
  RCLCPP_INFO(
    get_logger(), "Published %zu body points -> %s (latched)",
    cloud_body->size(), output_topic_.c_str());

  // Save body_<timestamp>.pcd
  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
  if (ec) {
    RCLCPP_ERROR(
      get_logger(), "Cannot create output dir '%s': %s",
      output_dir_.c_str(), ec.message().c_str());
    return;
  }

  const auto now_tt = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now());
  std::tm tm_buf{};
  localtime_r(&now_tt, &tm_buf);
  char time_str[32];
  std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm_buf);

  const std::string filepath =
    (std::filesystem::path(output_dir_) /
     ("body_" + std::string(time_str) + ".pcd")).string();

  try {
    if (pcl::io::savePCDFileBinary(filepath, *cloud_body) != 0) {
      throw std::runtime_error("savePCDFileBinary returned non-zero");
    }
    RCLCPP_INFO(
      get_logger(), "Scan stopped — %zu body points saved -> %s",
      cloud_body->size(), filepath.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Failed to save body PCD '%s': %s",
      filepath.c_str(), ex.what());
  }
}

}  // namespace lidar_camera_fusion

// ── main ───────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_camera_fusion::BodyPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
