#ifndef LIDAR_CAMERA_FUSION__BODY_PREPROCESS_NODE_HPP_
#define LIDAR_CAMERA_FUSION__BODY_PREPROCESS_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

namespace lidar_camera_fusion
{

// body_preprocess_node
// ---------------------
// Subscribes:
//   /scanner/colored_cloud  (sensor_msgs/PointCloud2, XYZRGB, in base_link)
//   /scanner/is_scanning    (std_msgs/Bool, transient_local)
//
// Publishes:
//   /scanner/body_cloud     (sensor_msgs/PointCloud2, XYZRGB, in base_link)
//
// Pipeline (run ONCE when the scan stops, not per incoming frame):
//   1. Crop to a Z slab around the bed surface so RANSAC doesn't lock onto
//      the floor or ceiling reflections (z_min ≤ z ≤ z_max in base_link).
//   2. pcl::SACSegmentation<SACMODEL_PLANE> with distance threshold ~5 mm
//      to detect the bed plane. The plane is rejected if its normal is far
//      from world-vertical (we expect the bed to be roughly horizontal in
//      base_link); this prevents the algorithm from picking the side of a
//      tall body as a plane.
//   3. Remove inliers (the bed itself) -> remainder is body + noise.
//   4. Optional Euclidean cluster extraction; keep only the largest cluster
//      whose size exceeds min_cluster_size.
//   5. Publish the cleaned cloud and save body_<YYYYMMDD_HHMMSS>.pcd.
//
// Lifecycle:
//   /scanner/is_scanning=true  -> reset the raw cache, start caching frames
//   /scanner/is_scanning=false -> run the RANSAC pipeline ONCE on the final
//                                 cached colored cloud, publish /scanner/body_cloud
//                                 (RELIABLE + TRANSIENT_LOCAL so a late subscriber
//                                 still gets the one-shot result), and save the PCD.
//
// Rationale for processing only on stop: /scanner/colored_cloud is an
// accumulating snapshot, so the final frame already contains every point.
// Running RANSAC per-frame wasted CPU and produced log spam ("no cluster
// >= N pts") while the body was only partially scanned. One pass on the
// complete cloud is both cleaner and correct.
class BodyPreprocessNode : public rclcpp::Node
{
public:
  explicit BodyPreprocessNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void colored_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void is_scanning_callback(std_msgs::msg::Bool::ConstSharedPtr msg);

  // Run the full RANSAC bed-removal + clustering pipeline once on the given
  // raw colored cloud. Returns the cleaned body cloud (may be empty).
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr process_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in) const;

  // Parameters --------------------------------------------------------------
  std::string input_topic_;
  std::string output_topic_;
  std::string output_dir_;

  double z_min_;                  // crop slab lower bound (m, base_link)
  double z_max_;                  // crop slab upper bound (m, base_link)
  double plane_distance_thresh_;  // RANSAC inlier threshold (m, ~0.005)
  int    plane_max_iter_;         // RANSAC iterations
  double plane_eps_angle_deg_;    // max angle between plane normal and +Z axis
  bool   enable_cluster_;         // whether to keep only the largest cluster
  double cluster_tolerance_;      // Euclidean cluster radius (m)
  int    cluster_min_size_;       // smallest acceptable cluster
  int    cluster_max_size_;       // safety cap

  // ROS handles -------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           is_scanning_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    body_pub_;

  // Session cache: raw colored cloud (latest accumulating snapshot from
  // /scanner/colored_cloud). The RANSAC pipeline runs ONCE on this when the
  // scan stops.
  pcl::PointCloud<pcl::PointXYZRGB> raw_cache_;
  bool                              has_cache_{false};
  mutable std::mutex                cache_mutex_;
  std_msgs::msg::Header             last_header_;
};

}  // namespace lidar_camera_fusion

#endif  // LIDAR_CAMERA_FUSION__BODY_PREPROCESS_NODE_HPP_
