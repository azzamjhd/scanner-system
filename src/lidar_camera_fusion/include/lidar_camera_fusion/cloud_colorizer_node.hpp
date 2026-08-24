#ifndef LIDAR_CAMERA_FUSION__CLOUD_COLORIZER_NODE_HPP_
#define LIDAR_CAMERA_FUSION__CLOUD_COLORIZER_NODE_HPP_

#include <ctime>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace lidar_camera_fusion
{

class CloudColorizerNode : public rclcpp::Node
{
public:
  explicit CloudColorizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void sync_callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr  cloud_msg,
    sensor_msgs::msg::Image::ConstSharedPtr        image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr   info_msg);
  void is_scanning_callback(std_msgs::msg::Bool::ConstSharedPtr msg);

  // params
  std::string optical_frame_;
  std::string output_dir_;
  std::string image_topic_;
  std::string camera_info_topic_;
  int         queue_size_;
  double      approx_time_slop_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // message_filters
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image>       image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo>  info_sub_;
  std::shared_ptr<Synchronizer>                               sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  colored_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          is_scanning_sub_;

  // Colored-cloud cache — accumulated across scan, saved on stop signal
  pcl::PointCloud<pcl::PointXYZRGB> cloud_cache_;
  bool                               has_cloud_cache_{false};
  mutable std::mutex                 cloud_cache_mutex_;
};

}  // namespace lidar_camera_fusion

#endif  // LIDAR_CAMERA_FUSION__CLOUD_COLORIZER_NODE_HPP_
