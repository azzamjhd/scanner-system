#ifndef LIDAR_CAMERA_FUSION__CLOUD_COLORIZER_NODE_HPP_
#define LIDAR_CAMERA_FUSION__CLOUD_COLORIZER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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

  // params
  std::string optical_frame_;
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

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;
};

}  // namespace lidar_camera_fusion

#endif  // LIDAR_CAMERA_FUSION__CLOUD_COLORIZER_NODE_HPP_
