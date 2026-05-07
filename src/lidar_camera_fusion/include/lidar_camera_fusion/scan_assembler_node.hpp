#ifndef LIDAR_CAMERA_FUSION__SCAN_ASSEMBLER_NODE_HPP_
#define LIDAR_CAMERA_FUSION__SCAN_ASSEMBLER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace lidar_camera_fusion
{

class ScanAssemblerNode : public rclcpp::Node
{
public:
  explicit ScanAssemblerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void publish_callback();
  void clear_callback(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void append_cloud(const sensor_msgs::msg::PointCloud2 & cloud_in);

  // params
  std::string target_frame_;
  std::string scan_topic_;
  int         max_points_;
  double      publish_rate_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  laser_geometry::LaserProjection laser_projection_;

  // Circular buffer — pre-allocated to max_points_, no runtime reallocation
  std::vector<geometry_msgs::msg::Point32> point_buffer_;
  std::size_t write_index_{0};
  std::size_t active_count_{0};
  mutable std::mutex buffer_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr            clear_srv_;
  rclcpp::TimerBase::SharedPtr                                   publish_timer_;
};

}  // namespace lidar_camera_fusion

#endif  // LIDAR_CAMERA_FUSION__SCAN_ASSEMBLER_NODE_HPP_
