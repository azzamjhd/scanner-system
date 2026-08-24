#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class VoxelFilterNode(Node):
    def __init__(self):
        super().__init__('voxel_filter_node')
        
        # Declare parameters
        self.declare_parameter('voxel_size', 0.015)  # 15mm voxel grid size
        self.declare_parameter('input_topic', '/scanner/body_cloud')
        self.declare_parameter('output_topic', '/point_cloud/downsampled')
        
        self.voxel_size = self.get_parameter('voxel_size').value
        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        
        # Subscription and Publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self.cloud_callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            self._output_topic,
            10
        )
        self.get_logger().info(f"Voxel Filter Node initialized. Input: {self._input_topic}, Output: {self._output_topic}, Voxel: {self.voxel_size} m")

    def cloud_callback(self, msg: PointCloud2):
        if not msg.data:
            return
            
        # Parse points safely using helper
        points_list = list(pc2.read_points(msg, skip_nans=True))
        if not points_list:
            return
            
        # Extract voxel grid key: round x, y, z to voxel_size
        voxel_coords = {}
        for p in points_list:
            # First three values are always x, y, z
            x, y, z = p[0], p[1], p[2]
            key = (int(x / self.voxel_size), int(y / self.voxel_size), int(z / self.voxel_size))
            if key not in voxel_coords:
                voxel_coords[key] = p
                
        downsampled_points = list(voxel_coords.values())
        
        # Recreate PointCloud2 message preserving header and fields
        filtered_msg = pc2.create_cloud(msg.header, msg.fields, downsampled_points)
        self.publisher.publish(filtered_msg)
        self.get_logger().debug(f"Downsampled points from {len(points_list)} to {len(downsampled_points)}")

def main(args=None):
    rclpy.init(args=args)
    node = VoxelFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
