#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, Header
from std_srvs.srv import Trigger
import numpy as np
import math
import os
from datetime import datetime
import struct


class Scanner3DNode(Node):
    def __init__(self):
        super().__init__('scanner_3d_node')
        
        # Declare and get parameters
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 90.0)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('ticks_per_unit', 100.0)
        self.declare_parameter('axis', 1)  # 0=Rotational Y, 1=Linear Z (gantry default)
        self.declare_parameter('simulate_encoder', False)
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('position_topic', '/current_position')
        
        self.angle_min_filter = math.radians(self.get_parameter('angle_min').value)
        self.angle_max_filter = math.radians(self.get_parameter('angle_max').value)
        self.range_max = self.get_parameter('range_max').value
        self.ticks_per_unit = self.get_parameter('ticks_per_unit').value
        self.axis = self.get_parameter('axis').value
        self.simulate_encoder = self.get_parameter('simulate_encoder').value
        self.output_dir = self.get_parameter('output_dir').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.position_topic = self.get_parameter('position_topic').value
        
        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # State variables
        self.is_scanning = False
        self.point_cloud = []
        self.current_position_value = 0.0
        self.simulated_position_value = 0.0
        self._warned_position_unit_mismatch = False
        
        # Timer for publishing accumulated point cloud
        self.viz_timer = self.create_timer(0.5, self.publish_accumulated_pointcloud)  # 2 Hz
        
        # Services
        self.start_srv = self.create_service(
            Trigger, 
            'start_scan', 
            self.start_scan_callback
        )
        self.stop_srv = self.create_service(
            Trigger, 
            'stop_scan', 
            self.stop_scan_callback
        )
        self.clear_viz_srv = self.create_service(
            Trigger,
            'clear_visualization',
            self.clear_visualization_callback
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )
        
        if not self.simulate_encoder:
            self.position_sub = self.create_subscription(
                Float32,
                self.position_topic,
                self.position_callback,
                10
            )
        
        # Publisher for real-time point cloud visualization
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/scanner/pointcloud', 10)
        
        self.get_logger().info('Scanner 3D Node initialized')
        self.get_logger().info(f'Simulation mode: {self.simulate_encoder}')
        self.get_logger().info(f'Axis: {"Rotational Y" if self.axis == 0 else "Linear Z"}')
        self.get_logger().info(f'Angle filter: {math.degrees(self.angle_min_filter):.1f}° to {math.degrees(self.angle_max_filter):.1f}°')
        self.get_logger().info(f'Range max: {self.range_max}m')
    
    def parameter_callback(self, params):
        """Callback for parameter changes"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'angle_min':
                self.angle_min_filter = math.radians(param.value)
                self.get_logger().info(f'Updated angle_min to {param.value}°')
            elif param.name == 'angle_max':
                self.angle_max_filter = math.radians(param.value)
                self.get_logger().info(f'Updated angle_max to {param.value}°')
            elif param.name == 'range_max':
                self.range_max = param.value
                self.get_logger().info(f'Updated range_max to {param.value}m')
            elif param.name == 'simulate_encoder':
                self.simulate_encoder = param.value
                self.get_logger().info(f'Updated simulate_encoder to {param.value}')
            elif param.name == 'axis':
                if param.value not in (0, 1):
                    return SetParametersResult(
                        successful=False,
                        reason='axis must be 0 (rotational) or 1 (linear)'
                    )
                self.axis = param.value
                self._warned_position_unit_mismatch = False
                self.get_logger().info(
                    f'Updated axis to {param.value} '
                    f'({"Rotational Y" if param.value == 0 else "Linear Z"})'
                )
        
        return SetParametersResult(successful=True)
        
    def start_scan_callback(self, request, response):
        """Service callback to start scanning"""
        if self.is_scanning:
            response.success = False
            response.message = 'Already scanning'
            self.get_logger().warn('Start scan requested but already scanning')
        else:
            self.is_scanning = True
            self.point_cloud = []
            self.simulated_position_value = 0.0
            response.success = True
            response.message = 'Scan started'
            self.get_logger().info('Scan started')
        return response
    
    def stop_scan_callback(self, request, response):
        """Service callback to stop scanning and save data"""
        if not self.is_scanning:
            response.success = False
            response.message = 'Not currently scanning'
            self.get_logger().warn('Stop scan requested but not scanning')
        else:
            self.is_scanning = False
            num_points = len(self.point_cloud)
            
            if num_points > 0:
                filename = self.save_point_cloud()
                response.success = True
                response.message = f'Scan stopped. Saved {num_points} points to {filename}'
                self.get_logger().info(f'Scan stopped. Saved {num_points} points to {filename}')
            else:
                response.success = False
                response.message = 'Scan stopped but no points collected'
                self.get_logger().warn('Scan stopped but no points collected')
        return response
    
    def clear_visualization_callback(self, request, response):
        """Service callback to clear visualization (keeps saved data)"""
        # Publish empty point cloud to clear RViz
        self.publish_pointcloud([])
        response.success = True
        response.message = 'Visualization cleared'
        self.get_logger().info('Visualization cleared (data preserved)')
        return response
    
    def position_callback(self, msg):
        """Callback for real position data from firmware (mm by default)."""
        self.current_position_value = msg.data
    
    def scan_callback(self, msg):
        """Callback for laser scan data"""
        if not self.is_scanning:
            return
        
        # Get position value (real or simulated)
        if self.simulate_encoder:
            position = self.simulated_position_value
            self.simulated_position_value += 1.0 / self.ticks_per_unit
        else:
            position = self.current_position_value

        # Helpful guard for common misconfiguration:
        # gantry encoders publish mm, but rotational mode expects degrees.
        if (
            self.axis == 0
            and not self.simulate_encoder
            and not self._warned_position_unit_mismatch
            and abs(position) > 360.0
        ):
            self.get_logger().warn(
                'axis=0 expects encoder angle in degrees, but /current_position is '
                f'{position:.2f} (likely mm). Set axis:=1 for linear gantry scans.'
            )
            self._warned_position_unit_mismatch = True
        
        # Temporary list for this scan's points
        scan_points = []
        
        # Process laser scan
        angle = msg.angle_min
        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if math.isnan(distance) or math.isinf(distance):
                angle += msg.angle_increment
                continue
            
            # Skip if out of range limits
            if distance < msg.range_min or distance > msg.range_max or distance > self.range_max:
                angle += msg.angle_increment
                continue
            
            # Apply angle filters - convert current angle to degrees for comparison
            angle_deg = math.degrees(angle)
            
            # Normalize angle to 0-360 range
            while angle_deg < 0:
                angle_deg += 360
            while angle_deg >= 360:
                angle_deg -= 360
            
            # Get filter bounds in degrees (already in degrees from parameters)
            min_deg = self.get_parameter('angle_min').value
            max_deg = self.get_parameter('angle_max').value
            
            # Check if angle is within filter range
            in_range = False
            if min_deg <= max_deg:
                # Normal case: e.g., 0° to 180°
                in_range = (angle_deg >= min_deg and angle_deg <= max_deg)
            else:
                # Wrap-around case: e.g., 270° to 90° (crosses 0°)
                in_range = (angle_deg >= min_deg or angle_deg <= max_deg)
            
            if not in_range:
                angle += msg.angle_increment
                continue
            
            # Convert polar to Cartesian (LiDAR frame)
            x_lidar = distance * math.cos(angle)
            y_lidar = distance * math.sin(angle)
            
            # Transform to 3D based on axis type
            if self.axis == 0:  # Rotational around Y-axis
                # Position is rotation angle in degrees
                theta = math.radians(position)
                x = x_lidar * math.cos(theta)
                y = y_lidar
                z = x_lidar * math.sin(theta)
            else:  # Linear along Z-axis (axis == 1)
                # Position is linear displacement in mm
                x = x_lidar
                y = y_lidar
                z = position / 1000.0  # Convert mm to meters
            
            # Add point to cloud
            self.point_cloud.append([x, y, z])
            scan_points.append([x, y, z])
            
            angle += msg.angle_increment
        
        if len(self.point_cloud) % 1000 == 0 and len(self.point_cloud) > 0:
            self.get_logger().info(
                f'Collected {len(self.point_cloud)} points (position: {position:.2f})')
    
    def publish_accumulated_pointcloud(self):
        """Publish full accumulated point cloud for visualization"""
        if not self.is_scanning or len(self.point_cloud) == 0:
            return
            
        # Publish full accumulated cloud (with downsampling if too large)
        points_to_publish = self.point_cloud
        
        # Downsample if too many points for real-time display
        max_viz_points = 50000  # Reduced from 100k for better performance with 700k scans
        if len(points_to_publish) > max_viz_points:
            # Take every Nth point
            step = len(points_to_publish) // max_viz_points
            points_to_publish = self.point_cloud[::step]
        
        self.publish_pointcloud(points_to_publish)
    
    def publish_pointcloud(self, points):
        """Publish point cloud for real-time visualization"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'scanner_frame'
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Convert points to bytes
        points_array = np.array(points, dtype=np.float32)
        cloud_data = points_array.tobytes()
        
        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3 floats * 4 bytes
            row_step=12 * len(points),
            data=cloud_data
        )
        
        self.pointcloud_pub.publish(cloud_msg)
    
    def save_point_cloud(self):
        """Save point cloud to PLY file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, f'scan_{timestamp}.ply')
        
        points = np.array(self.point_cloud)
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Write PLY file
        with open(filename, 'w') as f:
            # Header
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('end_header\n')
            
            # Data
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')
        
        self.get_logger().info(f'Point cloud saved to {filename}')
        return filename
    
    def save_point_cloud_pcd(self):
        """Alternative: Save point cloud to PCD file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, f'scan_{timestamp}.pcd')
        
        points = np.array(self.point_cloud)
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        with open(filename, 'w') as f:
            # Header
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            
            # Data
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')
        
        self.get_logger().info(f'Point cloud saved to {filename}')
        return filename


def main(args=None):
    rclpy.init(args=args)
    node = Scanner3DNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
