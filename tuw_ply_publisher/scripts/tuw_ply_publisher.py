#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class ply_publisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.declare_parameter("use_geolocation", False)
        self.declare_parameter("map_origin_x", 0.0)
        self.declare_parameter("map_origin_y", 0.0)
        self.declare_parameter("plc_origin_x", 0.0)
        self.declare_parameter("plc_origin_y", 0.0)
        self.declare_parameter("plc_rotation", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("ply_path", "./ws00/src/tuw_geo/tuw_ply_publisher/config/facade.ply")
        geo_location = self.get_parameter("use_geolocation").get_parameter_value().bool_value
        self.map_origin_x = self.get_parameter("map_origin_x").get_parameter_value().double_value
        self.map_origin_y = self.get_parameter("map_origin_y").get_parameter_value().double_value
        self.plc_origin_x = self.get_parameter("plc_origin_x").get_parameter_value().double_value
        self.plc_origin_y = self.get_parameter("plc_origin_y").get_parameter_value().double_value
        self.plc_rotation = self.get_parameter("plc_rotation").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.ply_path = self.get_parameter("ply_path").get_parameter_value().string_value
        
        self.file_path = self.ply_path
        self.pcd = o3d.io.read_point_cloud(self.file_path)
        self.points = np.asarray(self.pcd.points)
        
        if self.pcd.has_colors():
            self.colors = np.asarray(self.pcd.colors)
        else:
            self.colors = None
        
        if geo_location:
            self.use_geolocation()
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def use_geolocation(self):
        self.get_logger().info("using geo location")
        map_offset = np.array([self.map_origin_x, self.map_origin_y, 0])
        facade_offset = np.array([self.plc_origin_x, self.plc_origin_y, 0])

        center = np.mean(self.points, axis=0)

        points_centered = self.points - center

        angle_deg = self.plc_rotation
        angle_rad = np.deg2rad(angle_deg)
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad), 0],
            [np.sin(angle_rad),  np.cos(angle_rad), 0],
            [0,                  0,                 1]
        ])
        rotated_points = points_centered @ rotation_matrix.T

        relative_offset = facade_offset - map_offset
        # some correction offset to better align it
        relative_offset = np.array([-relative_offset[0]+2, relative_offset[1]-7, relative_offset[2]])

        self.points = rotated_points + center - relative_offset
        
    def timer_callback(self):
        self.get_logger().info("publish pointcloud!")
        msg = self.convert_to_pointcloud()
        self.publisher_.publish(msg)
        
    def convert_to_pointcloud(self) -> PointCloud2:
        msg = PointCloud2()
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        msg.header = header
        msg.height = 1
        msg.width = len(self.points)
        
        msg.is_dense = True
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
                
        data = bytearray()
        for i in range(len(self.points)):
            x, y, z = self.points[i]
            r, g, b = (self.colors[i] * 255).astype(np.uint8)
            rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            data += struct.pack('ffff', x, y, z, rgb)
           
        point_step = 16
        msg.data = bytes(data)
        msg.point_step = point_step
        msg.row_step = point_step * len(self.points)
        msg.fields = fields
        
        return msg
        
def main(args=None):
    rclpy.init(args=args)
    node = ply_publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()