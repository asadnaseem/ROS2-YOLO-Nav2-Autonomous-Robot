#!/usr/bin/env python3
"""Detect COCO objects with YOLO and expose their RGB-D positions to Nav2."""

from pathlib import Path
import math
import struct
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from ultralytics import YOLO

try:
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
except (ImportError, RuntimeError):
    pass
cv2.setNumThreads(1)


class SemanticObstacleNode(Node):
    def __init__(self):
        super().__init__('semantic_obstacle_node')
        self.declare_parameter('confidence', 0.30)
        self.declare_parameter('maximum_depth', 5.0)
        self.declare_parameter('inference_rate', 2.0)
        self.declare_parameter(
            'obstacle_classes',
            ['person', 'chair', 'car', 'truck', 'bus', 'bicycle', 'stop sign']
        )
        model = Path(get_package_share_directory('yolo_nav2_semantic')) / 'models/yolov8n.pt'
        self.detector = YOLO(str(model))
        self.detector.predict(
            np.zeros((240, 320, 3), dtype=np.uint8), imgsz=320,
            device='cpu', verbose=False)
        self.bridge = CvBridge()
        self.depth = None
        self.camera_info = None
        self.depth_stamp = None
        self.last_inference_time = 0.0
        self.last_depth_cloud_time = 0.0
        self.cached_boxes = []
        self.cloud_pub = self.create_publisher(PointCloud2, '/yolo/obstacles', 10)
        self.depth_cloud_pub = self.create_publisher(PointCloud2, '/depth/obstacles', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/annotated_image', 2)
        self.status_pub = self.create_publisher(String, '/yolo/status', 10)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image_raw', self.rgb_cb, qos_profile_sensor_data)
        self.get_logger().info('YOLO semantic obstacle layer is ready')

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if self.depth.dtype == np.uint16:
            self.depth = self.depth.astype(np.float32) / 1000.0
        self.depth_stamp = msg.header.stamp
        now = time.monotonic()
        if self.camera_info is None or now - self.last_depth_cloud_time < 0.5:
            return
        self.last_depth_cloud_time = now
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        height, width = self.depth.shape[:2]
        stride = max(10, width // 24)
        points = []
        for v in range(0, height, stride):
            for u in range(0, width, stride):
                z = float(self.depth[v, u])
                if not math.isfinite(z) or z < 0.25 or z > 5.0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append((x, y, z))
        self.depth_cloud_pub.publish(self.make_cloud(msg.header, points))

    def info_cb(self, msg):
        self.camera_info = msg

    def rgb_cb(self, msg):
        now = time.monotonic()
        period = 1.0 / max(0.5, float(self.get_parameter('inference_rate').value))
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if now - self.last_inference_time < period:
            annotated = frame.copy()
            for x1, y1, x2, y2, label, confidence in self.cached_boxes:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (40, 220, 40), 2)
                cv2.putText(
                    annotated, f'{label} {confidence:.2f}', (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 220, 40), 1, cv2.LINE_AA)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.image_pub.publish(out)
            return
        self.last_inference_time = now
        result = self.detector.predict(
            frame, conf=float(self.get_parameter('confidence').value),
            imgsz=320, device='cpu', verbose=False
        )[0]
        self.cached_boxes = []
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            class_name = self.detector.names[int(box.cls.item())]
            confidence = float(box.conf.item())
            self.cached_boxes.append((x1, y1, x2, y2, class_name, confidence))
        points = []
        labels = []
        allowed = set(self.get_parameter('obstacle_classes').value)
        if self.depth is not None and self.camera_info is not None:
            fx, fy = self.camera_info.k[0], self.camera_info.k[4]
            cx, cy = self.camera_info.k[2], self.camera_info.k[5]
            height, width = self.depth.shape[:2]
            max_depth = float(self.get_parameter('maximum_depth').value)
            for box in result.boxes:
                name = self.detector.names[int(box.cls.item())]
                if name not in allowed:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                u = max(0, min(width - 1, (x1 + x2) // 2))
                v = max(0, min(height - 1, (y1 + y2) // 2))
                radius = 4
                patch = self.depth[max(0, v-radius):min(height, v+radius+1),
                                   max(0, u-radius):min(width, u+radius+1)]
                valid = patch[np.isfinite(patch) & (patch > 0.15) & (patch < max_depth)]
                if valid.size == 0:
                    continue
                z = float(np.median(valid))
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                # A small cluster gives the costmap a meaningful obstacle footprint.
                for lateral in np.linspace(-0.22, 0.22, 5):
                    for longitudinal in np.linspace(-0.22, 0.22, 5):
                        # Optical coordinates: x is right, y is down, z is forward.
                        points.append((x + lateral, y, z + longitudinal))
                labels.append(f'{name}:{z:.1f}m')
        self.cloud_pub.publish(self.make_cloud(msg.header, points))
        self.status_pub.publish(String(data=', '.join(labels) if labels else 'no semantic obstacle'))
        annotated = result.plot()
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    @staticmethod
    def make_cloud(source_header, points):
        cloud = PointCloud2()
        cloud.header = Header(stamp=source_header.stamp, frame_id='camera_optical_frame')
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = SemanticObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
