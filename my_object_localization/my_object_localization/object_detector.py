import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from my_object_localization.object_database import ObjectDatabase
import numpy as np
from my_object_localization.yolo_detector import YoloDetect

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscriber_rgb = self.create_subscription(
            Image,
            '/intel_realsense_r200_rgb/image_raw',
            self.rgb_callback,
            10)
        self.subscriber_depth = self.create_subscription(
            PointCloud2,
            '/intel_realsense_r200_depth/points',
            self.depth_callback,
            10)

        self.pose_subscriber = self.create_subscription(
            Odometry,  
            '/odom',
            self.pose_callback,
            10)

        self.yolo = YoloDetect()
        self.Current_position = None
        self.bridge = CvBridge()
        self.object_database = ObjectDatabase()
        self.depth_image = None
        self.point_cloud = None
        self.rgb_image = None
        self.image_msg = None

        self.timer = self.create_timer(2, self.timer_callback)

    def rgb_callback(self, msg):
        self.image_msg = msg

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        self.Current_position = (position.x, position.y, position.z)

    def depth_callback(self, msg):
        self.point_cloud = msg

    def timer_callback(self):
        self.process_image()

    def process_image(self):
        print("Processing image")
        if self.image_msg is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")

        if self.point_cloud is None or self.Current_position is None:
            return

        cv_image = cv2.resize(cv_image, (320, 240))
        if self.rgb_image is not None and self.check_similarity(self.rgb_image, cv_image):
            return

        print("Calling YOLO")
        self.objects = self.yolo.detect(cv_image)

        print("Objects detected: ", self.objects)
        for obj in self.objects:
            # obj_depth = self.get_depth(obj['position'])
            # print('Object depth:',obj_depth)
            # if obj_depth < 1:
            self.object_database.add_object(obj['description'], self.Current_position, obj['confidence'])


    def check_similarity(self, img1, img2):
        img1 = cv2.resize(img1, (224, 224))
        img2 = cv2.resize(img2, (224, 224))
        img1 = np.array(img1).flatten()
        img2 = np.array(img2).flatten()
        similarity = np.dot(img1, img2) / (np.linalg.norm(img1) * np.linalg.norm(img2))
        print("Similarity: ", similarity)
        return similarity > 0.8

  
    def get_depth(self, bbox):
        if self.point_cloud is None:
            print('Point cloud is None')
            return float('inf')

        x_min, y_min, x_max, y_max = map(int, bbox)

        # Access point cloud data
        pc_data = np.frombuffer(self.point_cloud.data, dtype=np.float32)
        width = self.point_cloud.width
        height = self.point_cloud.height
        point_step = self.point_cloud.point_step

        # Ensure the bounding box is within the image dimensions
        x_min = max(0, x_min)
        y_min = max(0, y_min)
        x_max = min(width - 1, x_max)
        y_max = min(height - 1, y_max)

        # Validate the bounding box
        if x_max < x_min or y_max < y_min:
            print('Invalid bounding box')
            return float('inf')

        # Extract Z coordinates efficiently
        valid_z_values = []
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                idx = (y * width + x) * point_step  # Accessing the first element of the point
                if idx + 8 < pc_data.size * pc_data.itemsize:  # Ensure we're within bounds
                    z_value = pc_data[idx + 2]  # Z is usually the 3rd value (index 2)
                    if z_value > 0:  # Check if the depth is valid
                        valid_z_values.append(z_value)

        if not valid_z_values:
            print('No valid Z values found')
            return float('inf')  # Return infinity if no valid depth found

        return np.mean(valid_z_values)  # Return the average depth of the detected object
