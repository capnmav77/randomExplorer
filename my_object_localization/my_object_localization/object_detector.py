import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry  # Import the Odometry message type
from my_object_localization.object_database import ObjectDatabase
import numpy as np
from my_object_localization.yolo_detector import YoloDetect
import struct
from sensor_msgs.msg import PointField

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
        self.Current_position = (position.x, position.y, position.z) # Store the current position

    def depth_callback(self, msg):
        self.point_cloud = msg

    def timer_callback(self):
        self.process_image()

    def process_image(self):
        print("Processing image")
        cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")

        if self.point_cloud is None or self.Current_position is None:
            return

        # Resize and check for similarity
        cv_image = cv2.resize(cv_image, (320, 240))
        if self.rgb_image is not None and self.check_similarity(self.rgb_image, cv_image):
            return

        print("Calling YOLO")
        self.objects = self.yolo.detect(cv_image)

        print("Objects detected: ", self.objects)
        for obj in self.objects:
            obj_depth = self.get_depth(obj['position'])
            if obj_depth < 1:
                self.object_database.add_object(obj['description'], self.Current_position, obj['confidence'])

    def check_similarity(self, img1, img2):
        # Cosine similarity
        img1 = cv2.resize(img1, (224, 224))
        img2 = cv2.resize(img2, (224, 224))
        img1 = np.array(img1).flatten()
        img2 = np.array(img2).flatten()
        similarity = np.dot(img1, img2) / (np.linalg.norm(img1) * np.linalg.norm(img2))
        print("Similarity: ", similarity)
        return similarity > 0.8

    def get_depth(self, bbox):
        if self.point_cloud is None:
            return float('inf')

        x_min, y_min, x_max, y_max = bbox
        depth_values = []

        pc_data = np.frombuffer(self.point_cloud.data, dtype=np.float32)
        width = self.point_cloud.width
        height = self.point_cloud.height
        point_step = self.point_cloud.point_step

        # Pre-compute indices for faster lookup
        start_idx = y_min * width + x_min
        end_idx = y_max * width + x_max - 1

        # Extract Z coordinates efficiently
        z_indices = range(start_idx * point_step + 8, (end_idx + 1) * point_step + 8, 4)
        
        valid_z_values = []
        for idx in z_indices:
            if pc_data[idx] > 0:
                valid_z_values.append(pc_data[idx])

        if not valid_z_values:
            return float('inf')  # Return infinity if no valid depth found

        return np.mean(valid_z_values)  # Return the average depth of the detected object