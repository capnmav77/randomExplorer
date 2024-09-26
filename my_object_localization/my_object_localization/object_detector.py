import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
from nav_msgs.msg import Odometry
from my_object_localization.object_database import ObjectDatabase
import numpy as np
from my_object_localization.yolo_detector import YoloDetect
import sensor_msgs_py.point_cloud2 as pc2


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
            obj_depth = self.get_depth(obj['position'])
            #print('Object depth:',obj_depth)
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
            return None

        # Convert PointCloud2 to a list of points
        point_list = list(pc2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True))
        #print('Point List:',point_list)

        # Calculate the bounding box coordinates
        x_min, y_min, x_max, y_max = bbox

        # Initialize variables to compute the average depth
        total_depth = 0
        depthx = 0
        depthy = 0
        count = 0

        # iterate over the points that fall in the range of the bounding box
        for i in range(int(x_min), int(x_max)):
            for j in range(int(y_min), int(y_max)):
                # Calculate the index of the point in the point cloud
                index = i * 320 + j
                # Get the depth of the point
                depth = point_list[index][2]
                dx = point_list[index][0]
                dy = point_list[index][1]
                # Check if the depth is a valid number
                if not np.isnan(depth):
                    total_depth += depth
                    depthx += dx
                    depthy += dy
                    count += 1

        # Calculate average depth
        if count > 0:
            average_depth = total_depth / count
            print('Average Depth:',average_depth)
            print('DepthX:',depthx/count)
            print('DepthY:',depthy/count)
            return average_depth  # Return depth in meters
        else:
            return None  # No points found in the bounding box