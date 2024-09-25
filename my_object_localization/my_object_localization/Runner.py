# main.py
import rclpy
from rclpy.executors import SingleThreadedExecutor
from my_object_localization.object_detector import ObjectDetector
from my_object_localization.object_tracker import ObjectTracker

def main():
    rclpy.init()

    detector = ObjectDetector()
    tracker = ObjectTracker()

    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(tracker)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    detector.destroy_node()
    tracker.destroy_node()
    rclpy.shutdown()