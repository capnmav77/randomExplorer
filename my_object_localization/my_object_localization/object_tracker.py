import rclpy
from rclpy.node import Node
from my_object_localization.object_database import ObjectDatabase

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.object_database = ObjectDatabase()
        self.get_logger().info('Object Tracker has been started')

    def run(self):
        while True:
            for obj in self.object_database.get_objects():
                self.get_logger().info(f'Object: {obj["description"]} at position: {obj["[position]"]} , with confidence: {obj["confidence"]}')
            # wait for 5 seconds
            rclpy.spin_once(self, timeout_sec=5)



                    
