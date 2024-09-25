import rclpy
from rclpy.node import Node
from collections import defaultdict
import json
import os

class ObjectDatabase(Node):
    def __init__(self):
        super().__init__('object_database')  # Initialize the Node
        # defaultdict of lists with the key as the object description
        self.Localized_Objects = defaultdict(list)
        self.json_file_path = '/home/neo/Robotics/Nav2Sensors/localized_objects.json'  # Path to the JSON file

    def add_object(self, description, curr_position, confidence):
        obj_location = {
            'position': curr_position,
            'confidence': confidence
        }
        
        print(f"Adding object '{description}' to database")

        # Check if the object is already present
        if self.is_object_present(description, obj_location):
            print(f"Object '{description}' already present in database, updating position and confidence")
            self.write_to_json()  # Update the JSON file
            return

        self.Localized_Objects[description].append(obj_location)
        self.write_to_json()  # Write to JSON after adding a new object

    def get_objects(self):
        print("Getting objects from database")
        return dict(self.Localized_Objects)  # Convert defaultdict to a regular dict for easier viewing

    def is_object_present(self, description, new_obj):
        for existing_obj in self.Localized_Objects[description]:
            existing_position = existing_obj['position']

            print(existing_position)
            
            x1, y1, z1 = existing_position
            x2, y2, z2 = new_obj['position']

            # Check if the object is present within a 1 meter radius
            if ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2) < 1:
                # Update confidence if the new object's confidence is higher
                if existing_obj['confidence'] < new_obj['confidence']:
                    existing_obj['confidence'] = new_obj['confidence']
                    existing_obj['position'] = new_obj['position']
                return True
                
        return False

    def batch_processing(self, YOLO_Objects):
        for obj in YOLO_Objects:
            self.add_object(obj['description'], obj['position'], obj['confidence'])
    
    def range_check(self, obj1, obj2):
        pass  # Placeholder for future implementation

    def write_to_json(self):
        """Write the current state of Localized_Objects to a JSON file."""
        with open(self.json_file_path, 'w') as json_file:
            json.dump(self.Localized_Objects, json_file, default=lambda o: o.__dict__, indent=4)
        print(f"Updated JSON file at '{self.json_file_path}'.")

