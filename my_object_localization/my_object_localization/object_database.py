import rclpy
from rclpy.node import Node

class ObjectDatabase(Node):
    def __init__(self):
        self.objects = []

    def add_object(self, description, curr_position , confidence):
        print(f"Adding object {description} to database")
        if self.is_object_present(description, curr_position, confidence):
            print(f"Object {description} already present in database updating position and confidence")
            return
        self.objects.append({
            'description': description, 
            'position': curr_position, 
            'confidence': confidence
        })

    def get_objects(self):
        print("Getting objects from database")
        return self.objects

    def is_object_present(self, description, curr_position, confidence):
        for obj in self.objects:
            if obj['description'] == description:
                
                x1 = obj['position'].position.x
                y1 = obj['position'].position.y
                z1 = obj['position'].position.z


                x2 = curr_position.position.x
                y2 = curr_position.position.y
                z2 = curr_position.position.z

                # check if the object is present in a 1 meter radius
                if ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2) < 1:
                    if(obj['confidence'] < confidence):
                        obj['confidence'] = confidence
                        obj['position'] = curr_position
                    return True
                
        return False