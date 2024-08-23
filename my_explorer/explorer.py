import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from queue import Queue

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map = None
        self.exploring = False
        self.get_logger().info('Explorer node initialized')
        self.top_interests = Queue()

    def map_callback(self, msg):
        self.get_logger().info('Received map update')
        self.map = msg
        if not self.exploring:
            self.exploring = True
            self.explore()

    def explore(self):
        self.get_logger().info('Starting exploration')
        if self.map is None:
            self.get_logger().warn('No map available')
            return

        # if(self.top_interests.empty()):
        #     self.find_frontier()
        #select the first frontier cell from the queue and pop it 
        frontier = self.find_frontier()

        if frontier is not None:
            goal = PoseStamped()
            goal.header = self.map.header
            goal.pose.position.x = frontier[0] * self.map.info.resolution + self.map.info.origin.position.x
            goal.pose.position.y = frontier[1] * self.map.info.resolution + self.map.info.origin.position.y
            goal.pose.orientation.w = 1.0

            self.get_logger().info(f'Moving to frontier: ({goal.pose.position.x}, {goal.pose.position.y})')
            self.send_goal(goal)
        else:
            self.get_logger().info('No frontiers found. Exploration complete.')
            self.exploring = False

    def find_frontier(self):
        self.get_logger().info('Finding frontier')
        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        frontier_cells = []

        for y in range(1, self.map.info.height - 1):
            for x in range(1, self.map.info.width - 1):
                if map_data[y, x] == 0:  # Free cell
                    if -1 in map_data[y-1:y+2, x-1:x+2]:  # Adjacent to unknown
                        frontier_cells.append((x, y))

        if frontier_cells:
            self.get_logger().info(f'Found {len(frontier_cells)} frontier cells')
            # finding the closest frontier cell and pushing the top 5 frontier cells to the queue
            #self.get_fartherest_frontiers_count5(frontier_cells)
            return frontier_cells[np.random.randint(len(frontier_cells))]
        self.get_logger().warn('No frontier cells found')
        return None
    
    
    def get_fartherest_frontiers_count5(self, frontier_cells):
        if(len(frontier_cells) < 5):
            for i in range(len(frontier_cells)):
                self.top_interests.put(frontier_cells[i])
        else:
            # finding the farthest frontier cell and pushing the top 5 frontier cells to the queue
            for i in range(5):
                max_distance = 0
                max_distance_index = 0
                for j in range(len(frontier_cells)):
                    distance = np.sqrt((frontier_cells[j][0] - self.map.info.width/2)**2 + (frontier_cells[j][1] - self.map.info.height/2)**2)
                    if distance > max_distance:
                        max_distance = distance
                        max_distance_index = j
                self.top_interests.put(frontier_cells[max_distance_index])
                frontier_cells.pop(max_distance_index)


    def send_goal(self, goal):
        self.get_logger().info('Sending goal to navigation stack')
        self.nav_client.wait_for_server()
        self.future = self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal))
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached, finding next frontier')
        self.explore()

def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()