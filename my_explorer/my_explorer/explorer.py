import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math
import heapq
import scipy.interpolate as si
from threading import Lock

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        # 2 subs , 1 pub and 1 action client
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Initialize variables and locks
        self.map = None
        self.odom = None
        self.exploring = False
        self.map_lock = Lock()
        self.odom_lock = Lock()
        
        # Parameters
        self.expansion_size = 3  # Don't you fucking dare change this
        self.target_error = 0.5  # same here !
        self.safety_distance = 1
        
        self.get_logger().info('Explorer node initialized')

    def map_callback(self, msg):
        with self.map_lock:
            self.map = msg
        if not self.exploring:
            self.exploring = True
            self.explore()

    def odom_callback(self, msg):
        with self.odom_lock:
            self.odom = msg

    def view_map(self, map):
        with open('/home/neo/Robotics/Nav2Sensors/src/my_explorer/my_explorer/map.txt', 'w') as file:
            for i in range(len(map)):
                for j in range(len(map[0])):
                    file.write(str(map[i, j]) + ' ')
                file.write('\n')
    

    def is_point_safe(self, map_data, point):
        x, y = point
        for dx in range(-self.safety_distance, self.safety_distance + 1):
            for dy in range(-self.safety_distance, self.safety_distance + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < map_data.shape[1] and 0 <= ny < map_data.shape[0]:
                    if map_data[ny, nx] == 100:  # Occupied cell
                        return False
        return True

    def find_safe_goal(self, map_data, path):
        for point in reversed(path):
            grid_point = self.world_to_grid(point)
            if self.is_point_safe(map_data, grid_point):
                return point
        return None

    def explore(self):
        self.get_logger().info('Starting exploration')
        if self.map is None or self.odom is None:
            self.get_logger().warn('No map or odometry available')
            return

        with self.map_lock:
            map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width)) 
            self.view_map(map_data)

        with self.odom_lock:
            current_pos = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)

        frontier = self.find_frontier(map_data)
        if frontier is not None:
            path = self.plan_path(map_data, current_pos, frontier)
            if path:
                smoothed_path = self.smooth_path(map_data, path)
                safe_goal = self.find_safe_goal(map_data, smoothed_path)
                if safe_goal:
                    goal = self.path_to_pose(safe_goal)
                    self.send_goal(goal)
                else:
                    self.get_logger().warn('No safe goal found')
                    self.exploring = False
            else:
                self.get_logger().warn('No path found to frontier')
                self.exploring = False
        else:
            self.get_logger().info('No frontiers found. Exploration complete.')
            self.exploring = False

    def find_frontier(self, map_data):
        frontiers = self.detect_frontiers(map_data)
        if frontiers:
            return self.select_best_frontier(frontiers, map_data)
        return None

    def detect_frontiers(self, map_data):
        frontier_cells = []
        for y in range(1, map_data.shape[0] - 1):
            for x in range(1, map_data.shape[1] - 1):
                if map_data[y, x] == 0:  # Free cell
                    if -1 in map_data[y-1:y+2, x-1:x+2]:  # Adjacent to unknown
                        frontier_cells.append((x, y))
        return frontier_cells

    def select_best_frontier(self, frontiers, map_data):
        if not frontiers:
            return None
        
        # Group frontiers
        groups = self.group_frontiers(frontiers, map_data)
        
        # Select the largest group
        largest_group = max(groups, key=lambda g: len(g))
        
        # Return the centroid of the largest group
        return self.calculate_centroid(largest_group)

    def group_frontiers(self, frontiers, map_data):
        groups = []
        for frontier in frontiers:
            added = False
            for group in groups:
                if self.is_adjacent(frontier, group[0], map_data):
                    group.append(frontier)
                    added = True
                    break
            if not added:
                groups.append([frontier])
        return groups

    def is_adjacent(self, cell1, cell2, map_data):
        return abs(cell1[0] - cell2[0]) <= 1 and abs(cell1[1] - cell2[1]) <= 1

    def calculate_centroid(self, group):
        x = sum(cell[0] for cell in group) / len(group)
        y = sum(cell[1] for cell in group) / len(group)
        return (int(x), int(y))

    def plan_path(self, map_data, start, goal):
        # Convert start and goal to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = goal  # goal is already in grid coordinates

        # Implement A* pathfinding
        path = self.astar(map_data, start_grid, goal_grid)
        
        if path:
            # Convert path back to world coordinates
            return [self.grid_to_world(cell) for cell in path]
        return None

    def world_to_grid(self, pos):
        x = int((pos[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        y = int((pos[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        return (x, y)

    def grid_to_world(self, cell):
        x = cell[0] * self.map.info.resolution + self.map.info.origin.position.x
        y = cell[1] * self.map.info.resolution + self.map.info.origin.position.y
        return (x, y)

    def astar(self, array, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[1] and 0 <= neighbor[1] < array.shape[0]:
                    if not self.is_point_safe(array, neighbor):
                        continue
                else:
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

        return False

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def smooth_path(self, map_data, path):
        if len(path) < 4:
            return path

        x = [p[0] for p in path]
        y = [p[1] for p in path]

        t = range(len(x))
        x_tup = si.splrep(t, x, k=3)
        y_tup = si.splrep(t, y, k=3)

        x_list = list(x_tup)
        y_list = list(y_tup)

        ipl_t = np.linspace(0.0, len(x) - 1, 100)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)

        smoothed_path = list(zip(rx, ry))
        return [p for p in smoothed_path if self.is_point_safe(map_data, self.world_to_grid(p))]
    
    def path_to_pose(self, point):
        goal = PoseStamped()
        goal.header = self.map.header
        goal.pose.position.x = point[0]
        goal.pose.position.y = point[1]
        goal.pose.orientation.w = 1.0
        return goal

    def send_goal(self, goal):
        self.get_logger().info(f'Sending goal: ({goal.pose.position.x}, {goal.pose.position.y})')
        self.nav_client.wait_for_server()
        self.future = self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal))
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.exploring = False
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