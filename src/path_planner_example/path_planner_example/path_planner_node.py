#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator
from heapq import heappush, heappop
import math

class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        self.basic_navigator = BasicNavigator()  # Access Global Costmap

        # Creating a new service "create_plan", which is called by our Nav2 C++ planner plugin
        # to receive a plan from us.
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    def create_plan_cb(self, request, response):
        # Getting all the information to plan the path
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()

        # Access the global costmap
        global_costmap = self.basic_navigator.getGlobalCostmap()

        # Convert the costmap to a 2D grid
        costmap_grid, resolution, origin_x, origin_y = convert_costmap_to_grid(global_costmap)

        # Call the A* function to create a path
        response.path = create_a_star_plan(start_pose, goal_pose, time_now, costmap_grid, resolution, origin_x, origin_y)
        return response

def convert_costmap_to_grid(costmap):
    """
    Converts a nav2_msgs/Costmap message to a 2D grid.

    The Costmap message (nav2_msgs/msg/Costmap) has:
      - metadata (CostmapMetaData): includes size_x, size_y, resolution, and origin
      - data: a 1D array of size size_x * size_y

    We'll convert this into a 2D Python list (grid).
    """
    width = costmap.metadata.size_x
    height = costmap.metadata.size_y
    resolution = costmap.metadata.resolution
    origin_x = costmap.metadata.origin.position.x
    origin_y = costmap.metadata.origin.position.y

    flat_data = list(costmap.data)

    # Each cell in flat_data represents a cost:
    # 0 is free, 
    # 1-100 are different levels of cost/obstacle,
    # 255 might represent unknown or lethal obstacle.
    # For now, let's treat anything > 0 as an obstacle.
    # If you want a more nuanced approach, adjust this.
    grid = []
    for i in range(height):
        row = flat_data[i*width:(i+1)*width]
        # Convert cell costs: 0 = free, anything else = blocked
        # Or you can do: cell == 0 means free, cell > 0 means blocked
        processed_row = [0 if cell == 0 else 1 for cell in row]
        grid.append(processed_row)

    return grid, resolution, origin_x, origin_y

def create_a_star_plan(start, goal, time_now, costmap, resolution, origin_x, origin_y):
    """
    Creates a path using the A* algorithm between start and goal positions.

    Args:
        start (PoseStamped): Starting pose in world coordinates.
        goal (PoseStamped): Goal pose in world coordinates.
        time_now: Current ROS time for timestamping.
        costmap: 2D list representing the environment grid (0=free, 1=blocked).
        resolution (float): The resolution of the costmap in m/cell.
        origin_x (float): The origin X of the costmap in world coordinates.
        origin_y (float): The origin Y of the costmap in world coordinates.

    Returns:
        Path: A ROS Path message with the planned path.
    """

    def heuristic(node, goal_node):
        # Euclidean distance as the heuristic
        return math.sqrt((node[0] - goal_node[0]) ** 2 + (node[1] - goal_node[1]) ** 2)

    def get_neighbors(node, grid):
        # 4-directional movement
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        neighbors = []
        x_size = len(grid[0])
        y_size = len(grid)
        for dx, dy in directions:
            ny = node[0] + dy
            nx = node[1] + dx
            if 0 <= ny < y_size and 0 <= nx < x_size and grid[ny][nx] == 0:
                neighbors.append((ny, nx))
        return neighbors

    # Convert start and goal positions (in world coordinates) to grid indices
    # Note: grid indexing: row = (y - origin_y)/resolution, col = (x - origin_x)/resolution
    start_row = int((start.pose.position.y - origin_y) / resolution)
    start_col = int((start.pose.position.x - origin_x) / resolution)
    goal_row = int((goal.pose.position.y - origin_y) / resolution)
    goal_col = int((goal.pose.position.x - origin_x) / resolution)

    start_index = (start_row, start_col)
    goal_index = (goal_row, goal_col)

    # Check bounds
    if not (0 <= start_index[0] < len(costmap) and 0 <= start_index[1] < len(costmap[0])):
        # Start out of bounds, return empty path
        return Path()

    if not (0 <= goal_index[0] < len(costmap) and 0 <= goal_index[1] < len(costmap[0])):
        # Goal out of bounds, return empty path
        return Path()

    # If start or goal are blocked, return empty path
    if costmap[start_index[0]][start_index[1]] != 0 or costmap[goal_index[0]][goal_index[1]] != 0:
        return Path()

    # A* algorithm
    open_list = []
    heappush(open_list, (0 + heuristic(start_index, goal_index), start_index))  # (f, node)
    came_from = {}
    g_score = {start_index: 0}
    f_score = {start_index: heuristic(start_index, goal_index)}

    while open_list:
        _, current = heappop(open_list)

        if current == goal_index:
            # Reconstruct path
            path = Path()
            path.header.frame_id = goal.header.frame_id
            path.header.stamp = time_now

            # Backtrack from goal to start
            path_points = [current]
            while current in came_from:
                current = came_from[current]
                path_points.append(current)

            path_points.reverse()

            # Convert grid indices back to world coordinates and create poses
            for (r, c) in path_points:
                px = c * resolution + origin_x
                py = r * resolution + origin_y
                pose = PoseStamped()
                pose.header.frame_id = goal.header.frame_id
                pose.header.stamp = time_now
                pose.pose.position.x = px
                pose.pose.position.y = py
                path.poses.append(pose)

            return path

        for neighbor in get_neighbors(current, costmap):
            # cost to move from current to neighbor is assumed 1 cell
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_index)
                # If neighbor not in open_list, push it
                # (We don't directly check membership in open_list for performance reasons,
                # typically we rely on g_score improvements)
                heappush(open_list, (f_score[neighbor], neighbor))

    # Return an empty path if no solution
    return Path()

def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()

    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass

    path_planner_node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
