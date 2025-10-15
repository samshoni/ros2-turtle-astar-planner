#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import heapq  # For priority queue in A*

class TurtleAStarPlanner(Node):
    def __init__(self):
        super().__init__('turtle_astar_planner')
        self.get_logger().info('A* Planner Node started.')

        # Subscribe to the turtle's pose
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = Pose()

        # Publisher for controlling the turtle's velocity
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create the grid map for path planning
        self.create_grid()

        # Timer for control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.path = []
        self.path_index = 0
        self.reached_goal = False

        # Define goal in turtlesim coords (you can change this)
        self.goal_x = 8.0
        self.goal_y = 8.0

        # Start planning once pose received
        self.plan_started = False

    def create_grid(self):
        # Create a 10x10 grid representing turtlesim window
        grid_size = 10
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.grid_size = grid_size
        self.get_logger().info('Grid map created: 10x10 free space')

    def coords_to_grid(self, x, y):
        # Map turtlesim coordinates [0, 11] to grid indexes [0, grid_size-1]
        gx = min(max(int(x), 0), self.grid_size - 1)
        gy = min(max(int(y), 0), self.grid_size - 1)
        return gx, gy

    def grid_to_coords(self, gx, gy):
        # Map grid indexes back to center of cell coordinates
        cx = gx + 0.5
        cy = gy + 0.5
        return cx, cy

    def heuristic(self, node, goal):
        # Using Manhattan distance as heuristic
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def get_neighbors(self, node):
        neighbors = []
        directions = [(1,0),(-1,0),(0,1),(0,-1)]  # 4-connected grid
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                if self.grid[ny][nx] == 0:  # 0 means free cell
                    neighbors.append((nx, ny))
        return neighbors

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, None))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, cost, current, parent = heapq.heappop(open_set)
            if current == goal:
                # Reconstruct path
                path = [current]
                while parent:
                    path.append(parent)
                    parent = came_from.get(parent, None)
                path.reverse()
                return path
            if current not in came_from:
                came_from[current] = parent
            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, new_cost, neighbor, current))
        return []

    def pose_callback(self, msg):
        self.pose = msg
        if not self.plan_started:
            self.plan_path()
            self.plan_started = True

    def plan_path(self):
        start = self.coords_to_grid(self.pose.x, self.pose.y)
        goal = self.coords_to_grid(self.goal_x, self.goal_y)
        self.get_logger().info(f'Planning from {start} to {goal}')
        self.path = self.astar(start, goal)
        self.path_index = 0
        if self.path:
            self.get_logger().info(f'Path found with {len(self.path)} steps')
        else:
            self.get_logger().info('No path found!')
        self.reached_goal = False

    def control_loop(self):
        if not self.path or self.reached_goal:
            # No path or already at goal, stop the turtle
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # Current target point from path
        target_cell = self.path[self.path_index]
        target_x, target_y = self.grid_to_coords(target_cell[0], target_cell[1])

        # Compute distance and angle to target
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.pose.theta)

        twist = Twist()

        # Angle threshold to decide rotation vs forward
        angle_threshold = 0.1  # radians
        dist_threshold = 0.1  # distance to consider reached

        # Prioritize rotation then forward movement
        if abs(angle_diff) > angle_threshold:
            twist.angular.z = 2.0 * angle_diff
            twist.linear.x = 0.0
        elif distance > dist_threshold:
            twist.linear.x = 1.5 * distance
            twist.angular.z = 0.0
        else:
            # Reached current waypoint
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.reached_goal = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Reached goal!')
        
        self.cmd_pub.publish(twist)

        # Log info with throttling (avoid excessive prints)
        if self.get_clock().now().nanoseconds % 20 == 0:
            self.get_logger().info(f'Moving to waypoint {self.path_index}/{len(self.path)}, Distance: {distance:.2f}, Angle diff: {angle_diff:.2f}')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TurtleAStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

