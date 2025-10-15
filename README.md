# ROS2 Turtle A* Path Planner

This project demonstrates a basic implementation of the A* path planning algorithm in ROS2 using the turtlesim simulator. It subscribes to the turtle's pose, computes an optimal path on a grid, and navigates the turtle along the resulting waypoints.

## Features
- Pose subscription and velocity control
- Grid map creation for environment representation
- Implementation of the A* pathfinding algorithm
- Turtle follows the planned path step-by-step with smooth control

## Setup Instructions

1. Launch the turtlesim simulator:

ros2 run turtlesim turtlesim_node


2. Run the path planning node:

ros2 run turtle_path_planner astar_planner



---

## Screenshots

### Environment and Path Planning

![Path Planning Environment](images/Screenshot%20from%202025-10-15%2023-31-32.png)

### Turtle Moving Toward Goal

![Turtle Moving](images/Screenshot%20from%202025-10-15%2023-32-12.png)


---

## Future Work
- Add dynamic obstacle handling in the grid map
- Visualize the planned path in RViz for better insight
- Add interactive GUI or service to set goal positions dynamically

## Author
Sam Shoni

