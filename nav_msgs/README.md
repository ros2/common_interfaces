# nav_msgs

This package provides several messages and services for robotic navigation.

For more information about the navigation2 stack in ROS 2, see https://ros-planning.github.io/navigation2/.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)
* [GridCells](GridCells.msg): An array of cells in a 2D grid.
* [MapMetaData](MapMetaData.msg): Basic information about the characteristics of the OccupancyGrid.
* [OccupancyGrid](OccupancyGrid.msg): Represents a 2-D grid map, in which each cell represents the probability of occupancy.
* [Odometry](Odometry.msg): This represents an estimate of a position and velocity in free space.
* [Path](Path.msg): An array of poses that represents a Path for a robot to follow.

## Services (.srv)
* [GetMap](GetMap.srv): Get the map as a nav_msgs/OccupancyGrid.
* [GetPlan](GetPlan.srv): Get a plan from the current position to the goal Pose.
* [SetMap](SetMap.srv): Set a new map together with an initial pose.

## Quality Declaration
This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
