# explab2
##Introduction
The objective of this assignment is to design a software architecture for the TMRC1.0 assignment. The assignment involves demonstrating robot movements on a map using Gazebo and Rviz. Additionally, the robot is equipped with a manipulator that includes a camera for environment detection. To address this challenge![mapassign2](https://github.com/tahmineh-tabarestani/explab2/assets/80887743/b4367c28-aea4-4881-8fa4-a77e3b5d88b0)


##tools were utilized:

1. aRMOR: This tool was employed to load the topological map, manipulate and add rooms, and retrieve information through queries.

2. MoveIt: MoveIt was used to construct the URDF for the robot model and generate trajectory plans for moving the robot's arm_link.

3. MoveBase: MoveBase facilitated the movement of the robot's base_link.

4. Slam_gmapping: This tool was utilized to map the environment detected by the robot.

5. SMACH Viewer: SMACH Viewer provided visual representation and monitoring of the robot's state machine during operation.

By leveraging these tools, a comprehensive software architecture was developed to enable efficient robot movements, incorporating capabilities such as environment detection, motion planning, mapping, and state monitoring.
##Software architecture
