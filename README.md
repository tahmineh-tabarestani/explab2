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
![assignment2uml asli2](https://github.com/tahmineh-tabarestani/explab2/assets/80887743/f548748c-5f95-442c-b2c9-048760143ce8)
UML graph
The whole software structure is divided into 5 important parts:

1.Python nodes
2.Cpp nodes
3.Helper scripts
4.URDF
5.Packages
##/1. Python nodes/
Finite State Machine (FSM):

The FSM serves as the central node in the overall architecture. It utilizes the build_ontology_map.py class to handle functionalities such as adding, manipulating, and querying information. Additionally, it imports three crucial services from set_object_state.py to manage the state of the battery, base movement, and arm movement. To obtain information from the robot's camera, the FSM subscribes to the marker_publisher.cpp node, receiving the /image_id detected by the camera. This id is then communicated to the marker_server.cpp node to retrieve the corresponding information. When the robot intends to move, it publishes a Move Base Action to the move_base package, instructing the robot's base movement. Finally, the FSM utilizes a service response to obtain the pose, base movement state, and arm movement state from the robot_states.py node.

Robot States:

The Robot States node subscribes to the /odom topic from the robot's URDF to gather odometry information. It acts as the hub for various services, including base_movement_state and battery_level_state. Additionally, the Get_pose service is executed within this node, facilitating communication between the FSM node, helper scripts, and other components via the <response & request> mechanism.
##2. Cpp nodes/
Marker Server:

The Marker Server node is responsible for defining the properties of each room, such as the room ID and its details (position, connections, doors, last visit time). It establishes communication with the FSM node through a service (Request/Response) to transmit room information.

My Moveit:

Within the My Moveit node, all the necessary joint configurations are stored to detect boxes in the environment. This node communicates with the set_object_state.py helper script via a service to toggle the state of the robot's arm between True and False. When the arm state is True, indicating activation, this node becomes active and sends joint position information to the robot_assignment.urdf. Conversely, if the arm state is False, the robot arm ceases its operation.

##/3. Helper scripts/
The helper scripts, although they don't initiate any nodes, play a crucial role by importing essential functions used in the FSM node and robot_state node.

Build Ontology Map:

The Build Ontology Map script utilizes a service response from the robot_state.py node to retrieve the robot's current position in the map. It then updates the "is_in" method, which indicates the robot's current location. Additionally, this script facilitates the loading of the ontology_map defined in the armor package and ensures synchronization with the reasoner.

Set Object State:

The Set Object State script defines three significant services: battery_state, base_movement_state, and arm_movement_state. It sends service requests to the robot_state.py node to set the base movement state and battery level state. It also sends a service request to the my_moveit.cpp node to set the arm movement state. These services are essential for managing and updating the states of the battery, base movement, and arm movement in the system.

#/4. URDF/
The Robot Assignment file is generated using the MoveIt Setup Assistant. It contains a robot model and various topics that are useful for controlling the robot's base and arm movements. The robot URDF subscribes to the move_base topic to receive velocity commands via the /cmd_vel topic. It also publishes odometry information via the /odom topic to the robot_state.py node, which provides the current position of the robot in the ontology map. Additionally, the URDF publishes the /robot_camera/image_box topic to the marker_publisher.cpp node to transmit images detected by the robot's camera. It also publishes base information using the /scan topic and the TF (Transform) tree via the /tf topic, which is used by slam_gmapping to update the robot's position in the map.
##/5. Packages/
Aruco Ros:
The Aruco Ros package utilizes the Aruco library for real-time marker-based 3D pose estimation using AR markers. In this assignment, the marker_publisher.cpp node from the Aruco Ros package is used to publish the /image_id detected from the /robot_camera/image_box topic, which is then received by the FSM node.

SLAM Gmapping:
The SLAM Gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping) capabilities. The slam_gmapping node in this package generates a 2D occupancy grid map based on laser and pose data collected by a mobile robot. It utilizes the /scan and TF topics generated by the robot URDF to update the position of the base_link and arm_link in the map.

Move Base:
The Move Base package provides an action implementation that allows a mobile robot to reach a given goal in the world. The move_base node combines global and local planners to accomplish global navigation tasks. It receives an action from the FSM node and publishes velocity commands via the /cmd_vel topic to the robot URDF.

Assignment Moveit:
The Assignment Moveit package provides a setup assistant that assists in configuring robot models. By running the command roslaunch moveit_setup_assistant setup_assistant.launch in the terminal, the Moveit setup assistant is launched. It enables the definition of various properties for robot models and generates a new URDF file and package, including essential configurations and launch files for using the robot model.

aRMOR:
aRMOR is a powerful management system for single and multi-ontology architectures under ROS. It allows for the loading, querying, and modification of ontology maps. In the context of this assignment, aRMOR is used to add and manipulate rooms and doors in the ontological_map.owl file. It provides the query results as feedback to the FSM node. The build_ontology_map.py helper script works in parallel with aRMOR to load the ontology map and synchronize with the armor_client.py whenever the ontology is updated.
##Temporal diagram
![temporal drawio assignment2 AKHARI](https://github.com/tahmineh-tabarestani/explab2/assets/80887743/a1f62a82-3a63-47cc-a09b-9fbee2956b48)

In the initial phase, a launch file is used to start all the nodes and packages, excluding finite_state_machine.py. The finite_state_machine.py node automatically begins functioning after a delay of 20 seconds. This delay is necessary as it allows time for the simulation to incorporate the robot model into the environment and enable the execution of functions defined in the FSM node.

During the SETUP_MAP step, slam_gmapping subscribes to the /scan and /tf topics generated by the robot URDF. Additionally, my_moveit.cpp subscribes to the /tf information, which is utilized for the /arm_controller. The robot's arm is equipped with a camera at its tip, enabling it to detect marker IDs in the surrounding world. The acquired image_od from the camera is published to the FSM node. The FSM node utilizes a service (request and response) to obtain information about the detected room ID.

Upon completion of the SETUP_MAP step, the GO_IN_ROOM state is initiated. The FSM sends an action to the move_base package, resulting in the publication of /cmd_vel to the robot URDF, which in turn moves the robot within the map. Odometry information related to the robot's movement is sent to the robot_state.py node, and this node stores the information in the Get_pose.srv.

It's important to note that the FSM node encompasses five states, but only two of them are demonstrated here. At the beginning of each state, changes occur in the /arm_movement_state and /base_movement_state, while the /battery_level_state starts consuming after the first state is initiated.

##Usage:
to use the assignment, follow the steps below:

1.Create a repository named "assignment_ws" by executing the following commands:
$ mkdir assignment_ws /src
$ cd src

2.Fork the repository from my GitHub by running the command:
$ git clone https://github.com/tahmineh-tabarestani/explab2

3.Install the necessary packages to run the assignment, including:

MoveIt package with all its dependencies
Move base
Slam_gmapping
SMACH viewer
Navigate to the workspace and build the packages by running:
$ cd ..
$ catkin_make
5.Launch the assignment using the following command:
$ roslaunch assignment2 assignment.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
Note: Please be aware that there is a 20-second delay before the assignment starts working. This delay allows time for the robot to be added to the environment and for the MoveIt package to initialize.

6.Optionally, if you want to change the battery level manually, use the following command:
$ rosservice call /state/set_battery_level "battery_level: <value>"
Replace <value> with the desired battery level.

WARNING:

When launching the files, there is a 20-second delay before the assignment starts working. This is necessary to allow for the initialization of the robot in the environment and the execution of the MoveIt package.

In the scripts, the system path is specified to use the "carried" package. If you cloned the repository in a different path, you need to update this line of code accordingly.

Running the assignment multiple times may generate excessive logs, which can slow down your computer. To resolve this, use the following command to clear the logs:
$ rosclean purge
By following these steps, you can successfully use the assignment and run the necessary launch files for your application.
##Hypothesis and environment

The system revolves around the following features:

1. The robot utilizes its arm to locate the marker_id and initiates automatic patrolling on the map.
2. The camera on board the robot allows users to visualize the robot's perspective.
3. Whenever the robot moves to a new location, the map updates accordingly, and the changes can be observed through Rviz.

However, there are certain limitations to the system:

1. Occasionally, the robot faces difficulty in making accurate turns and may inadvertently collide with walls. This limitation could be attributed to suboptimal angular velocity settings compared to linear velocity.
2. The map updates at a relatively slow pace, leading to the robot generating incorrect paths. However, once the map is updated, it generates the correct path.
3. Sometimes, while the robot is in the process of exploring a room, the base starts moving prematurely. This issue is caused by the high control frequency.

To enhance the system's performance, the following technical improvements can be considered:

1. Implementing an octomap to map the environment could be a valuable solution. The robot is equipped with a 3D sensor that captures information in the `/head_mount_kinect/depth_registered/points` topic. This data can be utilized in an octomap to create a point cloud representation of the environment.
2. Addressing the issue of the robot occasionally selecting incorrect paths and making wrong decisions can significantly improve system efficiency. Finding a resolution to this problem would reduce the time taken by the robot to reach its target, ultimately enhancing overall performance.




