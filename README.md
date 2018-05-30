
How to getting started with the navigation stack:
- got to /turtles_launch
- start roslaunch pses_ucbridge uc_bridge.launch
- start ./gyro_setup.sh 
- start roslaunch robot.launch
- start roslaunch move_base_teb.launch
- start rosrun turtles_goal_publisher turtles_goal_publisher

How to getting started with the gmapping:
- start roslaunch pses_ucbridge uc_bridge.launch
- start roslaunch robot.launch
- start rosrun gmapping slam_gmapping scan:=scan _odom_frame:=odom


How to getting started with the parking sequence:
- start roslaunch pses_ucbridge uc_bridge.launch
- start rosrun turtles_parking turtles_parking_node 



How to getting started with the pd controller:
- start roslaunch pses_ucbridge uc_bridge.launch
- start rosrun turtles_pdcontrol turtles_pdcontrol



------------------------------------------------------------------------------------------------
Short discription of the nodes:

turtles_base_controller:
The turtles base controller node is taking the values from the teb local planner and outputs them to the car.

turtles_goal_publisher:
This node is publishing goals(waypoints).

turtles_robot_tf:
This node is used to publish transformations.

turtle_nav:
This folder containes all parameter files.

turtles_laser_scan:
This folder containes a launch file to create a laserscan.

turtles_pdcontrol:
This node is a simple wall follow node based on a PD-controller.

turtles_parking:
This package is used for the parking process. It implements the abbility for detecting a parking space and useing the space for an autonomous parking.




------------------------------------------------------------------------------------------------
Not used nodes:

turtles_filter:
We used this node to apply a filtering algorithm to the IMU data.

turtles_pdcontrol_cornerdetect:
This node is a simple wall follow node based on a PD-controller. The curve is beeing detected and the controller is disabeld during the curve driving process.

turtles_hough: This node implements a huffman transformation. But the results were not satisfying enough for the parking detecting.

------------------------------------------------------------------------------------------------
Changes of the given pses packages:

- pses_ucbridge: change the variable base_footprint to base_link




