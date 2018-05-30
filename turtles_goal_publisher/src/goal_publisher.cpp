// goal_publisher.cpp

// I N C L U D E S -------------------------------------------------------------------
#include "goal_publisher.hpp"


// callback function for amcl
void amclCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr amclMsg, geometry_msgs::PoseWithCovarianceStamped* amclPose)
{
  *amclPose = *amclMsg;
}



// goal_publisher node
int main(int argc, char** argv){
  ros::init(argc, argv, "goal_publisher");

  // node handler for amcl subscribtion
  ros::NodeHandle nh;

  // amcl pose declaration
  geometry_msgs::PoseWithCovarianceStamped amclPose;

  // subscribtion of amcl_pose topic
  ros::Subscriber amclSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/amcl_pose", 1, boost::bind(amclCallback, _1, &amclPose));

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Definition of the array allGoals which has 4 waypoints that lead to a full round of the PSES parcours.
  move_base_msgs::MoveBaseGoal allGoals[4];
  //goal 1
  allGoals[0].target_pose.pose.position.x = 12.605;
  allGoals[0].target_pose.pose.position.y = -4.419;
  allGoals[0].target_pose.pose.orientation.w = 1.0;
  //goal 2
  allGoals[1].target_pose.pose.position.x = 19.739;
  allGoals[1].target_pose.pose.position.y = 0.534;
  allGoals[1].target_pose.pose.orientation.w = 1.0;
  //goal 3
  allGoals[2].target_pose.pose.position.x = 27.559;
  allGoals[2].target_pose.pose.position.y = -2.575;
  allGoals[2].target_pose.pose.orientation.w = 1.0;
  //goal 4
  allGoals[3].target_pose.pose.position.x = 23.499;
  allGoals[3].target_pose.pose.position.y = -7.037;
  allGoals[3].target_pose.pose.orientation.w = 1.0;

  // connection the navigation stack action server
  ROS_INFO("Wainting for the action server...");
  ac.waitForServer();
  ROS_INFO("Server started...");
  move_base_msgs::MoveBaseGoal goal;


  ROS_INFO("Sending goal");
  goal = allGoals[0];
  // waypoint header with the map topic and the time for goal execution, which is now in this case
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  // send the robot to the first waypoint
  ac.sendGoal(goal);

  // this variables are used later for the distance computation of the robot to its next goal
  double distance = 0;
  double x_diff = 0;
  double y_diff = 0;

  // goal_counter saves the achieved waypoints
  int goal_counter = 0;

  // loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    // calculate distance to current goal
    // sqrt(x² + y²) = distance
    x_diff = amclPose.pose.pose.position.x - allGoals[goal_counter].target_pose.pose.position.x;
    y_diff = amclPose.pose.pose.position.y - allGoals[goal_counter].target_pose.pose.position.y;
    distance = sqrt(pow(x_diff,2)+pow(y_diff,2));
    ROS_INFO("x_diff: %f", amclPose.pose.pose.position.x );
    ROS_INFO("distance to goal: %f", distance );
    ROS_INFO("goal counter: %i", goal_counter );


    // set the next goal if the distance of the robot is <= 4.0 to its actual goal
    if(distance <= 4.0){
      // increse goal_counter
      goal_counter =  goal_counter + 1;
      if(goal_counter > 3){
        goal_counter = 0;
      }
      // add time and frame to header
      allGoals[goal_counter].target_pose.header.frame_id = "map";
      allGoals[goal_counter].target_pose.header.stamp = ros::Time::now();
      // publish new goal
      goal = allGoals[goal_counter];
      ac.sendGoal(goal);
    }
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();

  }


  ros::spin();
  return 0;
}
