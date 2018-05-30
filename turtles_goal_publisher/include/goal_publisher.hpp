/* goal_publisher.hpp
 * This is the goal_publisher node which sends goals to the navigation stack.
 * In this case the car will drive the PSES parcours.
 *
 * This code is based on the ros tutorial: http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
 * @author: TUrtlES (tud.turtles@gmail.com)
 * @version: 1.0
 */

#ifndef TURTLES_GOAL_PUBLISHER_H
#define TURTLES_GOAL_PUBLISHER_H

// I N C L U D E S -------------------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// callback function for amcl
void amclCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr amclMsg, geometry_msgs::PoseWithCovarianceStamped* amclPose);


#endif
