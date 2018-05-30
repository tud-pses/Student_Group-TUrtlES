/* base_controller.hpp
 * This is the base controller node for the robot which can be used in combination with the navigation stack of ros.
 * It uses geomtry_msgs as input and sends motor and steering commands to uc_bridge.
 *
 * @author: TUrtlES (tud.turtles@gmail.com)
 * @version: 1.0
 */

#ifndef TURTLES_BASE_CONTROLLER_H
#define TURTLES_BASE_CONTROLLER_H

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
#include <cmath>


double pi_val = 3.1415926;

// converts a rad value to a degree value
double radToDeg(double angle);

// callback function for cmd_vel
void cmd_velCallback(geometry_msgs::Twist::ConstPtr cmd_velMsg , geometry_msgs::Twist* cmd_vel );


#endif

