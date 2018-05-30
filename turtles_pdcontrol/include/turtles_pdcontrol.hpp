/* turtles_pdcontrol.hpp
 * This is the turtles_pdcontrol node which is a simple wall follow implementation based on a PD controller.
 *
 *
 * This code is based on the ros tutorial: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
 * @author: TUrtlES (tud.turtles@gmail.com)
 * @version: 1.0
 */
#ifndef TURTLES_PDCONTROL_H
#define TURTLES_PDCONTROL_H

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>

// PD controller values
double t = 0.03;
double y = 0;
double ya = 0;
double e = 0;
double ea = 0;


double kp = 4000;
double ki =  0;
double kd = 1500;

/* Other possible value constelation
 * double kp = 5000;
 * double ki =  0;
 * double kd = 2000;
*/

// hall sensor counter
int cnt = 0;

// callback function for left ultra sonic sensor
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl);

// callback function for front ultra sonic sensor
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf);

// callback function for right ultra sonic sensor
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr);


#endif
