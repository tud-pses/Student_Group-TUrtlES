// CMovement.hpp

#ifndef __CMOVEMENT_HPP
#define __CMOVEMENT_HPP

// I N C L U D E S ------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>

// D E F I N E S ---------------------------------------------------------------------
#define MIN_MOTOR_SPEED     -400
#define MAX_MOTOR_SPEED     800
#define MAX_STEERING_LEVEL  800

// C L A S S - M O V E M E N T -------------------------------------------------------
class CMovement {
public:
    CMovement(ros::NodeHandle &nh);
    ~CMovement();

    void setMotorSpeed(int nMotorSpeed);
    void setSteeringLevel(int nSteeringLevel);
    int  getMotorSpeed();
    int  getSteeringLevel();

 private:
    // ROS Publisher
    ros::Publisher MotorControl;
    ros::Publisher SteeringControl;

    // Messages for Callback
    std_msgs::Int16 MotorMsg;
    std_msgs::Int16 SteeringMsg;

    bool bDataChanged;

    int m_nMotorSpeed;
}; // CLASS CMOVEMENT

#endif // __CMOVEMENT_HPP
