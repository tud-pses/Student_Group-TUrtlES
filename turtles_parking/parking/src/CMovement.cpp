// CMovement.cpp

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CMovement.hpp"

// C L A S S - C M O V E M E N T -----------------------------------------------------
CMovement::CMovement(ros::NodeHandle &nh) {
    MotorControl    = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);

    SteeringControl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

    // Wait until nodes connected
    ros::Rate poll_rate(100);
    while((SteeringControl.getNumSubscribers() == 0) || (MotorControl.getNumSubscribers() == 0))
        poll_rate.sleep();
} // CONSTRUCTOR

CMovement::~CMovement() {
    setMotorSpeed(0);
    setSteeringLevel(0);

    SteeringControl.shutdown();
    MotorControl.shutdown();
} // DESTRUCTOR

void CMovement::setMotorSpeed(int nMotorSpeed) {
    if (nMotorSpeed <= MAX_MOTOR_SPEED && nMotorSpeed >= MIN_MOTOR_SPEED)
        MotorMsg.data = nMotorSpeed;
    else
        MotorMsg.data = 0;

    m_nMotorSpeed = MotorMsg.data;

    MotorControl.publish(MotorMsg);
} // SETMOTORSPEED

void CMovement::setSteeringLevel(int nSteeringLevel) {
    if (nSteeringLevel >= MAX_STEERING_LEVEL)
        SteeringMsg.data = MAX_STEERING_LEVEL;
    else if (nSteeringLevel <= -MAX_STEERING_LEVEL)
        SteeringMsg.data = -MAX_STEERING_LEVEL;
    else
        SteeringMsg.data = nSteeringLevel;

    SteeringControl.publish(SteeringMsg);
} // SETSTEERINGLEVEL

int CMovement::getMotorSpeed() {
    return MotorMsg.data;
} // GETMOTORSPEED

int CMovement::getSteeringLevel() {
    return SteeringMsg.data;
} // GETSTEERINGLEVEL
