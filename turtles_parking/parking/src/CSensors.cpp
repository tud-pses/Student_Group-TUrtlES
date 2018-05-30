// CSensors.cpp

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>
#include <stdint.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CSensors.hpp"
#include "CSmoothing.hpp"
#include "CHelpers.hpp"

// C L A S S - C S E N S O R S -------------------------------------------------------
CSensors::CSensors(ros::NodeHandle &nh) {
    // generate subscriber for sensor messages
    HallSensorCountSub = nh.subscribe<std_msgs::UInt8>(
        "/uc_bridge/hall_cnt", 10, &CSensors::hallcntCallback, this);
    UltraSonicLeftSub = nh.subscribe<sensor_msgs::Range>(
          "/uc_bridge/usl", 10, &CSensors::uslCallback, this);
    UltraSonicRightSub = nh.subscribe<sensor_msgs::Range>(
          "/uc_bridge/usr", 10, &CSensors::usrCallback, this);
    UltraSonicFrontSub = nh.subscribe<sensor_msgs::Range>(
          "/uc_bridge/usf", 10, &CSensors::usfCallback, this);
    ImuSub = nh.subscribe<sensor_msgs::Imu>(
          "/uc_bridge/imu", 10, &CSensors::imuCallback, this);

     // Wait for all subscribers get connected
     ros::Rate poll_rate(100);
     while ((UltraSonicFrontSub.getNumPublishers() == 0) ||
            (UltraSonicLeftSub.getNumPublishers() == 0) ||
            (UltraSonicRightSub.getNumPublishers() == 0) ||
            (HallSensorCountSub.getNumPublishers() == 0))
        poll_rate.sleep();

     m_HallCnt        = 0;
     m_CntInitialVal  = 0;
     m_OldTimestamp   = ros::Time::now();
     m_TrackYaw       = false;
     m_CurrentYaw     = 0.0;

     // "Warming" up Buffers
     refreshSmoother();
} // CONSTRUCTOR

CSensors::~CSensors() {
    HallSensorCountSub.shutdown();
    UltraSonicFrontSub.shutdown();
    UltraSonicLeftSub.shutdown();
    UltraSonicRightSub.shutdown();
    ImuSub.shutdown();
} // DESTRUCTOR

void CSensors::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
    double dt = (imuMsg->header.stamp - m_OldTimestamp).toSec();
    double yaw = 0.0;
    integrateEulerAngles(imuMsg->angular_velocity.z, dt, yaw);

    m_OldTimestamp = imuMsg->header.stamp;

    if (!m_TrackYaw)
        return;

    if (fabs(yaw) < IMU_CALLBACK_TRESHOLD)
        return;

    m_CurrentYaw += yaw;
} // IMUCALLBACK

void CSensors::integrateEulerAngles(const double& dAngle, const double dt, double& angle) {
    double result = angle + dAngle * dt;

    if (result > M_PI)
      angle = -2 * M_PI + result;
    else if (result < -M_PI)
      angle = 2 * M_PI + result;
    else
      angle = result;
} // INTEGRATEEULERANGLES (From pses-github)

void CSensors::startTrackingYaw() {
    m_CurrentYaw = 0.0;
    m_TrackYaw   = true;
} // STARTTRACKINGYAW

void CSensors::stopTrackingYaw() {
    m_TrackYaw = false;
} // STOPTRACKINGYAW

double CSensors::getDrivenDist() {
    return m_HallCnt * WHEEL_DIAMATER * M_PI / HALL_TICKS_PER_TURN;
} // GETDRIVENDIST

float CSensors::getDistRight() {
    return smoothUSRight.getAverage();
} // GETDISTRIGHT

float CSensors::getDistLeft() {
    return smoothUSLeft.getAverage();
} // GETDISTLEFT

float CSensors::getDistFront() {
    return smoothUSFront.getAverage();
} // GETDISTFRONT

void CSensors::refreshSmoother() {
    for (int i = 0; i < MAX_ARRAY_SIZE; i++) {
        getDistFront();
        getDistLeft();
        getDistRight();
    } // FOR
} // REFRESHSMOOTHER
