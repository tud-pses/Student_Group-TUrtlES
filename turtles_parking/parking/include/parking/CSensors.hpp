// CSensors.hpp

#ifndef __CSENSORS_HPP
#define __CSENSORS_HPP

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include "nav_msgs/Odometry.h"
#include <stdint.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CSmoothing.hpp"

// D E F I N E S ---------------------------------------------------------------------
#define WHEEL_RADIUS 0.032                          // Radius of the robot-wheel
#define RAD_PER_TICK 0.7853981634                   // 2*PI/8 (Constants from pses_odom)
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412       // RAD_PER_TICK * WHEEL_RADIUS

// C L A S S - C S E N S O R S -------------------------------------------------------
class CSensors {
public:
    CSensors(ros::NodeHandle &nh);
    ~CSensors();

    float   getDistRight();                          // Returns distance from right UltraSonicSensor
    float   getDistLeft();                           // Returns distance from left UltraSonicSensor
    float   getDistFront();                          // Returns distance from front UltraSonicSensor
    double  getDrivenDist();                         // Return driven distance since last function call
    uint8_t getHallCounter();
    void    refreshSmoother();                       // Refreshes smoother

    void    startTrackingYaw();
    void    stopTrackingYaw();
    double  getCurrentYaw() const {
        return 2*m_CurrentYaw;
    } // GETCURRENTYAW

private:
    CSmoothing<float> smoothUSLeft;
    CSmoothing<float> smoothUSRight;
    CSmoothing<float> smoothUSFront;

    // ROS-Subscriber
    ros::Subscriber UltraSonicRightSub;
    ros::Subscriber UltraSonicLeftSub;
    ros::Subscriber UltraSonicFrontSub;
    ros::Subscriber HallSensorCountSub;
    ros::Subscriber ImuSub;

    ros::Time  m_OldTimestamp;
    uint8_t    m_HallCnt;
    uint8_t    m_CntInitialVal;
    double     m_CurrentYaw;
    bool       m_TrackYaw;

    void integrateEulerAngles(const double& dAngle, const double dt, double& angle); // From Odometry-package from pses-github

    // CALLBACK - Functions
    void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);
    void uslCallback(const sensor_msgs::Range::ConstPtr &uslMsg) { smoothUSLeft.addValue(uslMsg->range);}
    void usrCallback(const sensor_msgs::Range::ConstPtr &usrMsg) { smoothUSRight.addValue(usrMsg->range);}
    void usfCallback(const sensor_msgs::Range::ConstPtr &usfMsg) { smoothUSFront.addValue(usfMsg->range);}
    void hallcntCallback(const std_msgs::UInt8::ConstPtr &hallcntMsg) {
        uint8_t count = hallcntMsg->data;
        if (count > 0 && m_CntInitialVal < 1)
            m_CntInitialVal = count;

        m_HallCnt = count - m_CntInitialVal;
    }
}; // CLASS CSENSORS

#endif // __CSENSORS_HPP
