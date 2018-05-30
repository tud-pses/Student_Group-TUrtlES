// parking_node.cpp

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "parking/CParking.hpp"

// M A I N - F U N C T I O N ---------------------------------------------------------
int main(int argc, char** argv) {
    ROS_INFO("enter ParkingNode");

    ros::init(argc, argv, "parking_node");
    ros::NodeHandle nh;
    CParking  parking(nh);

    PCB status = PCB_OK;
    ros::Rate rate(PARKING_LOOP_RATE);
    while(ros::ok() && status == PCB_OK) {
        status = parking.spin();

        ros::spinOnce();
        rate.sleep();
    } // WHILE

    ROS_INFO("Parking exit with status %d", status);

    return 0;
} // MAIN
