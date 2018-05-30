#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>
#include <math.h>




double acc_x = 0;
double acc_y = 0;
double acc_z = 0;

double gyr_x = 0;
double gyr_y = 0;
double gyr_z = 0;

double phi = 0;
double theta = 0;

double anglex = 0;
double angley = 0;
double anglez = 0;

double y = 0;
double ya = 0;
double ydeg =0;

double PI = 3.14159265;


// gets called whenever a new message is availible in the input puffer
void imuCallback(sensor_msgs::Imu::ConstPtr imuMsg, sensor_msgs::Imu* imu)
{
  *imu = *imuMsg;
}


int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "filter");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  sensor_msgs::Imu imu;
  std_msgs::Float64 filterOrientation;

  // generate subscriber for sensor messages
  ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>(
       "/uc_bridge/imu", 10, boost::bind(imuCallback, _1, &imu));

  // generate control message publisher
  ros::Publisher filterOrientationPub =
      nh.advertise<std_msgs::Float64>("filterOrientation", 10);

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(100);
  while (ros::ok())
  {

    acc_x = imu.linear_acceleration.x;
    acc_y = imu.linear_acceleration.y;
    acc_z = imu.linear_acceleration.z;

    gyr_x = imu.angular_velocity.x * 180/PI;
    gyr_y = imu.angular_velocity.y * 180/PI;
    gyr_z = imu.angular_velocity.z * 180/PI;


    phi = atanf(acc_y/acc_z)*(180/PI);
    theta = atanf((-acc_x))/sqrt((acc_y)*(acc_y)+(acc_z)*(acc_z))*(180.0/PI);
    anglex += gyr_x*0.01;

    // ROS_INFO("anglex %f", anglex);
    double multipliedPhi = 0.01316*phi;
    double multipliedAngleX = 0.98684*anglex;
    double anglex2 = multipliedPhi+multipliedAngleX;

    angley += gyr_y*0.01;
    //angley =(0.98684) * angley + (0.01316)*theta;

    double d = (0.98684*angley);
    double e = (0.01316*theta);
    double f = d+e;

    anglez += 0.01* gyr_z;

    //Filter
    y =1-(ya +(anglez - ya) * 0.01/(1.5 +0.01));
    ya = y;
    ydeg = y*(180/PI);

    ROS_INFO("anglez %f", ydeg );

    // build filterOrientation message
    //filterOrientation.header.stamp = ros::Time::now();
    //filterOrientation.header.frame_id = "filterOrientation";

    filterOrientation.data = y;
    filterOrientationPub.publish(filterOrientation);


    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
