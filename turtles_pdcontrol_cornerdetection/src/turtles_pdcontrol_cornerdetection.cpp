#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>




double t = 0.03;
double y = 0;
double ya = 0;
double e = 0;
double ea = 0;

double rangeValues [5] = {};
double cornerStart = 0;
int cornerTimer = 0;

/*
double kp = 4000;
double ki =  0;
double kd = 1500;

 RESULT: lief ganz gut
*/

double kp = 5000;
double ki =  0;
double kd = 2000;

bool var = false;
int cnt = 0;



// gets called whenever a new message is availible in the input puffer
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

// gets called whenever a new message is availible in the input puffer
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

// gets called whenever a new message is availible in the input puffer
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}

// gets called whenever a new message is availible in the input puffer
void imuCallback(sensor_msgs::Imu::ConstPtr imuMsg, sensor_msgs::Imu* imu)
{
  *imu = *imuMsg;
}

// gets called whenever a new message is availible in the input puffer
void filterOrientationCallback(std_msgs::Float64::ConstPtr filterOrientationMsg, std_msgs::Float64* filterOrientation)
{
  *filterOrientation = *filterOrientationMsg;
}


int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "turtles_pdcontrol_cornerdetection");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  sensor_msgs::Range usr, usf, usl;
  sensor_msgs::Imu imu;
  std_msgs::Int16 motor, steering;
  std_msgs::Float64 halldt8, halldt1, filterOrientation;
  std_msgs::UInt8 hallcnt;


  // generate subscriber for sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));
  ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>(
       "/uc_bridge/imu", 10, boost::bind(imuCallback, _1, &imu));
  ros::Subscriber filterOrientationSub = nh.subscribe<std_msgs::Float64>(
      "/filterOrientation", 10, boost::bind(filterOrientationCallback, _1, &filterOrientation));


  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("turtles pdcontrol with corner detection!");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    //set speed
    motor.data = 200;

    // deviation
    e = 0.5 - usl.range;
    y = e*kp+kd*(e-ea)/t;
    ea=e;

    //save last five range values to an array
    double rangeValues_temp [5] = {};

    for(int i = 0; i<4;i++){
      rangeValues_temp[i+1] = rangeValues[i];
    }
    rangeValues_temp [0] = usl.range;
    rangeValues [0] = rangeValues_temp [0];
    rangeValues [1] = rangeValues_temp [1];
    rangeValues [2] = rangeValues_temp [2];
    rangeValues [3] = rangeValues_temp [3];
    rangeValues [4] = rangeValues_temp [4];
    //ROS_INFO("%f %f %f %f %f",rangeValues [0],rangeValues [1],rangeValues [2],rangeValues [3],rangeValues [4]);

    double range_temp = rangeValues [0]+rangeValues [1]+rangeValues [2]+rangeValues [3]+rangeValues [4];
    if(range_temp > 4.5){
      cornerStart = filterOrientation.data;
      ROS_INFO("Kurve erkannt ");
    }

    cornerTimer += 1;
    if((filterOrientation.data - cornerStart) > 90){
      ea = 0;
      y =0;
      cornerStart = 0;
      ROS_INFO("Kurve gefahren ");
    }

    int max_sterring = 900;
    if(y > max_sterring){
      steering.data = max_sterring;
    }
    else if(y < -max_sterring){
        steering.data = -max_sterring;
    }
    else{
      steering.data = (int) y;
    }

    // publish command messages on their topics
    motorCtrl.publish(motor);
    steeringCtrl.publish(steering);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
