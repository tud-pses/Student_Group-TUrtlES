// turtles_pdcontrol.cpp

// I N C L U D E S -------------------------------------------------------------------
#include "turtles_pdcontrol.hpp"


// callback function for left ultra sonic sensor
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

// callback function for front ultra sonic sensor
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

// callback function for right ultra sonic sensor
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}

int main(int argc, char** argv)
{
  // node initialization
  ros::init(argc, argv, "turtles_pdcontrol");
  ros::NodeHandle nh;

  // sensor message container for ultra sonic sensor
  sensor_msgs::Range usr, usf, usl;
  // container for motor and steering data
  std_msgs::Int16 motor, steering;

  // generate subscriber for ultra sonic sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));

  // generate motor and steering data message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("turtles pd control");

  // loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    // constant speed of 300
    motor.data = 300;

    // deviation
    e = 0.8 - usl.range;
    y = e*kp+kd*(e-ea)/t;
    ea=e;

    // limit of the stering data
    int target = 900;
    if(y > target){
      steering.data = target;
    }
    else if(y < -target){
        steering.data = -target;
    }
    else{
      steering.data = (int) y;
    }

    // publish command messages on their topics
    motorCtrl.publish(motor);
    steeringCtrl.publish(steering);

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a constant loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
