// base_controller.cpp

// I N C L U D E S -------------------------------------------------------------------
#include "base_controller.hpp"

// converts a rad value to a degree value
double radToDeg(double angle) {
    return ((angle / pi_val) * 180.0);
}

// callback function for cmd_vel
void cmd_velCallback(geometry_msgs::Twist::ConstPtr cmd_velMsg , geometry_msgs::Twist* cmd_vel ){
  *cmd_vel = *cmd_velMsg;

}


// base_controller node
int main(int argc, char **argv)
{
  // initialize node handler
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;

  // declaration of cmd_vel and odom
  geometry_msgs::Twist cmd_vel, odom;

  // subscriber of cmd_vel and odom topic
  ros::Subscriber cmd_velSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, boost::bind(cmd_velCallback, _1, &cmd_vel));
  ros::Subscriber odomSub = nh.subscribe<geometry_msgs::Twist>("/odom", 10, boost::bind(cmd_velCallback, _1, &odom));

  // declaration of output msgs
  std_msgs::String msg;

  // declaration of the motor and steering data
  std_msgs::Int16 motor, steering;
  double steering_angle, motor_speed, radius, omega;



  // generate control message publisher
  ros::Publisher motorCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  // loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(30);
  while (ros::ok())
  { 

    // get steering values of the planner and convert them to degree
    double temp3 = cmd_vel.angular.z;
    double temp2 = radToDeg(temp3);

    // get motor data of the planner
    double temp = (double) cmd_vel.linear.x;

    // test output of the planner
    ROS_INFO("cmd_vel: %f",temp );

    /*
     *  old motor curve: this data was gained through simple time measurements
     *  0<temp<0.26 ->  (270*temp+130)
     *  0.26<temp<0.49 -> (434.78*(temp-0.26)+200)
     *  0.49<temp<0.77 -> (357.14*(temp-0.49)+300)
     *  0.79<temp<1.2 -> (555.55*(temp-0.77)+400)
     *  -5.0<temp<0.0 -> -200
     *
     *
     *  new motor curve: this data was gained through the analysis of the included csv files with matlab
     *
     *  f1(x) = 392,167*x+104,6712
     *  Racsac filtered regression for motor data >= 0
     *
     *  f2(x) = 489,445*x-103,18
     *  Racsac filtered regression for motor data < 0
    */

    if((0.005<temp)){
      motor.data= (int) (392.17*temp+104.67);
      ROS_INFO("case 1, cmd_vel > 0,05, move forward");
    }
    else if((-0.005>temp)){
      motor.data= (int) (489.45*temp-103.18);
      ROS_INFO("case 2, cmd_vel < 0, move backwards");
    }
    else {
      motor.data= (int) (0);
      ROS_INFO("case 3, cmd_vel = 0, don't move");
    }

    // publish motor data
    ROS_INFO("Motor Data as integer: %d", motor.data );
    motorCtrl.publish(motor);

    /*
     *  old steering curve: this data was gained through simple time measurements
     *  temp2>0 -> -40*temp2
     *  temp2<0 -> -52*temp2
     *
     *  new steering curve: this data was gained through the analysis of the included csv files with matlab
     *
     *  s1(temp2) = -40.41*temp2-140.3
     *  Racsac filtered regression for steering angle >= 0
     *
     *  s2(temp2) = -63.75*temp2-178.23
     *  Racsac filtered regression for steering angle < 0
    */

    ROS_INFO("steering data in deg: %f",temp2 );
    if((0.0<temp2)){
      steering.data = (int) (-40.41*temp2-140.3);
      ROS_INFO("steering case 1, angle>");
    }
    else if((temp2<0.0)) {
      ROS_INFO("steering case 2");
      steering.data = (int) (-63.75*temp2-178.23);
    }
    else{
      steering.data = 0;
      ROS_INFO("error steering Data");
    }
    // publish steering data
    steeringCtrl.publish(steering);
    ROS_INFO("steering Data as integer: %d", steering.data );


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

