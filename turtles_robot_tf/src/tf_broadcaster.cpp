/* tf_broadcaster.cpp
 * This is the tf_broadcaster node which creates transformation by using the tf package.
 * The actual transformations are used by amcl.
 *
 *
 * This code is based on the ros tutorial: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
 * @author: TUrtlES (tud.turtles@gmail.com)
 * @version: 1.0
 */

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

  // initialization of the node
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

    //transformation for camera_depth_image
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.105, -0.05, 0.25)),
        ros::Time::now(),"base_link", "camera_depth_frame"));
    //transformation for kinect optical frame
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.105, -0.10, 0.25)),
        ros::Time::now(),"base_link", "kinect2_rgb_optical_frame"));

/*
    //transformation for ultrasonic senor right
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, -0.09, 0.05)),
        ros::Time::now(),"base_link", "right_sensor"));
    //transformation for ultrasonic sensor left
    broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.105, 0.09, 0.05)),
      ros::Time::now(),"base_link", "left_sensor"));
    //transformation for ultrasonic sensor front
    broadcaster.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.105, -0.05, 0.05)),
      ros::Time::now(),"base_link", "front_sensor"));
*/
    r.sleep();
  }
}
