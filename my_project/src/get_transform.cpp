#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
//#include <tf_msgs/TFMessage>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_transform");
  
  ros::NodeHandle node;
  tf::TransformListener listener; 
  
  listener.waitForTransform("camera_link", "laser_scanner_link", ros::Time(0), ros::Duration(5.0));

  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform("camera_link", "laser_scanner_link", ros::Time(0), transform);
    printf("Got the transform!\n");
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception caught %s",ex.what());
    ros::Duration(1.0).sleep();
  }

  return 0;
};