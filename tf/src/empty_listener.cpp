#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try 
    {
      listener.lookupTransform("odom_combined", "base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    rate.sleep();
  }
  return 0;
};
