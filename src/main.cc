#include <navigation_node.h>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  NavigationSystemNode navigation_node(nh, pnh);
  navigation_node.Run();

  ros::spin();
  return 0;
}
