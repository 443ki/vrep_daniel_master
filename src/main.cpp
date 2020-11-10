#include <ros/ros.h>
#include "vrep_daniel_master.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vrep_daniel_master_node");
  VrepDanielMaster vrep_daniel_master;
  ROS_INFO("VrepDanielMaster was initialized!");

  ros::spin();
  return 0;
}
