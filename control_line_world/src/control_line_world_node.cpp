#include <ros/ros.h>
#include "control_line.h"



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "control_line_world"); //defalut name of node.
  
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubAPub SAPObject;
  SAPObject.SubscribeAndPublish();
  

  ros::spin();

  return 0;
}
