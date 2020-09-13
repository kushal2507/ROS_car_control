#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <prius_msgs/Control.h>
#include <iomanip>
#include <string.h>
#include<vision_msgs/Detection2D.h>
#include<vision_msgs/Detection3DArray.h>
#include <cmath>



class SubAPub
{


public:
  void SubscribeAndPublish();

  void control_opencv_callback(const vision_msgs::Detection2D& msg_opencv);
  void control_pcl_callback(const vision_msgs::Detection3DArray& msg_pcl);

  //const std::string THROTTLE = "throttle_param";
  //const std::string STEER_LEFT = "steer_param";
 // const std::string STEER_RIGHT = "rsteer_param";


private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub1;
  ros::Subscriber sub2;

};
