#include <ros/ros.h>
#include "control_line.h"



 void SubAPub::SubscribeAndPublish()
  {
    //Topic you want to publish
    pub = nh.advertise<prius_msgs::Control>("prius", 1000);

    //Topics you want to subscribe
    sub1 = nh.subscribe("opencv_line_detector_node/detections", 1000, &SubAPub::control_opencv_callback, this);
    sub2 = nh.subscribe("pcl_obstacle_detector_node/detections", 1000, &SubAPub::control_pcl_callback, this); 
  }


 void SubAPub::control_opencv_callback(const vision_msgs::Detection2D& msg_opencv)
  { 
   prius_msgs::Control cv_msg_out;
   cv_msg_out.shift_gears = cv_msg_out.FORWARD;
	
	//for (unsigned int i = 0; i < msg_opencv.detections.size(); i++)
	//{
	     //If bbox size is (0,0), continue driving with zero steer		
             if (msg_opencv.bbox.size_x == 0 && msg_opencv.bbox.size_y == 0)
		{
		   cv_msg_out.steer = 0;
		   cv_msg_out.throttle = 0.5;
			 cv_msg_out.brake = 0;
		}

		//If bbox.center.x < 320, steer left(1)
		if(msg_opencv.bbox.center.x < 320)
		{
		   cv_msg_out.steer = 1;
		   cv_msg_out.throttle = 0.5;
		   cv_msg_out.brake = 0;
		}

		//If bbox.center.x > 480, steer to right(-1)
		if(msg_opencv.bbox.center.x > 480)
		{
		   cv_msg_out.steer = -1;
		   cv_msg_out.throttle = 0.5;
		   cv_msg_out.brake = 0;
		}

		//If 320 <= bbox.center.x <= 480, zero steer and drive forward
		if(msg_opencv.bbox.center.x >= 320 && msg_opencv.bbox.center.x <= 480 )
		{
		   cv_msg_out.steer = 0;
		   cv_msg_out.throttle = 0.5;
		   cv_msg_out.brake = 0;
		}

	pub.publish(cv_msg_out);
  }
  
  
  
 // Filter 3D obstacle and if x>0, brake 
 void SubAPub::control_pcl_callback(const vision_msgs::Detection3DArray& msg_pcl)
 {
	prius_msgs::Control pcl_msg_out;
	prius_msgs::Control cv_msg_out;
	
	//double turn, throttle;
	//float distance, min_distance(6);
	//int obj_counter(0),dist_min_idx;
	
	for (int i = 0; i < msg_pcl.detections.size(); i++)
	{
		if (msg_pcl.detections[i].bbox.center.position.x > 0)
		{
			pcl_msg_out.brake=1;
			pcl_msg_out.throttle =0;
			pcl_msg_out.steer =0;
//cv_msg_out.brake=1;
			//cv_msg_out.throttle =0;
			//cv_msg_out.steer =0;

			pcl_msg_out.shift_gears = 1;//pcl_msg_out.NEUTRAL;
			
		}
		else{
			pcl_msg_out.throttle =0.5;
		     }

	}
	
	//pub.publish(cv_msg_out);
	
	//pcl_msg_out.shift_gears = pcl_msg_out.FORWARD;
	
	//publish the command
	pub.publish(pcl_msg_out);

}
