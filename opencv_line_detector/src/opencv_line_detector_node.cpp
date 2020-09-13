#include <ros/ros.h>
#include<stdlib.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include<std_msgs/String.h>
#include <iomanip>
#include <stdio.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include<vision_msgs/BoundingBox2D.h>


// Global declaration

image_transport::Publisher image_pub;
ros::Publisher pub;

void opencv_callback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)

{

		ros::init(argc, argv, "opencv_line_detector");
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);
		image_transport::Subscriber image_sub;
	
		vision_msgs::Detection2DArray d2array;
		vision_msgs::Detection2D d2obj;


		image_sub = it.subscribe("/prius/front_camera/image_raw", 1000,opencv_callback);
		image_pub = it.advertise("/opencv_line_detector_node/visual", 1000);
		pub = nh.advertise<vision_msgs::Detection2D>("/opencv_line_detector_node/detections", 1000);

		ros::spin();

}

     
    void opencv_callback(const sensor_msgs::ImageConstPtr& msg)
    {	
	
   	int x, y, width, height, image_width, image_height;
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

cv::Mat mat_image = cv_ptr->image;
image_width = mat_image.cols;
image_height = mat_image.rows;

x = 0;
y = image_height/2;
width = image_width;
height = image_height/5;

//Creating initial rectangle

cv::Rect mat_image_rect = cv::Rect(x, y, width, height);
cv::Mat mat_image_new = mat_image(mat_image_rect);
cv::Mat mat_image_copy = mat_image(mat_image_rect);


//Thresholding

cv::Mat thresh;
cv::threshold(mat_image_new,thresh,50,255,cv::THRESH_BINARY_INV);

//Morphing

cv::Mat morph_img;
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Point(5, 5));
cv::morphologyEx(thresh, morph_img, cv::MORPH_OPEN, kernel);

//Converting to bw
cv::Mat bw_image;
cv::cvtColor(morph_img, bw_image, cv::COLOR_RGB2GRAY);

//Finding non zero locations

cv::Mat non_zero_img;
cv::findNonZero(bw_image,non_zero_img);

//Bounding rectangle creation

cv::Rect bound_img = cv::boundingRect(non_zero_img);

//Finding coordinates of opposite points of rectangle

cv::Point p1(bound_img.x,(bound_img.y + image_height/2));
cv::Point p2(bound_img.x + bound_img.width,bound_img.y + image_height/2 + bound_img.height);
// Colouring rectangle green
cv::rectangle(mat_image, p1, p2, cv::Scalar(0,255,0),2);


// Converting back pointer
sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_copy).toImageMsg();

vision_msgs::Detection2DArray d2array;
vision_msgs::Detection2D d2obj;

// Publishing to detections 2D
          		
          	
          		d2obj.bbox.center.x = (bound_img.x + (bound_img.width/2));
          		d2obj.bbox.center.y = (bound_img.y +  image_height/2 + ( bound_img.height/2));
          		
          		d2obj.bbox.size_x = bound_img.width;
          		d2obj.bbox.size_y = bound_img.height;

			d2obj.header = (*msg).header;
          		
          		d2array.detections.push_back(d2obj);
	      	

      image_pub.publish(img_msg);	
      pub.publish(d2obj);

    } //end of callback


