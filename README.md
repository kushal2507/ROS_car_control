## Final Assignment - Autonomous driving

The final assignment involves developing software for autonomous control of a Prius in a simulated test track. The entire control assignment is divided into three tasks :

1) Detecting a line using OpenCv
2) Object detection using PCL and
3) Following the drawn line and stopping the vehicle when a human is detected and never drive again.

## Starting Singularity

It is assumed that singularity is already setup on the user's PC. 

To start using the singularity, run following command,
```
singularity shell --nv /opt/apps/singularity/2.4.5/data/robprac-singularity2019-v0.img


``` 

## To clone the repository follow the given steps:
Open a terminal in home directory i.e. `~/` location. Check that you dont have any directory named `catkin_ws` in home directory. Run following commands one after another.
```
source /opt/ros/kinetic/setup.sh
cd ~
mkdir catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone git@gitlab.me41025.3me.tudelft.nl:students-1819/lab4/group11.git
git clone git@gitlab.me41025.3me.tudelft.nl:students-1819/me41025_simulator.git
cd ..
catkin_make
source devel/setup.sh
roslaunch control_line_world solution.launch
```

## Task 1 : Line Detection using OpenCV

This package performs three tasks, 

1) Subscribes to an image topic /prius/front_camera/image_raw
2) Processes the received image to detect the line
3) Publishes the detection of the line on a ROS topic

A number of steps had to be performed in order to detect the line and publish the detected image on the ros topic

The image input from the prius camera was first converted into a MAT image type (array), for easier and convenient processing.
The command below stores the image input from the camera into a mat_image variable.

cv::Mat mat_image = cv_ptr->image


The input image was then cropped and thresholding was applied onto the image in order to detect the line based on pixel intensity variation.

The command below takes the mat image as input and stores a new image in thresh after undergoing thresholding with the specified parameters.
cv::threshold(mat_image_new,thresh,50,255,cv::THRESH_BINARY_INV)


The image post thresholding underwent a morphological transformation in order to remove small open spaces between the line drawn in the image. This would further help with processing.

The command below takes the image post thresholding as input and stores it in morph_img as output after applying the morphological transformations,
cv::morphologyEx(thresh,morph_img, cv::MORPH_OPEN, kernel)



Post a morphological transformation, the image was converted into a black and white image and all the non- zero pixel locations were identified, these would correspond to the pixel points which consisted of the line.

The command below takes the morph_img as input and stores it as a black and white mat image as output. 
cv::cvtColor(morph_img,bw_image,CV_RGB2GRAY)


Once the line in the image was succesfully identified a green rectangle had to be drawn around the line as detected in front of the car.

The command below is used to draw a rectangle having opposite corner points, p1 and p2. The green rectangle is drawn onto the image named mat_image_copy
cv::rectangle(mat_image_copy, p1, p2, cv::Scalar(0,255,0),2)



This green rectangle would successfully folow the line and vary its dimensions according to the changing curvature of the line.
Finally the rectangle along with image is published as a 2D detection and can be seen on the openCV publishing screen on Rviz.


## Task 2 : Obstacle Detection using PCL

This package:

1) subscribes to the point cloud topic /point_cloud 
2) Processes the recieved point cloud to final cluster of points and
3) Publishes the cluster as a Detection array on /pcl_obstacle_detector_node/detections

After subscribing to the desired topic PCL is used to remove the grond plane because only objects that are not part of the ground are of interest.

Next, Euclidean cluster extraction is used to detect clusters using the spcified parameters:

* Tolerance = 0.5

* Minimum Cluster size = 10

* Maximum Cluster Size = 25000

Finally, before publishing, the point cloud clusters are converted to bounding boxes. The center of the cluster and the extremes of the cluster are computed to fill the size of bounding box.

The detected box is published as vision_msgs/Detection3DArray on the topic /pcl_obstacle_detector_node/detections
 
## Task 3 : Prius Control


The control package is responsible for controlling the Prius. The basic objective of this package is to make sure the car follows the line and 
whenever a person is detected, it stops and nevers moves again.

The control algorithm consists of two parts, detecting the line while following it and detecting the point cloud cluster of the human and stopping when detected.

The line is detected through the green rectangle which was created in the OpenCV package. Whenever the position of the center coordinates of the rectangle change, the car steers accordingly.
Through the specified parameters and coordinates of the center, the car successfully follows the line and is able to steer according to the changing directions of the line.

The Point cloud cluster of the human is detected through the pcl package. Whenever the specified dimensions of the pcl detections are met the car successfully stops by increasing the brake parameters, reducing the throttle to zero and changing the gear to neutral. The above steps ensure that the car doesnt move again.

## Work Distribution

OPENCV - Kushal Thirani 4805569

PCL - Saket Sarawgi 4809254

Control Package - Kushal and Saket
 


