#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <pcl/common/centroid.h>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace pcl;

class pcl_solution_class
{
	private:
					ros::NodeHandle nh;
					ros::Publisher pub;
					ros::Subscriber sub;
	public:
					void pubSubFunc();
					void pcl_Callback(const sensor_msgs::PointCloud2& msg);
}; 

//make the publisher global initially
void pcl_solution_class::pubSubFunc()
{

//publishing and subsribing
	pub = nh.advertise<vision_msgs::Detection3DArray>("/pcl_obstacle_detector_node/detections", 1);
	sub = nh.subscribe("/point_cloud", 1000, &pcl_solution_class::pcl_Callback, this);
	
}	



void pcl_solution_class::pcl_Callback(const sensor_msgs::PointCloud2& msg)
{		

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(msg, *cloud);


	//create filtering object

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.1f, 0.1f, 0.1f);
	vg.filter (*cloud_filtered);
	

	//create segmentation object; set parameters;

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.3);

        // Ground Plane Removal

	int i =0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			ROS_ERROR_STREAM ("could not estimate a planar model for the given dataset.");
		}
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		extract.filter (*cloud_plane);
	
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	}
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

// Specifying Cluster extraction specifications

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.5);
	ec.setMinClusterSize (10);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);


	vision_msgs::Detection3DArray darray; 
	vision_msgs::Detection3D dobj;
	darray.header = msg.header;
	
	Eigen::Vector4f clusterCentroid, min, max;

// Computing point cloud cluster

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    	{
    		cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    	}
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			pcl::compute3DCentroid(*cloud_cluster, clusterCentroid);
			pcl::getMinMax3D(*cloud_cluster, min, max);
			
			
			dobj.header = msg.header;
			dobj.bbox.center.position.x = clusterCentroid[0];		
			dobj.bbox.center.position.y = clusterCentroid[1];
			dobj.bbox.center.position.z = clusterCentroid[2];
			
			dobj.bbox.size.x = max[0] - min[0];
			dobj.bbox.size.y = max[1] - min[1];
			dobj.bbox.size.z = max[2] - min[2];

			darray.detections.push_back(dobj);
  	}
  
  	pub.publish(darray);	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_obstacle_detector");
	
	pcl_solution_class pcl_obj;
	pcl_obj.pubSubFunc(); // Initializing publisher and subscriber functions
	
	ros::spin();
	
	return 0;
}
