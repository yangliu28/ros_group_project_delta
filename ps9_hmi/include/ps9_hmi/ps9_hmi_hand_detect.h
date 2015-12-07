//library for human machine interface
//part of ps9_hmi package
//written by Matthew Dobrowsky

#ifndef PS9_HMI_HAND_DETECT_H_
#define PS9_HMI_HAND_DETECT_H_

#include <ros/ros.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

//the minimum height for a point to be a "hand"
#define HandMinHeight 0.1
//flex value for hand curvature / angling
#define HandErr 0.2

//flex values for hand-height object detection
#define HeightRough 0.1742
#define HeightSpread 0.1

//values for finding all cloud points in detection zone
#define AllHeight 0.0
#define AllSpread 100.0

//range for hand detection from baxter
#define InterfaceRange 0.35

class HumanMachineInterface {
public:
	//constructor
	HumanMachineInterface(ros::NodeHandle* nh);

   	 ~HumanMachineInterface(void) {  }
	
	//function for returning hand detection state
	bool get_human_hand();

	//public getter methods for private values
	double get_hand_height() { return hand_height_; };
	double get_blocked_ratio() { return blocked_ratio_; };

	CwruPclUtils cwru_pcl_utils;

private:
	//Kinect usage
	tf::TransformListener tf_listener_;
	void get_kinect_snapshot_();
	void convert_rgb_to_xyz_(PointCloud<pcl::PointXYZRGB> inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
	int count_points_inside_range_(PointCloud<pcl::PointXYZRGB> pcl, double range);
	

    //measured hand values
	bool hand_present_;
	double hand_height_;
	double blocked_ratio_;
	Eigen::Vector3f detection_centroid_;

    //values for hand detection criteria
	double detect_radius_;
	double threshold_detection_ratio_;
};

#endif
