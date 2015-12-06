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
#define HandErr 0.01

//flex values for hand-height detection
#define HeightRough 0.2
#define HeightErr 0.05

//values for finding all cloud points
#define GeneralHeight 0.2
#define GeneralSpread 1.0

class HumanMachineInterface {
public:
	//constructor
	HumanMachineInterface(ros::NodeHandle* nh);

   	 ~HumanMachineInterface(void) {  }
	
	//function for returning hand detection state
	bool get_human_hand();

	//getter methods for private values
	double get_hand_height() { return hand_height_; };
	double get_blocked_ratio() { return blocked_ratio_; };

	CwruPclUtils cwru_pcl_utils;

private:
	//Kinect usage
	void get_kinect_snapshot_();
	void convert_rgb_to_xyz_(PointCloud<pcl::PointXYZRGB> inputCloud, PointCloud<pcl::PointXYZ> outputCloud);

    //measured hand values
	bool hand_present_;
	double hand_height_;
	double blocked_ratio_;
	Eigen::Vector3f hand_centroid_;

    //values for hand detection criteria
	double detect_radius_;
	double threshold_detection_ratio_;
};

#endif