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

//flex values for hand-height detection
#define HeightRough 0.1742
#define HeightSpread 0.1

//range for hand detection from baxter
#define InterfaceRange 0.35

class HumanMachineInterface {
public:
	//constructor
	HumanMachineInterface(ros::NodeHandle* nh);

   	~HumanMachineInterface(void) {  }
	
	//function for returning hand detection state
	bool get_human_hand();

	CwruPclUtils cwru_pcl_utils;

private:
	//Kinect usage
	tf::TransformListener tf_listener_;
	void get_kinect_snapshot_();
	int count_points_inside_range_(PointCloud<pcl::PointXYZRGB> pcl, double range);

	double threshold_detection_ratio_;
};

#endif
