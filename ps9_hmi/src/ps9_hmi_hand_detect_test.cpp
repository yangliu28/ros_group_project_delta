//test file for hmi ps9 final group project
//part of ps9_hmi
//written by Matthew Dobrowsky

#include <ros/ros.h>
#include <ps9_hmi/ps9_hmi_hand_detect.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

int main(int argc, char** argv) {
	ros::init(argc, argv, "ps9_hmi_hand_detect_test");
	ros::NodeHandle nh;

	//create an hmi class object
	HumanMachineInterface hmi(&nh);

	ROS_INFO("HMI testing is about to start.");

	while(ros::ok()) {
		if(hmi.get_human_hand()) {
			//exits test program once a hand is found
			std::cout << "There is a human hand present." << endl;
			return 0;
		} else {
			//will continue to execute test if no hand is found
			std::cout << "There is no hand detected." << endl;
		}
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	return 0;
}
