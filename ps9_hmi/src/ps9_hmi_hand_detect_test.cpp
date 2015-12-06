//test file for hmi ps9 final group project
//
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

	while(ros::ok()) {
		if(hmi.get_human_hand()) {
			std::cout << "There is a human hand present." << endl;
			std::cout << "The height detected is " << hmi.get_hand_height() << endl;
			std::cout << "The ratio of blocked points is " << hmi.get_blocked_ratio() << endl;
		} else {
			std::cout << "There is no hand detected." << endl;
			std::cout << "The detection height is " << hmi.get_hand_height() << endl;
			std::cout << "The ratio of blocked points is " << hmi.get_blocked_ratio() << endl;
		}
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	return 0;

	/**
	//create a "hand" point cloud to simulate hand placement

	//have values of get_human_hand() checked when "hand" is present, and not
	while(ros::ok()) {
		//initial check to see that hand is not there
		if(hmi.get_human_hand()) {
			ROS_ERROR("HMI test has found a hand at startup, there should not be one.");
		}

		//activate hand presence

		//check that hand is detected
		if(!hmi.get_human_hand()) {
			ROS_ERROR("HMI test has not found the simulated hand point cloud.");
		}

		//remove presence

		//check that hand is gone
		if(hmi.get_human_hand()) {
			ROS_ERROR("HMI test is still detecting a hand after it has been removed.");
		}
	}
	return 0;
	**/
}
