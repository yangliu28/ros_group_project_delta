//implementation of hmi class from library
//part of ps9_hmi package
//written by Matthew Dobrowsky

#include <ps9_hmi/ps9_hmi_hand_detect.h>

//library class constructor
HumanMachineInterface::HumanMachineInterface(ros::NodeHandle* nh) : cwru_pcl_utils(nh) {
	
	hand_present_ = false;

	//radius for detection area
	detect_radius_ = 0.075;
	
	//detected height of the hand
	hand_height_ = 10.0;

	//required ratio of present points to consider the hand "blocking"
	threshold_detection_ratio_ = 0.3;

	//kinect usage variables
}

//private function: polls kinect for current data 
void HumanMachineInterface::get_kinect_snapshot_() {
	cwru_pcl_utils.reset_got_kinect_cloud();
	ROS_INFO("HMI is getting a new kinect point cloud.");
	while(!cwru_pcl_utils.got_kinect_cloud()) {
		ROS_INFO("HMI is waiting on kinect point cloud.");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("HMI is now processing a kinect point cloud.");

	//transform objects to move kinect pcl to floor frame
	tf::StampedTransform tf_kinect_to_floor;
	tf:TransformListener tf_listener;

	bool tferr = true;
    ROS_INFO("HMI is calculating a transform between the kinect and the floor.");
    while (tferr) {
        tferr = false;
        try {
            //look for a transform from kinect to the FLOOR
            tf_listener.lookupTransform("world", "kinect_pc_frame", ros::Time(0), tf_kinect_to_floor);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); //sleep for half a second
            ros::spinOnce();
        }
    }
	ROS_INFO("HMI is executing a transform on the point cloud data.");

	//get eigen values for the transform
	Eigen::Affine3f eigen_kinect_to_floor;
	eigen_kinect_to_floor = cwru_pcl_utils.transformTFToEigen(tf_kinect_to_floor);

	cwru_pcl_utils.transform_kinect_cloud(eigen_kinect_to_floor);
	ROS_INFO("HMI has an updated and transformed kinect point cloud.");
}

//public function: human interaction checking
bool HumanMachineInterface::get_human_hand() {
	get_kinect_snapshot_();

	//number of pcl points that were counted as "hand" in detection box
	int hand_point_count;
	//total number of pcl points in detection box
	int detected_area_point_count;
	//fraction of points that are "hand"s in detection box
	double blocked_ratio;

	//for points that are hand height
	vector<int> valid_height_index;
	PointCloud<pcl::PointXYZ> height_cloud;

	//list of indices for all points in detection radius
	vector<int> detected_index;
	//list of indices for points that are considered "hand"
	vector<int> hand_index;

	//gather all points of approximately hand height
	//stored in valid_height_index
	valid_height_index.clear();
	cwru_pcl_utils.find_coplanar_pts_z_height(HeightRough, HeightErr, valid_height_index);
	//find a rough centroid of the hand
	cwru_pcl_utils.copy_indexed_pts_to_output_cloud(valid_height_index, height_cloud);
	hand_centroid_ = cwru_pcl_utils.compute_centroid(height_cloud);
	
	//use centroid to select a more precise set of points as hand
	//stored in hand_index
	cwru_pcl_utils.filter_cloud_z(HeightRough, HandErr, detect_radius_, hand_centroid_, hand_index);

	//select all points in the detection radius
	//stored in detected_index
	cwru_pcl_utils.filter_cloud_z(GeneralHeight, GeneralSpread, detect_radius_, hand_centroid_, detected_index);

	//get count values from index sizes
	hand_point_count = hand_index.size();
	detected_area_point_count = detected_index.size();

	//filter for centroid and height for best points, sum = size of indices

	/**iterate over all points while checking and doing:
		-Store points that are of the correct height (in hand area?)
		-count total points in hand area
	**/

	//then do either A or B
	//A
	//find center of hand
	//sum all points in box that are within flex height of center

	//B
	//sum points in hand area with hand height
	
	
	//checks the ratio 
	blocked_ratio = (double) valid_point_count / (double) total_area_point_count;
	if(blocked_ratio >= threshold_detection_ratio_) {
		hand_present_ = true;
	}
	else {
		hand_present_ = false;
	}

	return hand_present_;
}


