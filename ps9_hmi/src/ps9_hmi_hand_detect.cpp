//implementation of hmi class from library
//part of ps9_hmi package
//written by Matthew Dobrowsky

#include <ps9_hmi/ps9_hmi_hand_detect.h>
#include <math.h>

//library class constructor
HumanMachineInterface::HumanMachineInterface(ros::NodeHandle* nh) : cwru_pcl_utils(nh) {

	hand_present_ = false;

	//with base as target frame
	//great focal point for hand placement:
	//(with arm going away and up to person)
	//x = 0.594788 (go with 0.7)
	//y = -0.0239214 (go with 0.0)
	//z = 0.1742

	//values for detection area
	detect_radius_ = 0.25;
	detection_centroid_[0] = 0.7;
	detection_centroid_[1] = 0.0;
	detection_centroid_[2] = 0.1742;
	
	//detected height of the hand
	hand_height_ = 10.0;

	//required ratio of present points to consider the hand present
	threshold_detection_ratio_ = 0.2;

	//ratio of blocked hand points
	blocked_ratio_ = 0.0;

	//persistent transform listener
	tf::TransformListener tf_listener_;
}

//private function: polls kinect for a current snapshot 
void HumanMachineInterface::get_kinect_snapshot_() {
	cwru_pcl_utils.reset_got_kinect_cloud();
	//timeout counter for 10 seconds to wait on kinect
	int timeout = 0;
	ROS_INFO("HMI is getting a new kinect point cloud.");
	while(!cwru_pcl_utils.got_kinect_cloud() && timeout < 10) {
		ROS_INFO("HMI is waiting on kinect point cloud.");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
		timeout++;
	}
	
	//timeout error, no kinect cloud found still
	if(!cwru_pcl_utils.got_kinect_cloud()) {
		ROS_ERROR("A kinect cloud request in HMI timed out. Please try again.");
		return;
	}

	ROS_INFO("HMI is now processing a kinect point cloud.");

	//transform objects to move kinect pcl to base frame
	tf::StampedTransform tf_kinect_to_base;

	bool tferr = true;
    //ROS_INFO("HMI is calculating a transform between the kinect and the base.");
    while (tferr) {
        tferr = false;
    	try {
            //look for a transform from kinect to the base
            tf_listener_.lookupTransform("base", "camera_rgb_optical_frame", ros::Time(0), tf_kinect_to_base);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); //sleep for half a second
            ros::spinOnce();
        }
    }
	//ROS_INFO("HMI is executing a transform on the point cloud data.");

	//get eigen values for the kinect point cloud transform
	Eigen::Affine3f eigen_kinect_to_base;
	eigen_kinect_to_base = cwru_pcl_utils.transformTFToEigen(tf_kinect_to_base);

	cwru_pcl_utils.transform_kinect_cloud(eigen_kinect_to_base);
	ROS_INFO("HMI has updated and processed a current kinect point cloud.");
}

//private utility function converts a point cloud XYZRGB to only XYZ
//allows full use of the cwru_pcl_utils library
void HumanMachineInterface::convert_rgb_to_xyz_(PointCloud<pcl::PointXYZRGB> inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
	//transfer over input cloud values to output
    outputCloud->header = inputCloud.header;
    outputCloud->is_dense = inputCloud.is_dense;
    outputCloud->width = inputCloud.width;
    outputCloud->height = inputCloud.height;

	//size cloud and transfer all input cloud points
    int npts = inputCloud.points.size();
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud.points[i].getVector3fMap();
    }
}

//private function: prunes list of indices selecting those within a certain x,y planar range
int HumanMachineInterface::count_points_inside_range_(PointCloud<pcl::PointXYZRGB> pcl, double range) {
	double x;
	double y;
	double dist;
	int count = 0;
	int npts = pcl.points.size();
	for(int i = 0; i < npts; ++i) {
		x = pcl.points[i].getVector3fMap()[0];
		y = pcl.points[i].getVector3fMap()[1];
		dist = sqrt(pow(x,2) + pow(y,2));

		if(dist <= range) {
			count++;
		}
	}
	ROS_INFO("%d points were out of range.", npts-count);
	return count;
}

//ORGANIZE, TEST BOUNDARIES
//public function: human interaction checking
bool HumanMachineInterface::get_human_hand() {
	/* variable declarations */

	double hand_point_count = 0.0; //number of pcl points that were counted as "hand" in detection zone
	double detected_area_point_count = 0.0; //total number of pcl points in detection zone

	Eigen::Vector3f hand_centroid;
	hand_centroid[0] = 0;
	hand_centroid[1] = 0;
	hand_centroid[2] = 0;

	//point clouds of points that are hand height
	PointCloud<pcl::PointXYZRGB> rgb_height_cloud;
	rgb_height_cloud.clear();
	PointCloud<pcl::PointXYZ> height_cloud;
	height_cloud.clear();
	PointCloud<pcl::PointXYZ>::Ptr height_cloud_ptr = height_cloud.makeShared();

	//point cloud index arrays
	vector<int> valid_height_index(0); //all points of valid "hand" height
	vector<int> detected_index(0); //all points in detection zone
	vector<int> hand_index(0); //valid height points in detection zone - "hand" points

	/* begin function procedure */

	//get newest kinect point cloud
	get_kinect_snapshot_();

	//check if kinect update failed
	if(!cwru_pcl_utils.got_kinect_cloud()) {
		ROS_ERROR("HMI failed to update the kinect cloud. Cannot detect a human hand.");
		return false;
	}

	//gather all points of valid hand heights
	//store in valid_height_index
	valid_height_index.clear();
	cwru_pcl_utils.find_coplanar_pts_z_height(HeightRough, HeightSpread, valid_height_index);

	//debug stmt
	ROS_INFO("HMI found %lu points that are at a valid height.", valid_height_index.size());

	cwru_pcl_utils.copy_indexed_pts_to_output_cloud(valid_height_index, rgb_height_cloud);
	
	//check that there is a reasonable number of points to range-find
	if(valid_height_index.size() >= 2000) {
		int pcl_hand_count = count_points_inside_range_(rgb_height_cloud, InterfaceRange);
		if( (double) pcl_hand_count / valid_height_index.size() > 0.3) {
			return true;
		}
		return false;
	}

	else {
		ROS_WARN("HMI did not find enough valid points to consider a hand present.");
		return false;
	}
	/*
	//find a rough centroid of the hand-object at height
	cwru_pcl_utils.copy_indexed_pts_to_output_cloud(valid_height_index, rgb_height_cloud);
	
	//debug stmt
	ROS_INFO("copied %lu color points", rgb_height_cloud.points.size());

	convert_rgb_to_xyz_(rgb_height_cloud, height_cloud_ptr);

	//debug stmt
	std::cout << "number of points in xyz from rgbxyz:  " << height_cloud_ptr->points.size() << endl;

	hand_centroid = cwru_pcl_utils.compute_centroid(height_cloud_ptr);
	
	//debug stmt
	ROS_INFO("HMI calculated a centroid for hand with location %f, %f, %f relative to base.", hand_centroid[0], hand_centroid[1], hand_centroid[2]);

	//set hand height as centroid's z
	hand_height_ = hand_centroid[2];

	hand_index.clear();
	cwru_pcl_utils.find_coplanar_pts_z_height(hand_height_, HandErr, hand_index);

	ROS_INFO("Found %lu points at the hand height within acceptable error.", hand_index.size());
	*/
	//select all points at the hand height that fall within the detection zone
	//height is the height of computed hand centroid
	//radius and error are predetermined
	//stored in hand_index
	//hand_index.clear();
	//cwru_pcl_utils.filter_cloud_z(hand_height_, HandErr, detect_radius_, hand_centroid, hand_index);

	//select all points in the detection radius
	//stored in detected_index
	//detected_index.clear();
	//cwru_pcl_utils.filter_cloud_z(AllHeight, AllSpread, detect_radius_, hand_centroid, detected_index);

	//get count values from index sizes
	/*hand_point_count = (double) hand_index.size();
	detected_area_point_count = (double) detected_index.size();
	
	//checks the ratio 
	blocked_ratio_ = hand_point_count / detected_area_point_count;
	if(blocked_ratio_ >= threshold_detection_ratio_) {
		hand_present_ = true;
	}
	else {
		hand_present_ = false;
	}*/

	return hand_present_;
}


