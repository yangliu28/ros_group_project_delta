//implementation of hmi class from library
//part of ps9_hmi package
//written by Matthew Dobrowsky

#include <ps9_hmi/ps9_hmi_hand_detect.h>
#include <math.h>

//library class constructor
HumanMachineInterface::HumanMachineInterface(ros::NodeHandle* nh) : cwru_pcl_utils(nh) {

	//required ratio of present points to consider the hand present
	threshold_detection_ratio_ = 0.2;

	//persistent transform listener
	tf::TransformListener tf_listener_;
}

//private function: uses cwru_pcl_utils to poll kinect for a current snapshot 
void HumanMachineInterface::get_kinect_snapshot_() {
	cwru_pcl_utils.reset_got_kinect_cloud();
	//timeout counter for 10 seconds to wait on kinect
	int timeout = 0;
	// ROS_INFO("HMI is getting a new kinect point cloud.");
	while(!cwru_pcl_utils.got_kinect_cloud() && timeout < 10) {
		// ROS_INFO("HMI is waiting on kinect point cloud.");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
		timeout++;
	}
	
	//timeout error, no kinect cloud found still
	if(!cwru_pcl_utils.got_kinect_cloud()) {
		ROS_ERROR("A kinect cloud request in HMI timed out. Please try again.");
		return;
	}

	// ROS_INFO("HMI is now processing a kinect point cloud.");

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

	//get eigen values for the kinect point cloud transform
	Eigen::Affine3f eigen_kinect_to_base;
	eigen_kinect_to_base = cwru_pcl_utils.transformTFToEigen(tf_kinect_to_base);

	cwru_pcl_utils.transform_kinect_cloud(eigen_kinect_to_base);
	// ROS_INFO("HMI has updated and processed a current kinect point cloud.");
}

//private function: prunes list of indices selecting those within a certain x,y planar range
//allows triangulation of pcl points by reduction, removing those below a height and others outside range
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
	ROS_INFO("%d points were out of range, out of %d total.", npts-count, npts);
	return count;
}

//public function: human interaction checking
bool HumanMachineInterface::get_human_hand() {
	/* variable declarations */

	double hand_point_count = 0.0; //number of pcl points that were counted as "hand" in detection zone
	double detected_area_point_count = 0.0; //total number of pcl points in detection zone

	//point clouds of points that are hand height
	PointCloud<pcl::PointXYZRGB> rgb_height_cloud;
	rgb_height_cloud.clear();

	//point cloud index arrays
	vector<int> valid_height_index(0); //all points of valid detection heights

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
	ROS_INFO("HMI found %lu points that are at a valid height.", valid_height_index.size());

	//get detected points
	cwru_pcl_utils.copy_indexed_pts_to_output_cloud(valid_height_index, rgb_height_cloud);
	
	//check that there is a reasonable number of points to range-find
	if(valid_height_index.size() >= 2000) {
		int pcl_hand_count = count_points_inside_range_(rgb_height_cloud, InterfaceRange);
		double ranged_ratio = (double) pcl_hand_count /valid_height_index.size();
		if( ranged_ratio >= threshold_detection_ratio_) {
			return true;
		}
		//ROS_WARN("HMI found enough points for the hand, but they were out of range.");
		return false;
	}
	else {
		//ROS_WARN("HMI did not find enough valid points to consider a hand present.");
		return false;
	}
}


