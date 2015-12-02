#include <ps9_pcl/block_detection.h>

#include <ros/ros.h>  //  for ros
#include <math.h>

#include <Eigen/Eigen>  //  for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "block_detection_test_main");
    ros::NodeHandle nh;
    Block_detection cwru_pcl_utils(&nh);
    //ROS_INFO("Instantiation done.");
	

    while(ros::ok())
    {
        if (cwru_pcl_utils.find_stool())
        {
            cwru_pcl_utils.find_block();
        }


        ros::Duration(0.5).sleep();  // sleep for half a second
        ros::spinOnce();
    }
	
}

