#include <ps9_pcl/block_detection.h>

#include <math.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "block_detection_test_main");
    ros::NodeHandle nh;
    Block_detection cwru_pcl_utils(&nh);

    ROS_INFO("I'm ready!");
	

    while(ros::ok())
    {
        // if (cwru_pcl_utils.find_stool())
        // {
        //     cwru_pcl_utils.find_block();
        // }
        // cwru_pcl_utils.find_stool();
        // cwru_pcl_utils.find_block();
        cwru_pcl_utils.find_hand();
        
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
	
}

