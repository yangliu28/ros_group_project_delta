#include <ps9_pcl/block_detection.h>

#include <math.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

Eigen::Vector3f red(230,60,60);
geometry_msgs::Pose pose;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "block_detection_test_main");
    ros::NodeHandle nh;
    Block_detection cwru_pcl_utils(&nh);

    ROS_INFO("I'm ready!");
	

    while(ros::ok())
    {
        if (cwru_pcl_utils.find_stool())
        {
            cwru_pcl_utils.find_block();
            pose = cwru_pcl_utils.find_pose();
            // ROS_INFO(pose);
        }
        // cwru_pcl_utils.find_stool();
        // cwru_pcl_utils.find_block();
        // cwru_pcl_utils.find_hand();

        // cwru_pcl_utils.find_block_by_color(red);

        ros::Duration(1).sleep();
        ros::spinOnce();
    }
	
}

