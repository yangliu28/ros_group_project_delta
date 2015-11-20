// pseudo code for the work flow of this project

#include <ros/ros.h>
// eigen lilbraries to be used in position calculation of arm motion
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// for motion realization
#include <ps9_arm_motion/ps9_arm_motion_commander_lib.h>
// for gripper control
#include <ps9_gripper_control/ps9_gripper_controller_lib.h>
// for point cloud sensor
#include <ps9_pcl_hmi/ps9_pcl_hmi_lib.h>
// add more includes to define your variables

int main(int argc, char** argv) {
    ros::init(argc, argv, "ps9_main_pseudocode");
    ros::NodeHandle nh;




    // INSTANTIATION OF CLASS FROM YOUR LIBRARIES

    // instantiate an point cloud sensor object
    PointCloudSensor point_cloud_sensor;

    // instantiate an yale hand gripper object
    GripperController gripper_controller;

    // instantiate an arm motion object
    ArmMotionCommander arm_motion_commander(&nh);




    // PREPARATION WORK FOR YOUR LIBRARIES BEFORE THE LOOP

    // preparation work for point cloud sensor
    // check if point cloud from kinect is active
    while (!point_cloud_sensor.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");
    // fill in here if there is more 

    // preparation work for yale gripper
    gripper_controller.check_if_gripper_is_ready_to_go();
    gripper_controller.show_me_a_grasp_move_for_test();
    // fill in here if there is more preparation

    // preparation work for arm motion commander
    tf::StampedTransform tf_sensor_frame_to_torso_frame;  // transform from sensor frame to torso frame
    tf::TransformListener tf_listener;  // start a transform listener
    // warm up the tf_listener
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            // The direction of the transform returned will be from the target_frame to the source_frame. 
            // Which if applied to data, will transform data in the source_frame into the target_frame.
            // See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();  // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");  // tf-listener found a complete chain from sensor to world




    // VARIABLES TO BE USED IN THE LOOP

    // for point cloud sensor

    // for gripper control

    // for arm motion commander
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    Eigen::Vector3f plane_normal, major_axis, centroid;
    Eigen::Matrix3d Rmat;
    int rtn_val;    
    double plane_dist;
    geometry_msgs::PoseStamped rt_tool_pose_origin;  // right hand pose in the selected points
    geometry_msgs::PoseStamped rt_tool_pose_upper;  // right hand pose upper from origin
    geometry_msgs::PoseStamped rt_tool_pose_left;  // right hand pose left from origin
    geometry_msgs::PoseStamped rt_tool_pose_right;  // right hand pose right from origin
    // pre movement, move to pre-define pose in joint space
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();  // plan the path
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();  // execute the planned path




    // THE LOOP

    while (ros::ok()) {
        if (point_cloud_sensor.b_get_colored_block()) {  // check if there is blocks on the stool
            // if here, means there is new block been placed on the stool

            // get block color from point cloud
            block_color = point_cloud_sensor.get_block_color();

            // get block position and orientation, and move arm to it
            arm_position = point_cloud_sensor.get_block_position();
            arm_orientation = point_cloud_sensor.get_block_orientation();
            right_arm_pose = arm_motion_commander.set_destination_pose(arm_position, arm_orientation);
            arm_motion_commander.move_to_destination();

            // grasp the block
            gripper_controller.grasp_the_block();

            // move right arm away to a pre-defined pose
            arm_motion_commander.move_to_pre_define_pose();

            // assign different arm destination according to block color
            // we still need to perform different actions for different color
            // for one color we are going to track the coordinate block's position
            // and move the block to this position
            // coordinate block could be hold by hand
            switch block_color:
                case point_cloud_sensor.get_color_blue():
                    // if the color is blue defined in the point color class
                    // then the arm poses is ...
                    right_arm_pose...
                case point_cloud_sensor.get_color_red();
                    // if the color is red defined in the point color class
                    // then the arm poses is ...
                    right_arm_pose...
                case point_cloud_sensor.get_color_green();
                    // if the color is green defined in the point color class
                    // then the arm poses is ...
                    right_arm_pose...

            // move right arm to destiantion pose
            arm_motion_commander.set_destination_pose(right_arm_pose);
            arm_motion_commander.move_to_destination();

            // unrelease the block
            gripper_controller.release_the_block();

            // move right arm to pre-define pose
            arm_motion_commander.move_to_pre_define_pose();

            ros::Duration(3.0).sleep();  // delay for a while, for cleaning the stool
        }
        ros::Duration(0.5).sleep();  // sleep for half a second
        ros::spinOnce();  // let variables to update, if there is any
    }
}

