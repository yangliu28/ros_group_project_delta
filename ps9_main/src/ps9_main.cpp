// main program for libraries in this assignment to be used collaborately

#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// eigen lilbraries to be used in position calculation of arm motion
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
// for motion realization
#include <ps9_arm_motion/ps9_arm_motion_commander_lib.h>
// for gripper control
#include <ps9_gripper_control/ps9_gripper_control.h>
// for block detection with pcl
#include <ps9_pcl/block_detection.h>
// for human hand detection in hmi
#include <ps9_hmi/ps9_hmi_lib.h>


// arm motion strategy base on block colors



int main(int argc, char** argv) {
    ros::init(argc, argv, "ps9_main");
    ros::NodeHandle nh;




    // INSTANTIATION OF CLASS FROM YOUR LIBRARIES

    // instantiate block detection object
    Block_detection block_detection(&nh);
    // instantiate an yale hand gripper object
    Gripper baxter_gripper_control(&nh);
    // instantiate an arm motion object
    ArmMotionCommander arm_motion_commander(&nh);
    // instantiate an human hand detection object
    HumanMachineInterface human_machine_interface;




    // PREPARATION WORK FOR EACH LIBRARIES BEFORE THE LOOP

    // preparation work for point cloud sensor
    // // check if point cloud from kinect is active
    // while (!point_cloud_sensor.got_kinect_cloud()) {
    //     ROS_INFO("did not receive pointcloud");
    //     ros::spinOnce();
    //     ros::Duration(1.0).sleep();
    // }
    // ROS_INFO("got a pointcloud");

    // preparation work for yale gripper
    // check if gripper is working by a grasp move
    baxter_gripper_control.open_hand();  // no delay between these moves
    baxter_gripper_control.close_hand();
    baxter_gripper_control.open_hand();
    ros::Duration(1.0).sleep();

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
    // none

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

    // for hand detection in human machine interface
    bool b_human_hand_present;  // bool value to indicate current human hand status
    bool b_continue_blocks_operation;  // bool value to indicate whether continue operation on blocks




    // THE LOOP

    while (ros::ok()) {
        // baxter should be able to recognize the following human hand signals in sequence
            // human hand signal 1: present
            // human hand signal 2: not present
        // these signals indicate a permit of performing block detection and baxter movements

        b_human_hand_present = human_machine_interface.get_human_hand();
        while (!b_human_hand_present) {
            // wait for 0.5 second and re-check
            ros::Duration(0.5).sleep();
            b_human_hand_present = human_machine_interface.get_human_hand();
            ROS_INFO("waiting for human hand signal 1");
        }
        // if here, get an human hand presence signal
        ROS_INFO("human hand signal 1 detected");

        int time_count = 0;  // time count for hand presence
        while (b_human_hand_present && time_count<10) {
            ros::Duration(1.0).sleep();
            b_human_hand_present = human_machine_interface.get_human_hand();
            time_count = time_count + 1;  // usually an human interaction is within 10 seconds
            // this will avoid program accidently waits here forever
        }

        if (!b_human_hand_present) {  // the while-loop exits because hand signal is detected
            b_continue_blocks_operation = true;
            ROS_INFO("human hand signal 2 detected");
        }
        else {  // the while-loop exits because time is out
            b_continue_blocks_operation = false;
            ROS_ERROR("time out on human hand signal 2");
        }

        // continue operation on blocks
        if (b_continue_blocks_operation) {
            ros::Duration(1.0).sleep();  // let people walk away

            if (block_detection.b_get_colored_block()) {  // check if there is blocks on the stool
                // if here, means there is new block been placed on the stool

                // get block color from point cloud
                block_color = block_detection.get_block_color();

                // get block position and orientation, and move arm to it
                arm_position = block_detection.get_block_position();
                arm_orientation = block_detection.get_block_orientation();
                right_arm_pose = arm_motion_commander.set_destination_pose(arm_position, arm_orientation);
                arm_motion_commander.move_to_destination();

                // grasp the block
                baxter_gripper_control.close_hand();

                // move right arm away to a pre-defined pose
                arm_motion_commander.move_to_pre_define_pose();

                // assign different arm destination according to block color
                // colored-block classification precedure
                switch block_color:
                    case block_detection.get_color_blue():
                        // if the color is blue defined in the point color class
                        // then the arm poses is ...
                        right_arm_pose...
                    case block_detection.get_color_red();
                        // if the color is red defined in the point color class
                        // then the arm poses is ...
                        right_arm_pose...
                    case block_detection.get_color_green();
                        // if the color is green defined in the point color class
                        // then the arm poses is ...
                        right_arm_pose...

                // move right arm to destiantion pose
                arm_motion_commander.set_destination_pose(right_arm_pose);
                arm_motion_commander.move_to_destination();

                // unrelease the block
                baxter_gripper_control.open_hand();

                // move right arm to pre-define pose
                arm_motion_commander.move_to_pre_define_pose();

                ros::Duration(3.0).sleep();  // delay for a while, for cleaning the stool
            }
            else
                ROS_ERROR("fail to get block info");
        }

        ros::spinOnce();  // let variables to update, if there is any
    }
}

