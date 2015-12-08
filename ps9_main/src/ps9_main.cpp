// main program for libraries in this assignment to be used collaborately

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
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
#include <ps9_hmi/ps9_hmi_hand_detect.h>

// const double z_offset = 0.275;
// const double z_offset = 0.285;  // too high
// const double z_offset = 0.280;  // too low

// const double z_offset = 0.2;  // too high
// const double z_offset = 0.15;  // a little high
// const double z_offset = 0.12;  // almost
const double z_offset = 0.118;

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
    HumanMachineInterface human_machine_interface(&nh);




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
    ROS_INFO("test the gripper state by close-open precedure");
    // baxter_gripper_control.close_hand_w_torque();
    baxter_gripper_control.open_hand_w_position();
    // ros::Duration(1.0).sleep();

    // preparation work for arm motion commander
    // tf::StampedTransform tf_sensor_frame_to_torso_frame;  // transform from sensor frame to torso frame
    // tf::TransformListener tf_listener;  // start a transform listener
    // // warm up the tf_listener
    // bool tferr = true;
    // ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    // while (tferr) {
    //     tferr = false;
    //     try {
    //         // The direction of the transform returned will be from the target_frame to the source_frame. 
    //         // Which if applied to data, will transform data in the source_frame into the target_frame.
    //         // See tf/CoordinateFrameConventions#Transform_Direction
    //         tf_listener.lookupTransform("torso", "camera_rgb_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
    //     } catch (tf::TransformException &exception) {
    //         ROS_ERROR("%s", exception.what());
    //         tferr = true;
    //         ros::Duration(0.5).sleep();  // sleep for half a second
    //         ros::spinOnce();
    //     }
    // }
    // ROS_INFO("tf is good");  // tf-listener found a complete chain from sensor to world




    // VARIABLES TO BE USED IN THE LOOP

    // for point cloud sensor
    geometry_msgs::Pose block_pose;  // pose of the block
    int block_color;  // 1: red, 2: green, 3: blue
    double block_orientation;  // block orientation in torso frame

    ROS_INFO_STREAM("block position: " << 
        block_pose.position.x << ", " <<
        block_pose.position.y << ", " <<
        block_pose.position.z);
    ROS_INFO_STREAM("block orientation(w): " << block_pose.orientation.w);

    // for gripper control
    // none

    // for arm motion commander
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    Eigen::Vector3f major_axis, centroid;
    zvec_des << 0,0,-1;  // pointing to the ground
    Eigen::Matrix3d Rmat;
    int rtn_val;
    double plane_dist;
    geometry_msgs::PoseStamped rt_tool_pose_origin;  // right hand pose in the selected points
    geometry_msgs::PoseStamped rt_tool_pose_upper;  // right hand pose upper from origin
    geometry_msgs::PoseStamped rt_tool_pose_red_des;  // right hand destination pose for red block
    geometry_msgs::PoseStamped rt_tool_pose_green_des;  // right hand destination pose for green block
    geometry_msgs::PoseStamped rt_tool_pose_blue_des;  // right hand destination pose for blue block
    // prepare the destination position for different colors
    // orientation
    xvec_des << 1,0,0;  // major direction pointing to the front
    yvec_des = zvec_des.cross(xvec_des);
    Rmat.col(0) = xvec_des;
    Rmat.col(1) = yvec_des;
    Rmat.col(2) = zvec_des;
    Affine_des_gripper.linear() = Rmat;
    // position, change the position for different colors here
    // for red
    origin_des[0] = 0.5;
    origin_des[1] = 0.3;
    origin_des[2] = 0.2;
    Affine_des_gripper.translation() = origin_des;
    rt_tool_pose_red_des.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    // for green
    origin_des[0] = 0.55;
    origin_des[1] = 0.0;
    origin_des[2] = 0.2;
    Affine_des_gripper.translation() = origin_des;
    rt_tool_pose_green_des.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    // for blue
    origin_des[0] = 0.5;
    origin_des[1] = -0.3;
    origin_des[2] = 0.2;
    Affine_des_gripper.translation() = origin_des;
    rt_tool_pose_blue_des.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    // pre movement, move to pre-define pose in joint space
    rtn_val = arm_motion_commander.plan_move_to_pre_pose();  // plan the path
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();  // execute the planned path

    // for hand detection in human machine interface
    bool b_human_hand_present;  // bool value to indicate current human hand status
    bool b_continue_blocks_operation;  // bool value to indicate whether continue operation on blocks




    // THE LOOP

    while (ros::ok()) {
        // baxter should be able to recognize the following human hand signals in sequence
            // human hand signal 1: present
            // human hand signal 2: not present
        // these signals indicate a permit of performing block detection and baxter movements

        ROS_INFO("***** msg from HMI *****");
        b_human_hand_present = human_machine_interface.get_human_hand();
        ROS_INFO("***** msg from HMI *****");
        while (!b_human_hand_present) {
            // wait for 0.5 second and re-check
            ros::Duration(0.5).sleep();
            ROS_INFO("***** msg from HMI *****");
            b_human_hand_present = human_machine_interface.get_human_hand();
            ROS_INFO("***** msg from HMI *****");
            ROS_INFO("waiting for human hand signal 1");
        }
        // if here, get an human hand presence signal
        ROS_INFO("human hand signal 1 detected");

        int time_count = 0;  // time count for hand presence
        while (b_human_hand_present && time_count<10) {
            ros::Duration(1.0).sleep();
            ROS_INFO("***** msg from HMI *****");
            b_human_hand_present = human_machine_interface.get_human_hand();
            ROS_INFO("***** msg from HMI *****");
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

        // for debug of following movements
        // b_continue_blocks_operation = true;

        // continue operation on blocks
        if (b_continue_blocks_operation) {
            ros::Duration(1.0).sleep();  // let people walk away

            // begin trying to find the blocks
            if (block_detection.find_stool()) {
                // stool is within range
                block_color = 0;
                block_color = block_detection.find_block();  // get the color of the block
                ROS_INFO_STREAM("block color code: " << block_color);
                // if 0, no block is find
                if (block_color) {
                    ROS_INFO("stool is found, block is found");
                    ROS_INFO_STREAM("color of the block: " << block_color << " (1-red, 2-green, 3-blue)");
                    block_pose = block_detection.find_pose();  // calculate block pose
                    block_pose.position.z = block_pose.position.z + z_offset;
                    block_orientation = block_pose.orientation.w;
                    block_orientation = 2 * acos(block_orientation);  // angle value
                    ROS_INFO_STREAM("block position: " << 
                        block_pose.position.x << ", " <<
                        block_pose.position.y << ", " <<
                        block_pose.position.z);
                    ROS_INFO_STREAM("block orientation(w): " << block_pose.orientation.w);
                    // ROS_INFO_STREAM("block orientation: " << block_orientation);

                    // prepare the general arm position for detected block, Affine_des_gripper
                    // the orientation
                    Eigen::Vector3f block_axis = block_detection.get_major_axis_unit_vector();
                    xvec_des << block_axis[0], block_axis[1], 0;

                    yvec_des = zvec_des.cross(xvec_des);
                    // rotate 90 degree, compare to above
                    // yvec_des << cos(block_orientation), sin(block_orientation), 0;
                    // xvec_des = zvec_des.cross(xvec_des);
                    Rmat.col(0) = xvec_des;
                    Rmat.col(1) = yvec_des;
                    Rmat.col(2) = zvec_des;// the z direction of the gripper
                    Affine_des_gripper.linear() = Rmat;
                    // the position
                    origin_des[0] = block_pose.position.x;
                    origin_des[1] = block_pose.position.y;
                    origin_des[2] = block_pose.position.z - 0.01;  // block height is 0.03
                    Affine_des_gripper.translation() = origin_des;
                    // for rt_tool_pose_origin
                    rt_tool_pose_origin.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
                    // for rt_tool_pose_upper
                    origin_des[2] = block_pose.position.z + 0.1;  // the upper area of block
                    Affine_des_gripper.translation() = origin_des;
                    rt_tool_pose_upper.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);

                    // 1.move the gripper to the upper area of the block
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 1: move the gripper to the upper area of the block");
                    ROS_INFO_STREAM("des pos: " << rt_tool_pose_upper.pose.position.x
                        << ", " << rt_tool_pose_upper.pose.position.y
                        << ", " << rt_tool_pose_upper.pose.position.z);
                    // send move plan request
                    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose_upper);
                    if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) { 
                        // send command to execute planned motion
                        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                    }
                    else {
                        ROS_WARN("Cartesian path to desired pose not achievable");
                    }

                    // 2.move the gripper to the origin of the block
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 2: move the gripper to the origin of the block");
                    ROS_INFO_STREAM("des pos: " << rt_tool_pose_origin.pose.position.x
                        << ", " << rt_tool_pose_origin.pose.position.y
                        << ", " << rt_tool_pose_origin.pose.position.z);
                    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose_origin);
                    if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) { 
                        // send command to execute planned motion
                        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                    }
                    else {
                        ROS_WARN("Cartesian path to desired pose not achievable");
                    }

                    // 3.grasp the block with the gripper
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 3: grasp the block with the gripper");
                    baxter_gripper_control.close_hand_w_torque();

                    // 4.move the gripper to the upper area of the block
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 4: move the gripper to the upper area of the block");
                    ROS_INFO_STREAM("des pos: " << rt_tool_pose_upper.pose.position.x
                        << ", " << rt_tool_pose_upper.pose.position.y
                        << ", " << rt_tool_pose_upper.pose.position.z);
                    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose_upper);
                    if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) { 
                        // send command to execute planned motion
                        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                    }
                    else {
                        ROS_WARN("Cartesian path to desired pose not achievable");
                    }

                    // 5.move the gripper to destination according to block color
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 5: move the gripper to destination according to block color");
                    // set planed motion according to block color
                    switch (block_color) {
                        case 1:  // color is red
                            rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose_red_des);
                            ROS_INFO_STREAM("des pos: " << rt_tool_pose_red_des.pose.position.x
                                << ", " << rt_tool_pose_red_des.pose.position.y
                                << ", " << rt_tool_pose_red_des.pose.position.z);
                            break;
                        case 2:  // color is green
                            rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose_green_des);
                            ROS_INFO_STREAM("des pos: " << rt_tool_pose_green_des.pose.position.x
                                << ", " << rt_tool_pose_green_des.pose.position.y
                                << ", " << rt_tool_pose_green_des.pose.position.z);
                            break;
                        case 3:  // color is blue                    
                            rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose_blue_des);
                            ROS_INFO_STREAM("des pos: " << rt_tool_pose_blue_des.pose.position.x
                                << ", " << rt_tool_pose_blue_des.pose.position.y
                                << ", " << rt_tool_pose_blue_des.pose.position.z);
                            break;
                    }
                    if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) { 
                        // send command to execute planned motion
                        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                    }
                    else {
                        ROS_WARN("Cartesian path to desired pose not achievable");
                    }

                    // 6.release the block
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 6: release the block");
                    baxter_gripper_control.open_hand_w_position();

                    // 7.move the gripper to pre-pose
                    ROS_INFO_STREAM("");  // blank line here
                    ROS_INFO("move 7: move the gripper to pre-pose");
                    rtn_val = arm_motion_commander.plan_move_to_pre_pose();
                    if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) { 
                        // send command to execute planned motion
                        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                    }
                    else {
                        ROS_WARN("Cartesian path to desired pose not achievable");
                    }
                }
                else
                    ROS_WARN("stool is found, block is not found");
            }
            else
                ROS_WARN("stool is not found");
        ros::Duration(1.0).sleep();  // sleep 1 second each time    
        }
        ros::spinOnce();  // let variables to update, if there is any
    }
    return 0;
}

