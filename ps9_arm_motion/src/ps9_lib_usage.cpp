// this program is just for demo usage of ps9_arm_motion_commander_lib library
// this may not be able to run because of lack of pose data

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// for motion client, library implementation is copy from example_baxter_cart_move_action client 
#include <ps9_arm_motion/ps9_arm_motion_commander_lib.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ps9_lib_usage");
    ros::NodeHandle nh;

    // instantiate of the class define in ps9_arm_motion_commander_lib
    ArmMotionCommander arm_motion_commander(&nh);

    // variables that will be used in the loop
    Eigen::Affine3d Affine_pose_A_gripper, Affine_pose_B_gripper;
    Eigen::Matrix3d rotation_mat_A, rotation_mat_B;
    Eigen::Vector3d destination_A, destination_B;
    geometry_msgs::PoseStamped rt_tool_pose;  // right hand pose
    int rtn_val;
    // in the loop, repeat the cycle of moving to pose A and pose B
    while (ros::ok()) {
        // usually, should get pose data from point cloud...
        // get pose A data ready
        Affine_pose_A_gripper.linear() = rotation_mat_A;
        Affine_pose_A_gripper.translation() = destination_A;
        rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_pose_A_gripper);
        // send pose A to the motion plan
        rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        // execute planned motion
        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();

        // get pose data for pose B
        Affine_pose_B_gripper.linear() = rotation_mat_B;
        Affine_pose_B_gripper.translation() = destination_B;
        rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_pose_B_gripper);
        // send pose A to the motion plan
        rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        // execute planned motion
        rtn_val = arm_motion_commander.rt_arm_execute_planned_path();

        // ros::spinOnce() if necessary to update the global sensor data
    }
    return 0;
}

