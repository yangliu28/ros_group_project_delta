//this is the test to make sure it works


#include <ros/ros.h> 
#include <ps9_gripper_control/ps9_gripper_control.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "gripper_test"); //node name
    ros::NodeHandle nh;
    Gripper baxter_gripper_control(&nh);


    while (ros::ok()) {
        //closes hand, waits a second, opens hand
    	baxter_gripper_control.close_hand();
    	ros::Duration(1).sleep(); 
    	baxter_gripper_control.open_hand();
    	ros::Duration(1).sleep(); 
        ros::spinOnce();
    }
}