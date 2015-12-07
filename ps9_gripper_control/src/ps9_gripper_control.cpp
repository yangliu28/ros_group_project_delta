#include <ps9_gripper_control/ps9_gripper_control.h>

Gripper::Gripper(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
	dynamixel_publisher_ = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
	torque_toggle_ = nh_.advertise<std_msgs::Bool>("dynamixel_motor1_mode", 1);
	// opens the hand on start up to know the position
	cmd_msg.data = 3000;
	dynamixel_publisher_.publish(cmd_msg);
	// initialize and keep track of current position
	current = 3000;
}

// void Gripper::open_hand_w_position() {
// 	ROS_INFO("opening the gripper with position command...");
// 	// sleep, so that gripper does not open and close too quickly
// 	for(int i = current; i>=3000; i--){
// 		cmd_msg.data = i;
// 		dynamixel_publisher_.publish(cmd_msg);
// 		ros::Duration(.0005).sleep(); 
// 	}
// 	// keep track of new position
// 	current = 3000;
// }

void Gripper::open_hand_w_position() {
	ROS_INFO("opening the gripper with position command...");
	// prepare message
	toggle_msg.data = 0;
	cmd_msg.data = 3000;
	// publish operation mode and command to the motor node
	torque_toggle_.publish(toggle_msg);
	ros::Duration(0.2).sleep();  // wait for command been sent
	dynamixel_publisher_.publish(cmd_msg);
	ros::Duration(5.0).sleep();  // leave 5 second to finish
}

// void Gripper::close_hand_w_position() {
// 	ROS_INFO("closing the gripper with position command...");
// 	// sleep, so that gripper does not open and close too quickly
// 	for(int i = current; i<=3800; i++){
// 		cmd_msg.data = i;
// 		dynamixel_publisher_.publish(cmd_msg);
// 		ros::Duration(.0005).sleep();
// 	}
// 	// keep track of new position
// 	current = 3800;
// }

void Gripper::close_hand_w_torque() {
	ROS_INFO("closing the gripper with torque command...");
	// prepare message
	toggle_msg.data = 1;
	cmd_msg.data = 80;
	// publish operation mode and command to the motor node
	torque_toggle_.publish(toggle_msg);
	ros::Duration(0.2).sleep();
	dynamixel_publisher_.publish(cmd_msg);
	ros::Duration(5.0).sleep();  // leave 5 second to finish
}

