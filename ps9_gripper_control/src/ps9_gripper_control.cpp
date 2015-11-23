#include <ps9_gripper_control/ps9_gripper_control.h>
#include <std_msgs/Int16.h>

//used to keep track of where the hand currently is
int current = 0; 

Gripper::Gripper(ros::NodeHandle* nodehandle) : nh_(*nodehandle){
	ang_publisher_ = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
	std_msgs::Int16 a;
	//opens the hand on start up to know the position
	a.data = 3800;
	ang_publisher_.publish(a);
	//keep track of current position
	current = 3800;
}

void Gripper::open_hand(){
	std_msgs::Int16 a;
	//this for loop and sleep is so the gripper does not open and close too quickly
	for(int i = current; i>=3000; i--){
	a.data = i;
	ROS_INFO("sending: %d",i);
	ang_publisher_.publish(a);
	ros::Duration(.0005).sleep(); 
	}
	//keep track of new position
	current = 3000;
}

void Gripper::close_hand(){
	std_msgs::Int16 a;
	//this for loop and sleep is so the gripper does not open and close too quickly
	for(int i = current; i<=3800; i++){
	a.data = i;
	ROS_INFO("sending: %d",i);
	ang_publisher_.publish(a);
	ros::Duration(.0005).sleep(); 
	}
	//keep track of new position
	current = 3800;
}