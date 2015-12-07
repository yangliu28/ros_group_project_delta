#ifndef PS9_GRIPPER_CONTROL_H_
#define PS9_GRIPPER_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

class Gripper
{
public:
    Gripper(ros::NodeHandle* nodehandle);  // constructor
	
    // functions are open hand and close hand
    void open_hand_w_position();
    // void close_hand_w_position();
    void close_hand_w_torque();

private:
    ros::NodeHandle nh_;
    ros::Publisher dynamixel_publisher_;
    ros::Publisher torque_toggle_;

    std_msgs::Int16 cmd_msg;
    std_msgs::Bool toggle_msg;

    int current;  // used to keep track of where the hand currently is

};

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef

