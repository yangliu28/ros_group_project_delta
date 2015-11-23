#ifndef PS9_GRIPPER_CONTROL_H_
#define PS9_GRIPPER_CONTROL_H_



#include <ros/ros.h> //generic C++ stuff
class Gripper
{
	public:
		Gripper(ros::NodeHandle* nodehandle); //constructor
		
		//functions are open hand and close hand
		void open_hand();
		void close_hand();

	private:
   		ros::NodeHandle nh_; 
   		ros::Publisher ang_publisher_; 

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
