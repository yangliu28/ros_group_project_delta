# ros_group_project_delta
Group project repository of group delta for EECS 397/600

## Group member and assignment
Tao Liu: PCL w/ color, position orientation

Ke Niu: gripper control

Yang Liu: arm motion planning/execution

Matthew Dobrowsky: human machine interface

## What's inside
*ps9_main* package contains the main program of controlling the work flow of baxter. Libraries from other packages will be used.

*ps9_arm_motion* package contains a library of arm motion planning and execution of baxter's right arm.

*ps9_gripper_control* package contains a library of controlling the yale gripper.

*ps9_pcl_hmi* package contains a library of point cloud processing and human machine interface design, retrive information of color, position and orientation.

## Tips for team work
1) In the README.md of each package, give a demo usage of your library, includes the #include, class instantiation and important member function. Or, just give an standalone test program for your library.

2) Make sure there is no compile error before making a pull request.

3) Write comments in your code as much as you can.

## Test instruction
1) Make sure that Baxter's power, e-stop, and network cable are all plugged in and ready to go.

2) For every terminal opened, run `baxter_master` to make sure we are talking to the roscore in baxter.

3) Enable the robot to respond to motion command:

`rosrun baxter_tools enable_robot.py -e`

4) Start the baxter trajectory interpolation action server:

`rosrun baxter_traj_streamer  traj_interpolator_as`

5) Start the baxter cartesian move action server:

`rosrun baxter_cartesian_moves baxter_cart_move_as`

6) Publish yale gripper transformation (data inside needs to be refined):

`roslaunch cwru_baxter_launch yale_gripper_xform.launch`

7) Start kinect sensor (or our own launch file with sensor calibration data):

`roslaunch cwru_baxter_launch kinect.launch`

8) Start the ROS motor driver:

`rosrun baxter_gripper dynamixel_motor_node`

9) Start rviz:

`rosrun rviz rviz`

10) If all the above goes right, start our main program:

`rosrun ps9_main `

11) 2 to 10 is just for debug, alternative, skip 2 to 10 and run our launch file:

`roslaunch `
