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

*ps9_pcl* package contains a library of point cloud processing, retrive information of color, position and orientation.

*ps9_hmi* package contains a library of human machine interface design.

## Tips for team work
1) In the README.md of each package, give a demo usage of your library, includes the #include, class instantiation and important member function. Or, just give an standalone test program for your library.

2) Make sure there is no compile error before making a pull request.

3) Write comments in your code as much as you can.

## Test instruction
1) Make sure that Baxter's power, e-stop, and network cable are all plugged in and ready to go.

2) For every terminal opened, run `baxter_master` to make sure we are talking to the roscore in baxter.

3) For **arm motion**:

Enable the robot to respond to motion command:

`rosrun baxter_tools enable_robot.py -e`

Start the baxter trajectory interpolation action server:

`rosrun baxter_traj_streamer  traj_interpolator_as`

Start the baxter cartesian move action server:

`rosrun baxter_cartesian_moves baxter_cart_move_as`

4) For **yale gripper**

Gripper visualization in rviz:

`roslaunch cwru_baxter_launch yale_gripper_xform.launch`

Start the ROS motor driver:

`rosrun baxter_gripper dynamixel_motor_node`

5) For **kinect sensor** (our own launch file with sensor calibration data):

`roslaunch ps9_pcl kinect_.launch`

6) For rviz:

`rosrun rviz rviz`

7) If all the above goes right, run the main program:

`rosrun ps9_main ps9_main`

8) 3 to 7 are just for debug, alternatively, skip these and run our launch file:

`roslaunch ps9_main ps9_main.launch`

## Standard operating precedure of HMI
1) After everything is running, the right arm of baxter will move to pre-pose and go to idle mode, waiting for instructions.

2) The tester (human) puts his left hand in a suitable place in front of kinect sensor, let baxter know the tester is placing the blocks.

3) The tester places one block on the stool with right hand, and moves away left hand, let baxter know the tester has finished placing the blocks.

4) Baxter begins to recognize the blocks and make movements. The test then keeps waiting untill baxter finishes her movements. Repeat previous precedures.

