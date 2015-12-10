//block_detection.cpp
//by Tao Liu
//12/2/2015 Copy right reserved

#include <ps9_pcl/block_detection.h>

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;

Block_detection::Block_detection(ros::NodeHandle* nodehandle) : cwru_pcl_utils(nodehandle), display_ptr_(new PointCloud<pcl::PointXYZ>),
pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>), transformed_pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>) {
    StoolHeight = roughHeight;

    points_publisher = nodehandle->advertise<sensor_msgs::PointCloud2>("display_points", 1, true);
    pointcloud_subscriber_ = nodehandle->subscribe("/kinect/depth/points", 1, &Block_detection::KinectCameraCB, this);
    got_kinect_cloud_ = false;
    
    BlockPose.position.x = 0;
    BlockPose.position.y = 0;
    BlockPose.position.z = 0;
    BlockPose.orientation.x = 0;
    BlockPose.orientation.y = 0;
    BlockPose.orientation.z = 0;
    BlockPose.orientation.w = 0;

}


void Block_detection::update_kinect_points() 
{
    reset_got_kinect_cloud(); // turn on the camera
    //ROS_INFO("begin to update kinect points");
    while (!got_kinect_cloud()) {
        //ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    //ROS_INFO("got a pointcloud.");


    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    // let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    //ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            // The direction of the transform returned will be from the target_frame to the source_frame. 
            // Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "camera_rgb_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //  tf-listener found a complete chain from sensor to world; ready to roll

    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);

    transform_clr_kinect_cloud(A_sensor_wrt_torso);
    //ROS_INFO("transformed color kinect points");
    set_got_kinect_cloud(); // turn off the camera
}



void Block_detection::transform_clr_kinect_cloud(Eigen::Affine3f A) {
    transformed_pclKinect_clr_ptr_->header = pclKinect_clr_ptr_->header;
    transformed_pclKinect_clr_ptr_->is_dense = pclKinect_clr_ptr_->is_dense;
    transformed_pclKinect_clr_ptr_->width = pclKinect_clr_ptr_->width;
    transformed_pclKinect_clr_ptr_->height = pclKinect_clr_ptr_->height;
    int npts = pclKinect_clr_ptr_->points.size();
    //cout << "transforming npts = " << npts << endl;
    transformed_pclKinect_clr_ptr_->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        transformed_pclKinect_clr_ptr_->points[i].getVector3fMap() = A * pclKinect_clr_ptr_->points[i].getVector3fMap();
        transformed_pclKinect_clr_ptr_->points[i].r = pclKinect_clr_ptr_->points[i].r;
        transformed_pclKinect_clr_ptr_->points[i].g = pclKinect_clr_ptr_->points[i].g;
        transformed_pclKinect_clr_ptr_->points[i].b = pclKinect_clr_ptr_->points[i].b;
    }
}


void Block_detection::display_points(PointCloud<pcl::PointXYZ> points)
{
    sensor_msgs::PointCloud2 pcl2_display_cloud;
    pcl::toROSMsg(points, pcl2_display_cloud);
    pcl2_display_cloud.header.stamp = ros::Time::now();
    points_publisher.publish(pcl2_display_cloud);
}

void Block_detection::KinectCameraCB(const sensor_msgs::PointCloud2ConstPtr& cloud) 
{
    if (!got_kinect_cloud_) 
    {
        pcl::fromROSMsg(*cloud,*pclKinect_clr_ptr_);
        got_kinect_cloud_ = true;
    }

}



bool Block_detection::find_stool() {

    update_kinect_points();

    int npts = transformed_pclKinect_clr_ptr_->points.size();
    vector<int> index;
    Eigen::Vector3f pt;
    vector<double> color_err_RGB;
    double color_err;
    color_err = 255;
    color_err_RGB.resize(3);
    index.clear();
    // ROS_INFO("Try to find the stool. Wait");
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        color_err_RGB[0] = abs(StoolColor_R - transformed_pclKinect_clr_ptr_->points[i].r);
        color_err_RGB[1] = abs(StoolColor_G - transformed_pclKinect_clr_ptr_->points[i].g);
        color_err_RGB[2] = abs(StoolColor_B - transformed_pclKinect_clr_ptr_->points[i].b);

        // ROS_INFO("%f", pt[2]);

        if (abs(pt[2] - roughHeight) < HeightRange) 
        {
            if (color_err_RGB[0] < Maxerr && color_err_RGB[1] < Maxerr && color_err_RGB[2] < Maxerr) 
            {
                index.push_back(i);

            }
            // index.push_back(i);
        }
    }
    if (index.size() < 20) 
    {
        ROS_INFO("Stool not found");
        return 0;
    }
    int n_display = index.size();
    // ROS_INFO("found out %d points on the stool", n_display);


    display_ptr_->header = transformed_pclKinect_clr_ptr_->header;
    display_ptr_->is_dense = transformed_pclKinect_clr_ptr_->is_dense;
    display_ptr_->width = n_display; 
    display_ptr_->height = transformed_pclKinect_clr_ptr_->height;
    display_ptr_->points.resize(n_display);
    for (int i = 0; i < n_display; i++) {
        display_ptr_->points[i].getVector3fMap() = transformed_pclKinect_clr_ptr_->points[index[i]].getVector3fMap();
    }
    // ROS_INFO("display_point conversed.");

    // display_points(*display_ptr_); 
    
    StoolCentroid =cwru_pcl_utils.compute_centroid(display_ptr_);
    StoolHeight = StoolCentroid(2);

    // ROS_INFO_STREAM("Centroid of the Stool"<<StoolCentroid.transpose());
    ROS_INFO_STREAM("Height of the stool"<<StoolHeight);
    
    return true;
}

bool Block_detection::find_floor() 
{

    update_kinect_points();

    int npts = transformed_pclKinect_clr_ptr_->points.size();
    vector<int> index;
    Eigen::Vector3f pt;
    vector<double> color_err_RGB;
    double color_err;
    color_err = 255;
    color_err_RGB.resize(3);
    index.clear();
    ROS_INFO("Try to find the Floor. Wait");
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        color_err_RGB[0] = abs(FloorColor_R - transformed_pclKinect_clr_ptr_->points[i].r);
        color_err_RGB[1] = abs(FloorColor_G - transformed_pclKinect_clr_ptr_->points[i].g);
        color_err_RGB[2] = abs(FloorColor_B - transformed_pclKinect_clr_ptr_->points[i].b);

        //color_err = color_err_RGB[0] + color_err_RGB[1] + color_err_RGB[2];
        if (abs(pt[2] - roughHeight) < HeightRange) 
        {
            if (color_err_RGB[0] < Maxerr && color_err_RGB[1] < Maxerr && color_err_RGB[2] < Maxerr) 
            {
                index.push_back(i);

            }
        }
    }
    if (index.size() < 20) 
    {
        ROS_INFO("Floor not found");
        return 0;
    }
    int n_display = index.size();
    ROS_INFO("found out %d points on the floor", n_display);


    display_ptr_->header = transformed_pclKinect_clr_ptr_->header;
    display_ptr_->is_dense = transformed_pclKinect_clr_ptr_->is_dense;
    display_ptr_->width = n_display; 
    display_ptr_->height = transformed_pclKinect_clr_ptr_->height;
    display_ptr_->points.resize(n_display);
    for (int i = 0; i < n_display; i++) {
        display_ptr_->points[i].getVector3fMap() = transformed_pclKinect_clr_ptr_->points[index[i]].getVector3fMap();
    }
    ROS_INFO("display_point conversed.");

    // display_points(*display_ptr_); 
    
    StoolCentroid =cwru_pcl_utils.compute_centroid(display_ptr_);
    //StoolHeight = StoolCentroid(2);

    ROS_INFO_STREAM("Centroid of the Stool"<<StoolCentroid.transpose());
    //ROS_INFO_STREAM("Height of the stool"<<StoolHeight);
    
    return true;
}


int Block_detection::find_block()
{
    update_kinect_points();
    int npts = transformed_pclKinect_clr_ptr_->points.size();
    Eigen::Vector3f pt;
    Eigen::Vector3f dist;

    vector<int> index;
    std::vector<int> index_color;

    index.clear();
    index_color.clear();

    double distance = 1;
    BlockColor<<0,0,0;
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        dist = pt - StoolCentroid;
        dist[2]=0;
        distance = dist.norm();
        if(distance < StoolRadius)
            if(pt[2]>(StoolHeight+0.02) && pt[2]<BlockMaxHeight)
            {
                index.push_back(i);

                double color_range_R = abs(transformed_pclKinect_clr_ptr_->points[i].r - StoolColor_R);
                double color_range_G = abs(transformed_pclKinect_clr_ptr_->points[i].g -StoolColor_G);
                double color_range_B = abs(transformed_pclKinect_clr_ptr_->points[i].b - StoolColor_B);

                if(color_range_R >= 20 && color_range_G >= 20 && color_range_B >= 20)
                {
                    index_color.push_back(i);
                    BlockColor(0)+=transformed_pclKinect_clr_ptr_->points[i].r;
                    BlockColor(1)+=transformed_pclKinect_clr_ptr_->points[i].g;
                    BlockColor(2)+=transformed_pclKinect_clr_ptr_->points[i].b;
                }
            }
    }
    int n_block_points = index.size();
    if(n_block_points<10)
    {
        ROS_INFO("There is no block on the stool.");
        return 0;
    }
    //ROS_INFO("There is a block with %d points", n_block_points);
    int n_color_points = index_color.size();
    BlockColor/=n_color_points;
    //ROS_INFO_STREAM("The block color:"<<BlockColor.transpose());



    
    display_ptr_->header = transformed_pclKinect_clr_ptr_->header;
    display_ptr_->is_dense = transformed_pclKinect_clr_ptr_->is_dense;
    display_ptr_->width = n_block_points;
    display_ptr_->height = transformed_pclKinect_clr_ptr_->height;
    display_ptr_->points.resize(n_block_points);
    for (int i = 0; i < n_block_points; i++) 
    {
        display_ptr_->points[i].getVector3fMap() = transformed_pclKinect_clr_ptr_->points[index[i]].getVector3fMap();
    }
    // display_points(*display_ptr_);
    
    BlockCentroid =cwru_pcl_utils.compute_centroid(display_ptr_);
    ROS_INFO_STREAM("The centroid of the block:"<<BlockCentroid.transpose());


    
    double block_dist;
    cwru_pcl_utils.fit_points_to_plane(display_ptr_, Block_Normal, block_dist);
    Block_Major = cwru_pcl_utils.get_major_axis();
    //ROS_INFO_STREAM("The major vector of the block's top:"<<Block_Major.transpose());

    //return true;
    if (BlockColor[0] >= BlockColor[1] && BlockColor[0] >= BlockColor[2])
    {
        ROS_INFO("block is red");
        return 1;
    }
    else if (BlockColor[1] >= BlockColor[0] && BlockColor[1] >= BlockColor[2])
    {
        ROS_INFO("block is green");
        return 2;
    }
    else if (BlockColor[2] >= BlockColor[0] && BlockColor[2] >= BlockColor[1])
    {
        ROS_INFO("block is blue");
        return 3;
    }
}


int Block_detection::find_block_by_color(Vector3f color_given) {

    update_kinect_points();

    int npts = transformed_pclKinect_clr_ptr_->points.size();
    vector<int> index;
    Eigen::Vector3f pt;
    vector<double> color_err_RGB;
    double color_err;
    color_err = 255;
    color_err_RGB.resize(3);
    index.clear();

    ROS_INFO("Try to find the stool. Wait");
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        color_err_RGB[0] = abs(color_given[0] - transformed_pclKinect_clr_ptr_->points[i].r);
        color_err_RGB[1] = abs(color_given[1] - transformed_pclKinect_clr_ptr_->points[i].g);
        color_err_RGB[2] = abs(color_given[2] - transformed_pclKinect_clr_ptr_->points[i].b);

        // ROS_INFO("%f", pt[2]);

            if (color_err_RGB[0] < Maxerr && color_err_RGB[1] < Maxerr && color_err_RGB[2] < Maxerr) 
            {
                index.push_back(i);

            }
            // index.push_back(i);
    }
    if (index.size() < 20) 
    {
        ROS_INFO("block not found");
        return 0;
    }
    int n_display = index.size();
    ROS_INFO("found out %d points of the block", n_display);


    display_ptr_->header = transformed_pclKinect_clr_ptr_->header;
    display_ptr_->is_dense = transformed_pclKinect_clr_ptr_->is_dense;
    display_ptr_->width = n_display; 
    display_ptr_->height = transformed_pclKinect_clr_ptr_->height;
    display_ptr_->points.resize(n_display);
    for (int i = 0; i < n_display; i++) {
        display_ptr_->points[i].getVector3fMap() = transformed_pclKinect_clr_ptr_->points[index[i]].getVector3fMap();
    }
    //ROS_INFO("display_point conversed.");

    // display_points(*display_ptr_); 
    
    StoolCentroid =cwru_pcl_utils.compute_centroid(display_ptr_);
    StoolHeight = StoolCentroid(2);

    ROS_INFO_STREAM("Centroid of the block"<<StoolCentroid.transpose());
    ROS_INFO_STREAM("Height of the block"<<StoolHeight);
    
    return true;
}

bool Block_detection::find_hand()
{
    update_kinect_points();
    int npts = transformed_pclKinect_clr_ptr_->points.size();
    Eigen::Vector3f pt;
    Eigen::Vector3f dist;
    vector<int> index;
    index.clear();
    double distance = 1;
    BlockColor<<0,0,0;
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        dist = pt - StoolCentroid;
        dist[2]=0;
        distance = dist.norm();
        // if(distance < StoolRadius)
            if(pt[2]>(StoolHeight+0.10))
            {
                index.push_back(i);
            }
    }
    int n_block_points = index.size();
    if(n_block_points<10)
    {
        ROS_INFO("No hand found");
        return 0;
    }
    ROS_INFO("Hand found");

    return true;
}

// geometry_msgs::Pose Block_detection::find_pose()
// {
//     geometry_msgs::Pose pose;
//     pose.position.x = BlockCentroid[0];
//     pose.position.y = BlockCentroid[1];
//     pose.position.z = BlockCentroid[2];

//     double delta;

//     Eigen::Vector3f unit_axis_x(1,0,0);
//     double sin_delta = Block_Major.dot(unit_axis_x);
//     delta = acos(sin_delta);

//     pose.orientation.x = 0;
//     pose.orientation.y = 0;
//     pose.orientation.z = 0;
//     pose.orientation.w = cos(delta/2);
    
//     // pose.orientation.x = 0;
//     // pose.orientation.y = 0;
//     // pose.orientation.z = sin(delta/2);
//     // pose.orientation.w = cos(delta/2);

//     return pose;

// }

geometry_msgs::Pose Block_detection::find_pose()
{
    BlockPose.position.x = BlockCentroid[0];
    BlockPose.position.y = BlockCentroid[1];
    BlockPose.position.z = BlockCentroid[2];

    double delta;

    Eigen::Vector3f unit_axis_y(0,1,0);
    double cos_delta = Block_Major.dot(unit_axis_y);
    delta = acos(cos_delta);

    BlockPose.orientation.x = 0;
    BlockPose.orientation.y = 0;
    BlockPose.orientation.z = 0;

    //addition of pi/2 to the angle aligns the hand with Block's major axis
    //otherwise, hand orients to minor axis
    BlockPose.orientation.w = cos(delta/2 + M_PI/2);
    
    // pose.orientation.x = 0;
    // pose.orientation.y = 0;
    // pose.orientation.z = sin(delta/2);
    // pose.orientation.w = cos(delta/2);

    return BlockPose;

}

Eigen::Vector3f Block_detection::get_major_axis_unit_vector() {
    double magnitude;
    magnitude = sqrt( pow(Block_Major[0], 2) + pow(Block_Major[1], 2) );
    Eigen::Vector3f unit_vector;
    unit_vector[0] = Block_Major[0] / magnitude;
    unit_vector[1] = Block_Major[1] / magnitude;
    unit_vector[2] = 0;

    return unit_vector;
}

Eigen::Vector3d Block_detection::find_block_color()
{
    return BlockColor;
}
