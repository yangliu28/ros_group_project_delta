#ifndef Block_DETECTION_H_
#define Block_DETECTION_H_

#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


#define roughHeight -0.129146973155
#define HeightRange 0.05

#define StoolColor_R 74
#define StoolColor_G 138
#define StoolColor_B 169

#define FloorColor_R 74
#define FloorColor_G 138
#define FloorColor_B 169

#define Maxerr 100

#define StoolRadius 0.4
#define HandMinHeight 0.1
#define BlockMaxHeight 0.1
#define BlockTopRadius 0.006
#define BlockRadius 0.05

class Block_detection
{
public:
    Block_detection(ros::NodeHandle* nodehandle);
    int find_color(Vector3f color_wanted); 
    bool find_block(); 

    bool find_stool();
    bool find_floor();
    geometry_msgs::Pose getBlockPose(); 
    Eigen::Vector3d getColor(); 

    CwruPclUtils cwru_pcl_utils;

private:
    void update_kinect_points();
    void transform_clr_kinect_cloud(Eigen::Affine3f A);
    geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
    void display_points(PointCloud<pcl::PointXYZ> points);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_;   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pclKinect_clr_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr display_ptr_;
    ros::Publisher points_publisher;  
    geometry_msgs::Pose BlockPose;
    Eigen::Vector3d BlockColor;

    Eigen::Vector3d StoolColor;
    Eigen::Vector3f StoolCentroid;

    double StoolHeight;


    Eigen::Vector3d FloorColor;
    Eigen::Vector3f FloorCentroid;

    double FloorHeight;

    
    Eigen::Vector3f Block_Major;
    Eigen::Vector3f Block_Normal;
    Eigen::Vector3f BlockCentroid;


    ros::Subscriber pointcloud_subscriber_;
    bool got_kinect_cloud_;
    void KinectCameraCB(const sensor_msgs::PointCloud2ConstPtr& cloud) ;
    bool got_kinect_cloud() { return got_kinect_cloud_; };
    void reset_got_kinect_cloud() {got_kinect_cloud_= false;};
    void set_got_kinect_cloud() {got_kinect_cloud_ = true;};
};

#endif // Block_DETECTION_H_


