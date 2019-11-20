#include "pipeline.h"

// Pipeline constructor
Pipeline::Pipeline()
{
    cout << "PIPELINE CREATED" << endl;
    cout << "##############################" << endl;
}

// Pipeline member function
pcl::PointCloud<pcl::PointXYZ>::Ptr Pipeline::getCloud(std::string file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jeroen/catkin_ws/src/suii_3d_vision_ros/pcd/good_bearing_box.pcd", *cloud);
    //pcl::io::loadPCDFile<pcl::PointXYZ> ("src/good_bearing_box.pcd" , *cloud);
    return cloud;
}