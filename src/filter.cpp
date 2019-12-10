#include "filter.h"


//visualisation constructor
Filter::Filter(void)
{
    cout << "FILTER CREATED" << endl;
    cout << "##############################" << endl;
}

// visualisation member function
pcl::PointCloud<pcl::PointXYZ>::Ptr Filter::pt_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //Filter out the outer points which disturb the PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*bodyFiltered);
    return bodyFiltered;
}

// visualisation member function
pcl::PointCloud<pcl::PointXYZ>::Ptr Filter::d_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Downsample the dataset using a leaf size
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.001f, 0.001f, 0.001f);
    vg.filter (*cloud_filtered);
    return cloud_filtered;
}