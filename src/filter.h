#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>

class Filter
{
    private:
        // Define min and max for X, Y and Z
        float minX = -2.5, minY = -2.5, minZ = +0.1;
        float maxX = +2.5, maxY = +2.5, maxZ = +2.5;  
        rs2::points points;

    public:     
        Filter();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr d_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cut_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int xmin, int xmax, int ymin, int ymax);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cut_Filter_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int xmin, int xmax, int ymin, int ymax);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_Removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double meanK, double mulThresh);
};

#endif