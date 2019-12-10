#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

class Filter
{
    private:
        // Define min and max for X, Y and Z
        float minX = -0.2, minY = -0.2, minZ = -2.5;
        float maxX = +0.2, maxY = +0.2, maxZ = +2.5;  
    public:     
        Filter();
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr d_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif