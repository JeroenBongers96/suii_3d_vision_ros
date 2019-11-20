#ifndef PIPELINE_H
#define PIPELINE_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class Pipeline
{
    public:
        Pipeline();
        pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud(std::string file_name);
};

#endif