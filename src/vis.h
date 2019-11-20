#ifndef VIS_H
#define VIS_H

#include "tf_struct.h"
#include <iostream>
#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

struct color{
    int col1;
    int col2;
    int col3;
};

class Vis
{
    private:
        int cloud_count, tf_count;
        color cloud_color;        
        std::string cloud_name;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        tf_struct_data vis_tf_data;
        
    public:        
        Vis();
        pcl::visualization::PCLVisualizer::Ptr createViewer (void);
        pcl::visualization::PCLVisualizer::Ptr addCloud(pcl::visualization::PCLVisualizer::Ptr viewer,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::visualization::PCLVisualizer::Ptr addTf(pcl::visualization::PCLVisualizer::Ptr viewer, tf_struct_data vis_tf_data);
        void showViewer(pcl::visualization::PCLVisualizer::Ptr viewer);
        void nameCounter(int number);
};

#endif