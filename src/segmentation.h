#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "obj_struct.h"
#include <iostream>
#include <thread>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/parse.h>

class Segmentation
{
    private:
        obj_struct objs_data;
    public:
        Segmentation();
        obj_struct getTableSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr getObjectSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr* getObjectSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud);
};

#endif
