#ifndef  OBJ_STRUCT_H
#define OBJ_STRUCT_H

#include <iostream>
#include <vector>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct obj_struct
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr table;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object;
};

#endif