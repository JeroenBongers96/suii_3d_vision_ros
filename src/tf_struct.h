#ifndef  TF_STRUCT_H
#define TF_STRUCT_H

#include <iostream>
#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

struct tf_struct_data
{
    string name;
    pcl::PointXYZ center;
    pcl::PointXYZ x_axis;
    pcl::PointXYZ y_axis;
    pcl::PointXYZ z_axis;
};

#endif
