#ifndef TF_H
#define TF_H

#include "tf_struct.h"
#include <iostream>
#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


class Transformation
{
    private:
        tf_struct_data tf_tf_data;
    public:
        Transformation();
        tf_struct_data getTf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif