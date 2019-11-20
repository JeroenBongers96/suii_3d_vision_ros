#ifndef  TF_BR_DATA_H
#define TF_BR_DATA_H

#include <iostream>
#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

struct tf_br_data
{
    string name;
    double pos_x;
    double pos_y;
    double pos_z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
};

#endif