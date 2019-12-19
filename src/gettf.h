#ifndef GET_TF_H
#define GET_TF_H

#include <iostream>
#include <vector>
#include <math.h>
#include "filter.h"
#include "segmentation.h"
#include "tf.h"
#include "vis.h"
#include "tf_struct.h"
#include "obj_struct.h"
#include "tf_br_data.h"

#define PI 3.14159265

using namespace std;

class Gettf
{
    private:
        //creating variables
        pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        obj_struct objects_struct;
        vector<tf_br_data> center_list;
        //Creating objects of classes
        Filter filter;
        Segmentation segment;
        Transformation transform;
        Vis vis;        
        void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr); 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_objects(pcl::PointCloud<pcl::PointXYZ>::Ptr, vector<int>, bool);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr);
        tf_br_data transform_data(tf_struct_data center_result);

    public:
        Gettf(pcl::PointCloud<pcl::PointXYZ>::Ptr, bool);               
        void build_center(string, vector<int>, bool);
        vector<tf_br_data> build_view(bool);
        void show_viewer(void);
        bool reset_view(void);
        //time_test only for testing speed of sending PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr time_test(void);
};

#endif