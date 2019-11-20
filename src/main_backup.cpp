#include <iostream>
#include "pipeline.h"
#include "vis.h"
#include "tf.h"
#include "segmentation.h"
#include "tf_struct.h"
#include "obj_struct.h"
#include "filter.h"

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr table (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer::Ptr viewer;

int main(int argc, char** argv){
    string file_name = "";
    
    file_name = argv[1];
    cout << "##############################" << endl;
    cout << "MAIN \n"  << "file_name: " << argv[1] << endl;
    cout << "##############################" << endl;

    if (argc != 2){
        cout << "Add input_file.pcd" << endl;
        return (0);
    }        

    //Get PCD from Pipeline.cpp
    Pipeline pipe;
    cloud = pipe.getCloud(file_name);

    //Filter cloud
    Filter filter;
    cloud = filter.pt_Filter(cloud);
    cloud = filter.d_Filter(cloud);

    //Get segmented PCD's
    Segmentation segment;
    obj_struct objects_struct;
    //Segment.getTableSeg gets table segmentation and cuts it out of the PCD. It will retrun the table PCD and a PCD containing everything else
    objects_struct = segment.getTableSeg(cloud);
    //Need to implement ROI for specific objects and include pt_Filter cut the objects out
    object = segment.getObjectSeg(objects_struct.object);
    //object = *segment.getObjectSeg(&cloud);
    
    //Get tf of table
    Transformation transform;
    tf_struct_data table_tf;
    table_tf = transform.getTf(objects_struct.table);

    //Get tf of object
    tf_struct_data object_tf;
    object_tf = transform.getTf(objects_struct.object);

    //visualize PCD
    //Put visualisation in a debug mode
    Vis vis;
    viewer = vis.createViewer();
    viewer = vis.addCloud(viewer, objects_struct.table);
    viewer = vis.addTf(viewer, table_tf);
    viewer = vis.addCloud(viewer, object);
    viewer = vis.addTf(viewer, object_tf);
    vis.showViewer(viewer);
    
    
    return(0);
}