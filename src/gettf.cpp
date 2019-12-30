#include "gettf.h"
#include <iostream>
#include <fstream>

using namespace std;

//getTf constructor
Gettf::Gettf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool debug)
{
    //cout << "GetTf CREATED" << endl;
    //cout << "##############################" << endl;
    main_cloud = cloud;
    if (debug)
    {
        //Create viewer
        viewer = vis.createViewer();
    }
}

//getTf member function
void Gettf::build_center(string name, vector<int> roi, bool debug)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    tf_struct_data center;

    //Find table TF
    if (name == "table"){
        *temp_cloud = *main_cloud;
        temp_cloud = filter_pcd(temp_cloud);
        temp_cloud = filter.outlier_Removal(temp_cloud);
        segmentation(temp_cloud);
        temp_cloud = objects_struct.table;
        tf_struct_data table_tf = transform.getTf(temp_cloud);
        center.name = name;
        center.center = table_tf.center;
        center.x_axis = table_tf.x_axis;
        center.y_axis = table_tf.y_axis;
        center.z_axis = table_tf.z_axis;
        if (debug)
        {
            viewer = vis.addCloud(viewer, objects_struct.table);
            viewer = vis.addTf(viewer, table_tf);
        }
    }

    //Find object TF
    else{
        *temp_cloud = *main_cloud;
        temp_cloud = cutting_objects(temp_cloud, roi, debug);
        temp_cloud = filter_pcd(temp_cloud);
        temp_cloud = filter.outlier_Removal(temp_cloud);
        segmentation(temp_cloud);
        tf_struct_data object_tf = transform.getTf(objects_struct.object);

        center.name = name;
        center.center = object_tf.center;
        center.x_axis = object_tf.x_axis;
        center.y_axis = object_tf.y_axis;
        center.z_axis = object_tf.z_axis;
        if (debug)
        {
            //viewer = vis.addCloud(viewer, objects_struct.object);
            viewer = vis.addCloud(viewer, temp_cloud);
            viewer = vis.addTf(viewer, object_tf);
        }
    }
    
    tf_br_data tf_br = transform_data(center);

    center_list.push_back(tf_br);
}
    
// getTf member function
vector<tf_br_data> Gettf::build_view(bool debug)
{
    if (debug)
    {
        cout << "list contains: " << endl;
        for(int i = 0; i < center_list.size(); i++)
        {
            cout << "name: " << center_list[i].name << endl;
            cout << "pos_x: " << center_list[i].pos_x << endl;
            cout << "pos_y: " << center_list[i].pos_y << endl;
            cout << "pos_z: " << center_list[i].pos_z << endl;
            cout << "quat_z: " << center_list[i].quat_z << endl;
            cout << "quat_w: " << center_list[i].quat_w << endl;            
        }
    }
    //Return all the TF results
    return (center_list);
}

void Gettf::show_viewer(void)
{   
    vis.showViewer(viewer);
}

// getTf member function
bool Gettf::reset_view(void)
{
    //Clear center_list
    center_list.erase(center_list.begin(),center_list.end());
    objects_struct = obj_struct();
    cout << "cleared list" << endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Gettf::time_test(void)
{
    //This function is purely for testing the speed of sending a PCD!
    return(main_cloud);
}

void Gettf::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //Segment.getTableSeg gets table segmentation and cuts it out of the PCD. It will retrun the table PCD and a PCD containing everything else
    objects_struct = segment.getTableSeg(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Gettf::cutting_objects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> roi, bool debug)
{
    //This function will cut out the object PCD from the total PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    temp_cloud = filter.cut_Filter(cloud, roi[0], roi[2], roi[1], roi[3]);
    return(temp_cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Gettf::filter_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //Filter cloud
    temp_cloud = filter.pt_Filter(cloud);
    temp_cloud = filter.d_Filter(temp_cloud);
    return(temp_cloud);
}

tf_br_data Gettf::transform_data(tf_struct_data center_result)
{
    //Calculate the TF and transform it for the ROS TF broadcaster
    tf_br_data tf_br;
    tf_br.name = center_result.name;
    tf_br.pos_x = center_result.center.x;
    tf_br.pos_y = center_result.center.y;
    tf_br.pos_z = center_result.center.z;

    double yaw = atan((center_result.x_axis.y - center_result.center.y)/(center_result.x_axis.x - center_result.center.x));
    if ((center_result.x_axis.y - center_result.center.y) > 0 && (center_result.x_axis.x - center_result.center.x) < 0)
    {
        //Angle relative to positive x-axis is 180 - angle (angle is negative so we need to add it to 180)
        yaw = PI + yaw;
    }
    else if ((center_result.x_axis.y - center_result.center.y) < 0 && (center_result.x_axis.x - center_result.center.x) < 0)
    {
        //Angle relative to positive x-axis is 180 + angle
        yaw = yaw + PI;
    }
    else if ((center_result.x_axis.y - center_result.center.y) < 0 && (center_result.x_axis.x - center_result.center.x) > 0)
    {
        //Angle relative to positive x-axis is 360 - angle (angle is negative so we need to add it to 360)
        yaw = 2*PI + yaw;
    }

    //Write data in .csv file (Name, Pos_x, Pos_y, Pos_z, Angle (º))
    if(center_result.name == "object")
    {
        string filename = "test.csv";
        fstream fin;
        fin.open(filename, ios::out | ios::app);
        fin.seekg(ios_base::end);
        fin << center_result.name << ", " << 
        center_result.center.x<< ", " << 
        center_result.center.y << ", " <<
        center_result.center.z << ", " <<
        yaw*180/PI << "\n";
        fin.close();  
    }

    //Calculation to go from Euler to Quaternion
    double pitch = 0.0;
    double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    tf_br.quat_x = 0.0;
    tf_br.quat_y = 0.0;
    tf_br.quat_z = sy * cp * cr - cy * sp * sr;
    tf_br.quat_w = cy * cp * cr + sy * sp * sr;

    return tf_br;
}
