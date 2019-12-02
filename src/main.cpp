#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf_struct.h"
#include "gettf.h"
#include "getImages.h"

using namespace std;
using namespace cv;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

int main(int argc, char** argv){
    string file_name = "";
    clock_t start;
    double duration;
    bool time_debug = false;
    bool debug = true;

    ros::init(argc, argv,"vision_tf_broadcaster");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    start = clock();
    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"starting time: "<< duration <<'\n';
    }
    
    file_name = "obj";
    
    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"loading pcd from pc: "<< duration <<'\n';
    }

    //get PCD
    Gettf gettf(debug);
    GetImages getimage;
    img_struct images = getimage.GetPic();
    Mat color = images.Image; 
    cloud = images.Cloud;
    getimage.GetRoi(argc, argv, color, debug);
    gettf.send_pcd(cloud, file_name);

    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"Send, filter and get segmentation: "<< duration <<'\n';
    }

    //FOR TESTING SENDING TIME PCD
    cloud = gettf.time_test();
    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"DURATION OF SENDING A PCD FROM FILE TO FILE : "<< duration <<'\n';
    }

    //get TFs of object and table and visualise 
    vector<int> roi;
    for(int i = 1; i <= 4; i++){
        roi.push_back(i);
    }
    
    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"start build table : "<< duration <<'\n';
    }

    string name = "table";
    gettf.build_center(name, roi, debug);

    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"end buil table: "<< duration <<'\n';
    }

    name = "object";
    gettf.build_center(name, roi, debug);

    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"build object center: "<< duration <<'\n';
    } 

    vector<tf_br_data> center_list = gettf.build_view(debug);

    int count = 0;
    
    while(ros::ok())
    {
        for(int i = 0; i < center_list.size(); i++)
            {
                static tf::TransformBroadcaster br;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "camera";
                transformStamped.child_frame_id = center_list[i].name;
                transformStamped.transform.translation.x = center_list[i].pos_x;
                transformStamped.transform.translation.y = center_list[i].pos_y;
                transformStamped.transform.translation.z = center_list[i].pos_z;
                transformStamped.transform.rotation.x = center_list[i].quat_x;
                transformStamped.transform.rotation.y = center_list[i].quat_y;
                transformStamped.transform.rotation.z = center_list[i].quat_z;
                transformStamped.transform.rotation.w = center_list[i].quat_w;
                br.sendTransform(transformStamped);
                if(debug)
                {
                    cout << "published " << center_list[i].name << endl;
                }
            }
        loop_rate.sleep();
        count++;
        if (count >= 10)
        {
            break;
        }

    }

    if(debug)
    {
        //use this while loop to stop viewer with ^c
        while(ros::ok())
        {
            gettf.show_viewer();
            loop_rate.sleep();
        }
    }

    gettf.reset_view();  
    
    return(0);
}