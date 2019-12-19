#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf_struct.h"
#include "gettf.h"
#include "getImages.h"

using namespace std;
using namespace cv;
pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>);

int main(int argc, char** argv){
    string file_name = "";
    clock_t start;
    double duration;
    vector<int> roi_vect;
    bool time_debug = false;
    bool debug = true;
    string name = "";

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
    
    //main_cloud_2 = images.Cloud;
    //main_cloud

    roi_vect = getimage.GetRoi(argc, argv, color, debug);
    for(int x = 0; x < roi_vect.size(); x++)
    {
        cout << "roi list contatins : " << roi_vect[x] << endl;
    }
    
    main_cloud = images.Cloud;
    gettf.send_pcd(main_cloud, file_name);
    name = "table";
    gettf.build_center(main_cloud, name, roi_vect, debug);
    
    //IMPLEMENT ROI
    int nr_of_objs = roi_vect.size() / 5;
    std::cout<< "nr of objects: " << nr_of_objs << endl;
    int counter = 0;
    vector<int> obj_roi;
    obj_roi.clear();
    
    if(nr_of_objs > 0)
    {
        for(int x = 0; x < nr_of_objs; x++)
        {   
            name = std::to_string(roi_vect[counter]); // link from id
            std::cout << "Counter: " << counter << endl;
            std::cout << "Name: " << name << endl;
            for(int y = 0; y <= 3; y++)
            {
                counter++;
                obj_roi.push_back(roi_vect[counter]);
                std::cout << "roi_vect: " << roi_vect[counter] << endl;
                std::cout << "obj_roi: " << obj_roi[y] << endl;
            }  
            //CUT FILTER FOR EVERY OBJECT

            //main_cloud2
            *main_cloud = *images.Cloud; //make DATA copy of images.Cloud
            test_cloud = gettf.cutting_objects(main_cloud, obj_roi, debug);
            cout << "obj_roi cloud: " << obj_roi[0] << ", " << obj_roi[2] << ", " << obj_roi[1] << ", " << obj_roi[3] << ", " << endl;
            test_cloud = gettf.filter_pcd(test_cloud);
            gettf.build_center(test_cloud ,name, obj_roi, debug);
            obj_roi.clear();
            counter ++;
        } 
    }

    //gettf.send_pcd(cloud, file_name);

    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"Send, filter and get segmentation: "<< duration <<'\n';
    }

    //FOR TESTING SENDING TIME PCD
    main_cloud = gettf.time_test();

    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"DURATION OF SENDING A PCD FROM FILE TO FILE : "<< duration <<'\n';
    }
    
    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"start build table : "<< duration <<'\n';
    }

    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"end buil table: "<< duration <<'\n';
    }

    //name = "object";
    //gettf.build_center(name, roi_vect, debug);
    
    /*
    //IMPLEMENT ROI
    int nr_of_objs = roi_vect.size() / 5;
    std::cout<< nr_of_objs << endl;
    int counter = 0;
    vector<int> obj_roi;
    obj_roi.clear();
    
    if(nr_of_objs > 0)
    {
        for(int x = 0; x < nr_of_objs; x++)
        {   
            counter = x*5;
            name = std::to_string(roi_vect[counter]); // link from id
            std::cout << "Counter: " << counter << endl;
            std::cout << "Name: " << name << endl;
            for(int y = 0; y <= 3; y++)
            {
                counter++;
                obj_roi.push_back(roi_vect[counter]);
                std::cout << "roi_vect: " << roi_vect[counter] << endl;
                std::cout << "obj_roi: " << obj_roi[y] << endl;
            }      
            gettf.build_center(name, obj_roi, debug);
        } 
    }*/

    
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