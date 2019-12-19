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

int main(int argc, char** argv){
    clock_t start;
    double duration;
    vector<int> roi_vect;
    bool time_debug = false;
    bool debug = true;
<<<<<<< HEAD
    string name = "";
=======
>>>>>>> master

    ros::init(argc, argv,"vision_tf_broadcaster");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    start = clock();

    //get PCD
    GetImages getimage;
    img_struct images = getimage.GetPic();
    
    Mat color = images.Image; 
    Gettf gettf(images.Cloud, debug);

    roi_vect = getimage.GetRoi(argc, argv, color, debug);
    for(int x = 0; x < roi_vect.size(); x++)
    {
        cout << "roi list contatins : " << roi_vect[x] << endl;
    }
    
    *main_cloud = *images.Cloud;
    name = "table";
    gettf.build_center(name, roi_vect, debug);
    
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
            cout << "obj_roi cloud: " << obj_roi[0] << ", " << obj_roi[2] << ", " << obj_roi[1] << ", " << obj_roi[3] << ", " << endl;
            gettf.build_center(name, obj_roi, debug);
            obj_roi.clear();
            counter ++;
        } 
    }

    //FOR TESTING SENDING TIME PCD
    main_cloud = gettf.time_test();
    
    if (time_debug){
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"Finished program: "<< duration <<'\n';
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