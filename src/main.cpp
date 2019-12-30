#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf_struct.h"
#include "gettf.h"
#include "getImages.h"
#include "nameConverter.h"

using namespace std;
using namespace cv;
pcl::PointCloud<pcl::PointXYZ>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZ>);

int main(int argc, char** argv){
    clock_t start;
    double duration;
    vector<int> roiVect;
    bool timeDebug = false;
    bool debug = false;
    string name = "";
    NameConverter nameConverter;

    //Create vision_tf_broadcaster node
    ros::init(argc, argv,"vision_tf_broadcaster");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    //Start timer, this is for testing purpose only
    start = clock();

    //Get PCD and RGB image from camera
    GetImages getImage;
    img_struct images = getImage.GetPic();
    Mat color = images.Image; 

    //Create object of class Gettf
    Gettf getTf(images.Cloud, debug);

    //Get ROI results from YOLO
    roiVect = getImage.GetRoi(argc, argv, color, debug);
    if(debug)
    {
        for(int x = 0; x < roiVect.size(); x++)
        {
            cout << "roi list contatins : " << roiVect[x] << endl;
        }
    }
    
    //Copy only the data from images.Cloud to mainCloud
    *mainCloud = *images.Cloud;

    //Get TF of table
    name = "table";
    getTf.build_center(name, roiVect, debug);
    
    //Check for number of objects resulted from YOLO
    int nrOfObjs = roiVect.size() / 5;
    if(debug)
        std::cout<< "nr of objects: " << nrOfObjs << endl;
    int counter = 0;
    vector<int> objRoi;
    objRoi.clear();
    
    //Determine TF for every object found by YOLO
    if(nrOfObjs > 0)
    {
        for(int x = 0; x < nrOfObjs; x++)
        {   
            name = nameConverter.convertName(roiVect[counter]);
            if(debug)
            {            
                std::cout << "Counter: " << counter << endl;
                std::cout << "Name: " << name << endl;
            }
            for(int y = 0; y <= 3; y++)
            {
                counter++;
                objRoi.push_back(roiVect[counter]);
                if(debug)
                {
                    std::cout << "roiVect: " << roiVect[counter] << endl;
                    std::cout << "objRoi: " << objRoi[y] << endl;
                }
            }  
            
            //CUT FILTER FOR EVERY OBJECT
            if(debug)
                cout << "objRoi cloud: " << objRoi[0] << ", " << objRoi[2] << ", " << objRoi[1] << ", " << objRoi[3] << ", " << endl;
            getTf.build_center(name, objRoi, debug);
            objRoi.clear();
            counter ++;
        } 
    }

    
    //Get the result list of all the TF's found
    vector<tf_br_data> centerList = getTf.build_view(debug);

    int count = 0;
    
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total time of program: "<< duration <<'\n';


    //Function created to test the time required for sending a PCD.
    double startTime = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    mainCloud = getTf.time_test();    
    double endTime = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Sending PCD with 307200 data points: "<< (endTime - startTime) <<'\n';

    //Broadcast all TF's
    while(ros::ok())
    {   
        //loop 10 times so RVIZ, or other listeners, are able to recive the broadcasted data
        for(int i = 0; i < centerList.size(); i++)
            {
                static tf::TransformBroadcaster br;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "camera";
                transformStamped.child_frame_id = centerList[i].name;
                transformStamped.transform.translation.x = centerList[i].pos_x;
                transformStamped.transform.translation.y = centerList[i].pos_y;
                transformStamped.transform.translation.z = centerList[i].pos_z;
                transformStamped.transform.rotation.x = centerList[i].quat_x;
                transformStamped.transform.rotation.y = centerList[i].quat_y;
                transformStamped.transform.rotation.z = centerList[i].quat_z;
                transformStamped.transform.rotation.w = centerList[i].quat_w;
                br.sendTransform(transformStamped);
                if(debug)
                {
                    cout << "published " << centerList[i].name << endl;
                }
            }
        loop_rate.sleep();
        count++;
        if (count >= 10)
        {
            break;
        }

    }

    //Show the results
    if(debug)
    {
        getTf.show_viewer();
        loop_rate.sleep();        
    }

    getTf.reset_view();  
    
    return(0);
}