#include <iostream>
#include "tf_struct.h"
#include "gettf.h"
#include "getImages.h"

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

int main(int argc, char** argv){
    string file_name = "";
    clock_t start;
    double duration;
    bool time_debug = false;
    bool debug = true;

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
    cloud = images.Cloud;
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

    if (debug){
        gettf.show_viewer();
    }

    gettf.reset_view();  
    
    return(0);
}