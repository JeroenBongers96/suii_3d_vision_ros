#include "ros/ros.h"
#include "suii_3d_vision_ros/GetRoi.h"
#include <opencv2/opencv.hpp> 
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

bool GetRoi(suii_3d_vision_ros::GetRoi::Request  &req, suii_3d_vision_ros::GetRoi::Response &res)
{

    //image conversiong
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(req.input);
    cv::Mat img = cv_ptr->image;
    
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    //cv::imshow("Display Image", img);
    //cv::waitKey(0);
    //ROS_INFO("request an image");

    res.output = {1,2,3};
    int size = res.output.size();
    std::cout << "size array is " << size << std::endl;
    for(int i = 0; i < 3; i++)
        ROS_INFO("sending back response: [%ld]", (long int)res.output[i]);
    return(true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_roi_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_roi", GetRoi);
    ROS_INFO("Ready to get ROI's.");
    ros::spin();
    
    return 0;
}