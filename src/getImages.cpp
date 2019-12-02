#include "getImages.h"

using namespace std;

GetImages::GetImages()
{
    pipe.pipeStart();
    cout << "GETIMAGES CREATED" << endl;
}

img_struct GetImages::GetPic(void)
{
    images_struct = pipe.getFrames();
    return images_struct;
}

void GetImages::GetRoi(int argc, char **argv, Mat img, bool debug)
{
    Mat rgb_img = img;

    ros::init(argc, argv, "get_roi_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<suii_3d_vision_ros::GetRoi>("get_roi");
    suii_3d_vision_ros::GetRoi srv;
    
    //https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros

    sensor_msgs::Image img_msg; //message to be sent 
    cv_bridge::CvImage img_bridge;

    std_msgs::Header header; // empty header
    header.seq = 1; //user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    srv.request.input = img_msg;

    if (client.call(srv))
    {
        for(int i = 0; i < 3; i++)
            ROS_INFO("Sum: %ld", (long int)srv.response.output[i]);
    }
    else
    {
        ROS_ERROR("Failed to call service get_roi");
    }
}
