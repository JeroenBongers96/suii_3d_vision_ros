#include "pipeline.h"

using namespace cv;

// Pipeline constructor
Pipeline::Pipeline()
{
    cout << "PIPELINE CREATED" << endl;
    cout << "##############################" << endl;
}

void Pipeline::pipeStart(void)
{
    cfg.enable_stream( RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30 );
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }
    cout << "PIPELINE STARTED" << endl;
    cout << "##############################" << endl;

}

//Function for calculating the pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

//Get frames and push them in struct
img_struct Pipeline::getFrames(void)
{
    frames = pipe.wait_for_frames();

    rs2::frame depth = frames.get_depth_frame();
	rs2::frame color = frames.get_color_frame();

    //write colordata in struct
    Mat image(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    imgs_data.Image = image;

    pc.map_to(color);
	points = pc.calculate(depth);

    auto pcl_cloud = points_to_pcl(points);

    imgs_data.Cloud = pcl_cloud;

    return imgs_data;
}