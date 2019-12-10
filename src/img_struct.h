#ifndef  IMG_STRUCT_H
#define IMG_STRUCT_H

#include <iostream>
#include <vector>
#include <thread>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

struct img_struct
{
    cv::Mat Image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;
};

#endif