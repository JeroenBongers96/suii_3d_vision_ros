#ifndef PIPELINE_H
#define PIPELINE_H

#include "img_struct.h"
#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

class Pipeline
{
    private:
        img_struct imgs_data;
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::pointcloud pc;
        rs2::points points;
        rs2::frameset frames;

    public:
        Pipeline();
        void pipeStart(void);
        img_struct getFrames(void);
};

#endif