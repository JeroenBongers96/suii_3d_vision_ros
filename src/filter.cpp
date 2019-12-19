#include "filter.h"


//visualisation constructor
Filter::Filter(void)
{
    cout << "FILTER CREATED" << endl;
    cout << "##############################" << endl;
}

// visualisation member function
pcl::PointCloud<pcl::PointXYZ>::Ptr Filter::pt_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //Filter out the outer points which disturb the PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud_filtered);
    return cloud_filtered;
}

// visualisation member function
pcl::PointCloud<pcl::PointXYZ>::Ptr Filter::d_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Downsample the dataset using a leaf size
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.001f, 0.001f, 0.001f);
    vg.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Filter::cut_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int xmin, int xmax, int ymin, int ymax)
{
    int i = 0;
    for(int y = 0; y < 480; ++y)
    {
        for(int x = 0; x < 640; ++x)
        {
            if(x <= xmin | x >= xmax | y <= ymin | y >= ymax)
            {
                cloud->points[i].x = 0;
                cloud->points[i].y = 0;
                cloud->points[i].z = 0;
            }
            ++i;
        }
    }
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Filter::outlier_Removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double meanK, double mulThresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (mulThresh);
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}
