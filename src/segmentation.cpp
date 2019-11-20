#include "segmentation.h"

// Segmentation constructor
Segmentation::Segmentation()
{
    std::cout << "SEGMENTATION CREATED" << std::endl;
    std::cout << "##############################" << std::endl;
}

// Segmentation member function
obj_struct Segmentation::getTableSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02); // Original: 0.02 // 0.005

    int i=0, nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        //std::cout << "PointCloud representing the segmented component: " << cloud_f->points.size () << " data points." << std::endl;
        *cloud = *cloud_f;
    }

    objs_data.table = cloud_plane;
    objs_data.object = cloud;
    return objs_data;
}

// Segmentation member function
pcl::PointCloud<pcl::PointXYZ>::Ptr Segmentation::getObjectSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.005); // original: 0.02
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr biggest_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if( cloud_cluster->points.size () > biggest_cloud->points.size()){
            biggest_cloud = cloud_cluster;
        }
    
    }
    
    return biggest_cloud;
}

/*
////////////////////////////////////////////////////////////////////
//THIS IS FOR LATER IMPLEMENTING SENDING POINTERS INSTEAD OF PCD'S//
////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr* Segmentation::getObjectSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud)
{
    return cloud;
}
*/
