#include "tf.h"


Transformation::Transformation()
{
    std::cout << "TRANSFORMATION CREATED" << std::endl;
    std::cout << "##############################" << std::endl;

}

tf_struct_data Transformation::getTf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();
    
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    int scale = 8;
    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis ((major_vector (0) / scale) + mass_center (0), (major_vector (1) / scale) + mass_center (1), (major_vector (2) / scale) + mass_center (2));
    pcl::PointXYZ y_axis ((middle_vector (0) / scale) + mass_center (0), (middle_vector (1) / scale) + mass_center (1), (middle_vector (2) / scale) + mass_center (2));
    pcl::PointXYZ z_axis ((minor_vector (0) / scale) + mass_center (0), (minor_vector (1) / scale) + mass_center (1), (minor_vector (2) / scale) + mass_center (2));

    tf_tf_data.center = center;
    tf_tf_data.x_axis = x_axis;
    tf_tf_data.y_axis = y_axis;
    tf_tf_data.z_axis = z_axis;

    return(tf_tf_data);
}

