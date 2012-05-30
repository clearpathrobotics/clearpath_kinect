#include <ros/ros.h>

#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Publisher pub_cloud;

PointCloud::Ptr temp(new PointCloud);

//***********************
//       Functions
//***********************

void callback(const PointCloud::ConstPtr& msg)
{
    ROS_INFO("Beginning Moving Least Squares on %i Points", msg->points.size());

    // Make a copy of the point cloud
    pcl::copyPointCloud(*msg, *temp);

    // Create a KD-Tree
#if defined PCL_MAJOR_VERSION && defined PCL_MINOR_VERSION && PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 5
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
#else
    // support for PCL version 1.1.1 in electric
    pcl::KdTree<PointType>::Ptr tree (new pcl::KdTreeFLANN<PointType>);
#endif
    tree->setInputCloud (temp);

    // Output has the same type as the input one, it will be only smoothed
    PointCloud mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointType, pcl::Normal> mls;

    // Optionally, a pointer to a cloud can be provided, to be set by MLS
    pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
    mls.setOutputNormals (mls_normals);

    // Set parameters
    mls.setInputCloud (temp);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.05);

    // Reconstruct
    mls.reconstruct(mls_points);

    ROS_INFO("Done MLS");

    pub_cloud.publish(mls_points);
}


int main(int argc, char** argv)
{
    ros::init (argc, argv, "smooth_default");
    
    // Get Parameters
    ros::NodeHandle nh = ros::NodeHandle(ros::this_node::getName());
    std::string cloud_in, cloud_out;
    nh.param<std::string>("cloud_in", cloud_in, "smooth_cloud_in");
    nh.param<std::string>("cloud_out", cloud_out, "smooth_cloud_out");

    // Subscribe
    ros::NodeHandle nx;

    ros::Subscriber cloud_sub = nx.subscribe(cloud_in, 1, callback);
    pub_cloud = nx.advertise<PointCloud> (cloud_out, 1);

    ros::spin();

    return (0);
}



