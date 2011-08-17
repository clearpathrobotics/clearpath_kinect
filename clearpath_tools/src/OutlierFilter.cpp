#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
 
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/filters/statistical_outlier_removal.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

void callback(const PointCloud::ConstPtr& cloud)
{
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    pub.publish (cloud_filtered);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "outlier_default");
    
    ros::NodeHandle nh = ros::NodeHandle(ros::this_node::getName());
    std::string in, out;
    nh.param<std::string>("in", in, "outlier_filer_in");
    nh.param<std::string>("out", out, "outlier_filer_out");

    ros::NodeHandle nx;
    ros::Subscriber sub = nx.subscribe<PointCloud>(in, 1000, callback);
    pub = nx.advertise<PointCloud> (out, 1000);

    ros::spin();

    return (0);
}



