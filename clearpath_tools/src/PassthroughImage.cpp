#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

#include "ClearpathPointCloudTools.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

ros::Publisher pub;

double dist = 0.0;

void callback(const PointCloud::ConstPtr& cloud)
{
    PointCloud result;
    ClearpathPointCloudTools::PassthroughPointCloudImage((PointCloud*)&(*cloud), &result, dist);
    pub.publish (result);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "passthrough_default");
    
    ros::NodeHandle nh = ros::NodeHandle(ros::this_node::getName());
    std::string in, out;
    nh.param<std::string>("in", in, "passthrough_in");
    nh.param<std::string>("out", out, "passthrough_out");
    nh.param<double>("dist", dist, 1.0);

    ros::NodeHandle nx;
    ros::Subscriber sub = nx.subscribe<PointCloud>(in, 1, callback);
    pub = nx.advertise<PointCloud> (out, 1);

    ros::spin();

    return (0);
}



