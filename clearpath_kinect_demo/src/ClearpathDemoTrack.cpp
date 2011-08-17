// Includes
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include "geometry_msgs/Twist.h"
#include "ClearpathDemoTools.h"

// Slowest permissible is 10Hz
#define MAX_RUNTIME_SECONDS 0.1f

// Typedef PCL Types
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PCLPointCloud;

// Publishers
ros::Publisher pub;
ros::Publisher pubvel;

// Variables to get from ROS Param
double angular_vel_correction = 1.0;
double lin_speed = 0.2;
double ang_speed = 0.2;
double cam_height = 0.55;
double cam_z_trans = 0.0;
double window_size = 0.5;
int publish_visualization = 1;

// ROS-Independant Demo Tool
ClearpathDemoTools demo;

// PointCloud for drawing
PCLPointCloud::Ptr final(new PCLPointCloud);

// Callback from Kinect
void callback(const PCLPointCloud::ConstPtr& cloud)
{
    // Clear Result
    final->points.clear();

    // Start Timer
    ros::Time begin = ros::Time::now();

    // Convert PCL Point Cloud to our basic version
    // TODO This should be able to be done by a resize + memcpy
    PointCloud c, f;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Point p;  p.x = cloud->points[i].x; p.y = cloud->points[i].y; p.z = cloud->points[i].z; p.rgb = cloud->points[i].rgb;
        c.points.push_back(p);
    }

    // Call Demo Update and get Twist
    Twist tw2 = demo.Update(&c, &f);

    // Convert Twist to ROS-Twist msg
    geometry_msgs::Twist tw;
    tw.linear.x = tw2.linear;
    tw.angular.z = angular_vel_correction*tw2.angular;
    pubvel.publish(tw);        

    if (publish_visualization == 1)
    {
        // Convert our basic PointCloud to PCL Type
        // TODO This should be able to be done by a resize + memcpy
        for (int i = 0; i < f.points.size(); i++)
        {
            PointType p;  p.x = f.points[i].x; p.y = f.points[i].y; p.z = f.points[i].z; p.rgb = f.points[i].rgb;
            final->points.push_back(p);
        }
        
        // Publish Visualization Cloud
        pub.publish(*final);
    }

    ros::Duration duration = ros::Time::now() - begin;
    if (duration.toSec() > MAX_RUNTIME_SECONDS) {
      ROS_WARN("Iteration of vision loop took %f seconds (max is %f)", duration.toSec(), MAX_RUNTIME_SECONDS);
    }
}

// Main
int main(int argc, char** argv)
{
    ros::init (argc, argv, "idfloor_default");    

    // Get ROS Params
    ros::NodeHandle nh = ros::NodeHandle(ros::this_node::getName());
    std::string in, out, out_vel;
    nh.param<std::string>("in", in, "idfloor_in");
    nh.param<std::string>("out", out, "idfloor_out");
    nh.param<std::string>("out_vel", out_vel, "idfloor_out_vel");

    nh.param<double>("angular_vel_correction", angular_vel_correction, 1.0);
    nh.param<double>("lin_speed", lin_speed, 0.3);
    nh.param<double>("ang_speed", ang_speed, 0.3);
    nh.param<double>("cam_height", cam_height, 0.55);
    nh.param<double>("cam_z_trans", cam_z_trans, 0.0);
    nh.param<double>("window_size", window_size, 0.5);

    nh.param<int>("publish_visualization", publish_visualization, 1);

    // Init Demo Tool
    demo.InitTrack(lin_speed,
                   ang_speed,
                   cam_height,
                   cam_z_trans,
                   window_size);

    // Init Subscribers
    ros::NodeHandle nx;
    ros::Subscriber sub = nx.subscribe<PCLPointCloud>(in, 1, callback);

    // Init Publishers
    pub = nx.advertise<PCLPointCloud> (out, 1);
    pubvel = nx.advertise<geometry_msgs::Twist> (out_vel, 1);

    // Set to Spin
    ros::spin();

    return (0);
}



