#include "ClearpathPointCloudTools.h"

#include "pcl/filters/passthrough.h"

#include <cstdlib>

void ClearpathPointCloudTools::PassthroughPointCloud(PointCloud* cloud, PointCloud* result, double dist)
{
    pcl::PassThrough<PointType> pass;
    PointCloud cloud_filtered;

    pass.setInputCloud (boost::make_shared<const PointCloud > (*cloud));
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, dist);
    pass.filter (cloud_filtered);

    PointCloud final;
    final.height = 1;
    for (int i = 0; i < cloud_filtered.points.size(); i++)
    {
        if ( cloud_filtered.points[i].x == cloud_filtered.points[i].x &&
             cloud_filtered.points[i].y == cloud_filtered.points[i].y &&
             cloud_filtered.points[i].z == cloud_filtered.points[i].z )
        {    
            final.points.push_back(cloud_filtered.points[i]);
        }
    }
    final.width = final.points.size();
    final.header.frame_id = cloud->header.frame_id;

    (*result) = final;
}

void ClearpathPointCloudTools::PassthroughPointCloudImage(PointCloud* cloud, PointCloud* result, double dist)
{
    PointCloud final;
    final = *cloud;
    for (int i = 0; i < final.points.size(); i++)
    {
        if (final.points[i].z == final.points[i].z) // check that it is not nan
        {
            if (final.points[i].z < 0.0 || final.points[i].z > dist)
            {
                final.points[i].x = NAN;
                final.points[i].y = NAN;
                final.points[i].z = NAN;
            }
        }
    }
    *result = final;
}

void ClearpathPointCloudTools::SkimPointCloudImage(PointCloud* cloud, PointCloud* result, unsigned int num_of_times)
{
    unsigned int skip = 2*num_of_times;

    PointCloud cloud_filtered;
    cloud_filtered = *cloud;
    cloud_filtered.width = cloud->width/(skip); 
    cloud_filtered.height = cloud->height/(skip); 
    cloud_filtered.points.resize(cloud_filtered.width*cloud_filtered.height); 

    int w = cloud_filtered.width;
    int h = cloud_filtered.height;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            cloud_filtered.points[i*w+j].x = cloud->points[skip*i*cloud->width+skip*j].x;
            cloud_filtered.points[i*w+j].y = cloud->points[skip*i*cloud->width+skip*j].y;
            cloud_filtered.points[i*w+j].z = cloud->points[skip*i*cloud->width+skip*j].z;
            cloud_filtered.points[i*w+j].rgb = cloud->points[skip*i*cloud->width+skip*j].rgb;
        }
    }
    (*result) = cloud_filtered;
}

