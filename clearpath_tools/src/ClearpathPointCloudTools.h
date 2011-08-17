#ifndef H_CPPOINTCLOUDTOOLS
#define H_CPPOINTCLOUDTOOLS

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "ClearpathStructures.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class ClearpathPointCloudTools
{
	public:

		ClearpathPointCloudTools();
		virtual ~ClearpathPointCloudTools();

        static void PassthroughPointCloud(PointCloud* cloud, PointCloud* result, double dist);
        static void PassthroughPointCloudImage(PointCloud* cloud, PointCloud* result, double dist);
        static void SkimPointCloudImage(PointCloud* cloud, PointCloud* result, unsigned int num_of_times);

	private:

};

#endif
