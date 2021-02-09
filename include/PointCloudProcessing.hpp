#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <unordered_set> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Core>
/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/common/common.h>

using namespace pcl;

namespace PointCloudProcessing
{
    template <typename PointT> void downsample(typename pcl::PointCloud<PointT>::Ptr cloud,typename pcl::PointCloud<PointT>::Ptr cloud_filtered, double leaf)
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.setInputCloud (cloud);
        sor.filter (*cloud_filtered);
    }
};

