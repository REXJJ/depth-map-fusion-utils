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
    void makePointCloudNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,vector<double> viewpoint={0.0,0.0,0.0})
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new PointCloud<pcl::PointXYZ>);
        for(int i=0;i<cloud->points.size();i++)
        {
            PointXYZ ptxyz;
            PointXYZRGB ptxyzrgb = cloud->points[i];
            ptxyz.x=ptxyzrgb.x;
            ptxyz.y=ptxyzrgb.y;
            ptxyz.z=ptxyzrgb.z;
            cloud_bw->points.push_back(ptxyz);
        }
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_bw);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.005);
        ne.setViewPoint(viewpoint[0],viewpoint[1],viewpoint[2]);
        ne.compute (*normals); 
    }
};

