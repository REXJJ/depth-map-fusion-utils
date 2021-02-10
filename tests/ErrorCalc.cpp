/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>
#include <mutex>          // std::mutex

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

/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <Volume.hpp>
#include <VisualizationUtilities.hpp>
#include <DebuggingUtilities.hpp>
#include <TransformationUtilities.hpp>
#include <Camera.hpp>
#include <RayTracingEngine.hpp>
#include <PointCloudProcessing.hpp>
#include <Algorithms.hpp>
#include <CommonUtilities.hpp>
#include <FileRoutines.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_classified (new pcl::PointCloud<pcl::PointXYZRGB>);
    string filename_1 = string(argv[1]);
    string filename_2 = string(argv[2]);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_1, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_2, *cloud_ref) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_ref);

    double max_error = -1.0;
    double avg_error = 0.0;
    for(int i=0;i<cloud->points.size();i++)
    {
        auto ptxyz = cloud->points[i];
        vector<int> indices;
        vector<float> dist;
        auto t=tree->nearestKSearch (ptxyz, 1, indices, dist);
        auto distance = sqrt(dist[0]);
        if(max_error<distance)
            max_error = distance;
        pcl::PointXYZRGB ptc;
        ptc.x = ptxyz.x;
        ptc.y = ptxyz.y;
        ptc.z = ptxyz.z;
        if(distance>0.003)
            ptc.g = 255;
        else if(distance>0.005)
            ptc.r = 255;
        avg_error+=distance;
        cloud_classified->points.push_back(ptc);
    }
    std::cout<<"The max error is : "<<max_error<<std::endl;
    std::cout<<"The avg error is : "<<avg_error/cloud->points.size()<<std::endl;
    cloud_classified->height = 1;
    cloud_classified->width = cloud_classified->points.size();
    string filename = string(argv[3]);
    pcl::io::savePCDFileASCII (filename,*cloud_classified);
    return 0;
}
