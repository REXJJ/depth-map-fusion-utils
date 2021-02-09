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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_converted (new pcl::PointCloud<pcl::PointXYZRGB>);
    string filename = string(argv[1]);

#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
    }
#endif
    for(auto pt:cloud->points)
    {
        pt.x = pt.x/1000.0;
        pt.y = pt.y/1000.0;
        pt.z = pt.z/1000.0;
        cloud_converted->points.push_back(pt);
    }
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    viz.spinViewer();
    cloud_converted->height = 1;
    cloud_converted->width = cloud->points.size();
    pcl::io::savePCDFileASCII (filename,*cloud_converted);
    return 0;
}
