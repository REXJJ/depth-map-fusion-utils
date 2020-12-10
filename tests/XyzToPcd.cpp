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

void readXYZ(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,double scale=1.0)
{
    ifstream file(filename);
    string metrics;
    getline(file,metrics);
    int count = 0;
    while(getline(file,metrics)&&metrics.size())
    {
        vector<string> result; 
        boost::split(result,metrics, boost::is_any_of(" "));
        string temp = result[1];
        vector<string> coords;
        boost::split(coords, temp, boost::is_any_of(","));
        pcl::PointXYZRGB pt;
        pt.x = stof(coords[0])/scale;
        pt.y = stof(coords[1])/scale;
        pt.z = stof(coords[2])/scale;
        pt.r = 0; 
        pt.g = 0;
        pt.b = 0;
        count++;
        cloud->points.push_back(pt);
    }
    cloud->width = count;
    cloud->height = 1;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    readXYZ("/home/rex/REX_WS/Test_WS/data/Part_1/part_1.xyz",cloud,1000.0);
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    viz.spinViewer();
    pcl::io::savePCDFileASCII ("/home/rex/REX_WS/Test_WS/data/Part_1/part_1.pcd",*cloud);
    return 0;
}
