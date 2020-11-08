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

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

int main(int argc, char** argv)
{
    if(argc<2)
        usage(string(argv[0]));
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (argv[1], *cloud_temp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for(int i=0;i<cloud_temp->points.size();i++)
    {
        PointXYZRGBNormal pt = cloud_temp->points[i];
        pt.x/=1000.0;
        pt.y/=1000.0;
        pt.z/=1000.0;
        cloud->points.push_back(pt);
    }
    cloud->width = 1;
    cloud->height = cloud_temp->points.size();
    string filename = string(argv[1]);
    vector<string> result; 
    boost::split(result, filename , boost::is_any_of("."));
    string output = result[result.size()];
    std::cout<<"Output File Name: "<<output<<std::endl;
    pcl::io::savePCDFileASCII (output+"_converted.pcd", *cloud);
    return 0;
}
