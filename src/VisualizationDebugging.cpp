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

#include<Volume.hpp>
#include<VisualizationUtilities.hpp>
#include<DebuggingUtilities.hpp>
#include<TransformationUtilities.hpp>
#include<Camera.hpp>
#include<RayTracingEngine.hpp>
#include<PointCloudProcessing.hpp>
#include<Algorithms.hpp>
#include<CommonUtilities.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

void readPointCloud(string filename,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cout<<"Inside reading function"<<endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filename, *cloud_temp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    cout<<"Parsing the pointcloud"<<endl;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    for(int i=0;i<cloud_temp->points.size();i++)
    {
        PointXYZRGBNormal pt = cloud_temp->points[i];
        PointXYZRGB pt_rgb;
        Normal pt_n;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 0;
        // pt_n.normal = pt.normal;
        pt_n.normal[0] = pt.normal[0];
        pt_n.normal[1] = pt.normal[1];
        pt_n.normal[2] = pt.normal[2];
        cloud->points.push_back(pt_rgb);
        normals->points.push_back(pt_n);
    }
    cout<<"Pointcloud Parsed"<<endl;
}

vector<string> filenames;

class VizD : public VizThread 
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    public:
    VizD()
    {
    }
    void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void process(VisualizationUtilities::PCLVisualizerWrapper &viz);
    void input();
};

void VizD::process(VisualizationUtilities::PCLVisualizerWrapper &viz)
{
    viz.updatePointCloud<pcl::PointXYZRGB>(cloud_,"cloud");
}

void VizD::input()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    readPointCloud(filenames[0],cloud);
    addCloud(cloud);
    updateViewer();
    cout<<"Enter a number"<<endl;
    int n;
    cin>>n;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    readPointCloud(filenames[1],cloud_1);
    addCloud(cloud_1);
    updateViewer();
}

void VizD::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cloud_ = cloud;
}


int main(int argc, char** argv)
{
    if(argc<2)
        usage(string(argv[0]));
    filenames.push_back(string(argv[1]));
    filenames.push_back(string(argv[2]));
    VizD vd;
    vd.makeThreads();
    return 0;
}
