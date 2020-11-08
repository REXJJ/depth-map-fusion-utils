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

#include<Volume.hpp>
#include<VisualizationUtilities.hpp>
#include<DebuggingUtilities.hpp>
#include<TransformationUtilities.hpp>
#include<Camera.hpp>
#include<RayTracingEngine.hpp>
#include<PointCloudProcessing.hpp>
#include<Algorithms.hpp>

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    Vector3f centroid;
    centroid<<min_pt.x+(max_pt.x-min_pt.x)/2.0,min_pt.y+(max_pt.y-min_pt.y)/2.0,min_pt.z+(max_pt.z-min_pt.z)/2.0;
    // pcl::PointCloud<Normal>::Ptr normals_new(new pcl::PointCloud<pcl::Normal>);
    // makePointCloudNormal(cloud,normals_new,{1.0,1.0,1.0});
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 0
    for(int i=0;i<cloud->points.size();i++)
    {
        PointXYZRGBNormal pt;
        pt.x = cloud->points[i].x;
        pt.y = cloud->points[i].y;
        pt.z = cloud->points[i].z;
        pt.normal[0] = normals_new->points[i].normal[0];
        pt.normal[1] = normals_new->points[i].normal[1];
        pt.normal[2] = normals_new->points[i].normal[2];
        cloud_filtered->points.push_back(pt);
    }
    cloud_filtered->height = 1;
    cloud_filtered->width  = cloud->points.size();
#endif
    int counter = 0;
    std::cout<<"Total Points: "<<std::endl;
    std::cout<<cloud_temp->points.size()<<endl;
    for(int i=0;i<cloud_temp->points.size();i++)
    {
        PointXYZRGBNormal pt = cloud_temp->points[i];
        if(pt.normal[2]<0)
            continue;
        // Vector3f a,b;
        // a<<(pt.x-centroid(0)),(pt.y-centroid(1)),(pt.z-centroid(2));
        // b<<pt.normal[0],pt.normal[1],pt.normal[2];
        // if(acos(a.dot(b))>=90)
        //     continue;
        cloud_filtered->points.push_back(pt);
        counter++;
        
    }
    cloud_filtered->height = 1;
    cloud_filtered->width = counter;
    string filename = string(argv[1]);
    vector<string> result; 
    boost::split(result, filename , boost::is_any_of("."));
    string output = result[result.size()-1];
    pcl::io::savePCDFileASCII (output+"_filtered.pcd", *cloud_filtered);
    return 0;
}
