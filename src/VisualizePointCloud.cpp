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

Eigen::Affine3f orientCameras(vector<double> center,vector<double> pi,bool print=false)
{
    Vector3f t1(3);
    t1<<center[0],center[1],center[2];
    Vector3f t2(3);
    t2<<pi[0],pi[1],pi[2];
    Vector3f k(3);
    k<<0,0,1;
    k = k.normalized();
    auto n = (t1-t2).normalized();
    double theta = acos(k.dot(n));
    auto b = k.cross(n);
    Eigen::Affine3f Q = Eigen::Affine3f::Identity();
    if(fabs(theta-3.14159)<0.00001)
    {
        cout<<"Camera Directly Above."<<endl;
        Q = vectorToAffineMatrix({pi[0],pi[1],pi[2],0,0,-theta});
        return Q;
    }
    if(print)
    {
        cout<<"Printing Data: "<<endl;
        cout<<"Theta: "<<theta<<endl;
        cout<<"t1: "<<t1<<endl;
        cout<<"t2: "<<t2<<endl;
        cout<<"N: "<<n<<endl;
        cout<<"K: "<<k<<endl;
        cout<<"b: "<<b<<endl;
    }
    double q0 = cos(theta/2);
    double q1 = sin(theta/2)*b(0);
    double q2 = sin(theta/2)*b(1);
    double q3 = sin(theta/2)*b(2);
    Q(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    Q(0,1) = 2*(q1*q2-q0*q3);
    Q(0,2) = 2*(q1*q3+q0*q2);
    Q(1,0) = 2*(q2*q1+q0*q3);
    Q(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
    Q(1,2) = 2*(q2*q3-q0*q1);
    Q(2,0) = 2*(q3*q1-q0*q2);
    Q(2,1) = 2*(q2*q3+q0*q1);
    Q(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;
    Q(0,3) = pi[0];
    Q(1,3) = pi[1];
    Q(2,3) = pi[2];
    return Q;
}

vector<double> moveCameraAway(vector<double> center,vector<double> pi,double distance)
{
    Vector3f t1(3);
    Vector3f t2(3);
    t1<<center[0],center[1],center[2];
    t2<<pi[0],pi[1],pi[2];
    auto n = (t2-t1).normalized()*distance;
#if 0
    vector<double> test = {n(0)+center[0],n(1)+center[1],n(2)+center[2]};
    Vector3f tt(3);
    tt<<test[0],test[1],test[2];
    std::cout<<(tt-t1).normalized()<<endl;
    std::cout<<"----------------------------"<<endl;
    std::cout<<n.normalized()<<endl;
#endif
    return {n(0)+center[0],n(1)+center[1],n(2)+center[2]};
}

vector<Affine3f> repositionCameras(vector<Affine3f> camera_locations,VoxelVolume& volume)
{
    vector<Affine3f> new_camera_locations;
    for(auto x:camera_locations)
        new_camera_locations.push_back(x);
#if 0
    vector<bool> visited(camera_locations.size(),false);
    for(int i=1000;i>=0;i--)
    {
        for(int j=0;j<camera_locations.size();j++)
        {
            if(visited[j])
                continue;
            double distance = double(i)/1000.0;
            auto new_points = moveCameraAway({volume.xcenter_,volume.ycenter_,volume.zcenter_},{camera_locations[j](0,3),camera_locations[j](1,3),camera_locations[j](2,3)},distance);
            double x = new_points[0];
            double y = new_points[1];
            double z = new_points[2];
            if(volume.validPoints(x,y,z)==false)
                continue;
            auto coords = volume.getVoxel(x,y,z);
            int xid = get<0>(coords);
            int yid = get<1>(coords);
            int zid = get<2>(coords);
            Voxel *voxel = volume.voxels_[xid][yid][zid];
            if(voxel!=nullptr)
            {
                visited[j]=true;
                cout<<j<<" "<<distance<<endl;
                new_points = moveCameraAway({volume.xcenter_,volume.ycenter_,volume.zcenter_},new_points,0.3+distance);
                new_camera_locations[j](0,3) = new_points[0];
                new_camera_locations[j](1,3) = new_points[1];
                new_camera_locations[j](2,3) = new_points[2];
            }
        }
    }
#else
    for(int j=0;j<camera_locations.size();j++)
    {
        vector<double> new_points = {camera_locations[j](0,3),camera_locations[j](1,3),camera_locations[j](2,3)};
        double distance = sqrt(pow((volume.xcenter_-new_points[0]),2)+pow((volume.ycenter_-new_points[1]),2)+pow((volume.zcenter_-new_points[2]),2));
        new_points = moveCameraAway({volume.xcenter_,volume.ycenter_,volume.zcenter_},new_points,0.3+distance);
        new_camera_locations[j](0,3) = new_points[0];
        new_camera_locations[j](1,3) = new_points[1];
        new_camera_locations[j](2,3) = new_points[2];
    }
#endif
    return new_camera_locations;
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
    if(stoi(argv[2])==0)
    {
        VisualizationUtilities::PCLVisualizerWrapper viz(0,0,0);
        viz.addCoordinateSystem();
        viz.addPointCloudNormals<pcl::PointXYZRGB>(cloud,normals);
        viz.spinViewer();
    }
    if(stoi(argv[2])==1)
    {
        VisualizationUtilities::PCLVisualizerWrapper viz(1,1,1);
        viz.addCoordinateSystem();
        viz.addPointCloud<pcl::PointXYZRGB>(cloud);
        viz.spinViewer();
    }
    return 0;
}
