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

std::tuple<bool,Eigen::Affine3f> orientCameras(vector<double> center,vector<double> pi,bool print=false)
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
    std::cout<<"Angles:----------------> "<<degree(theta)<<endl;
    auto b = k.cross(n);
    Eigen::Affine3f Q = Eigen::Affine3f::Identity();
    if(degree(theta)<90)
        return {false,Q};
    if(fabs(theta-3.14159)<0.00001)
    {
        cout<<"Camera Directly Above."<<endl;
        Q = vectorToAffineMatrix({pi[0],pi[1],pi[2],0,0,-theta});
        return {true,Q};
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
    return {true,Q};
}

vector<double> movePointAway(vector<double> pi,vector<double> nor,double distance)
{
    Vector3f p1(3);
    Vector3f n(3);
    p1<<pi[0],pi[1],pi[2];
    n<<nor[0],nor[1],nor[2];
    n = n*distance;
    return {n(0)+pi[0],n(1)+pi[1],n(2)+pi[2]};
}

void  moveCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_initial,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final)
{
    for(auto pt:cloud_initial->points)
    {
        PointXYZRGB pt_n;
        vector<double> nor = {pt.normal[0],pt.normal[1],pt.normal[2]};
        auto new_points = movePointAway({pt.x,pt.y,pt.z},nor,0.3);
        pt_n.x = new_points[0];
        pt_n.y = new_points[1];
        pt_n.z = new_points[2];
        pt_n.r = 255;
        pt_n.g = 0;
        pt_n.b = 0;
        cloud_final->points.push_back(pt_n);
    }
}

vector<Affine3f> positionCameras(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations)
{
    vector<Affine3f> cameras;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto pt:locations->points)
    {
        PointXYZRGB pt_n;
        vector<double> nor = {pt.normal[0],pt.normal[1],pt.normal[2]};
        auto new_points = movePointAway({pt.x,pt.y,pt.z},nor,0.3);
        pt_n.x = new_points[0];
        pt_n.y = new_points[1];
        pt_n.z = new_points[2];
        pt_n.r = 255;
        pt_n.g = 0;
        pt_n.b = 0;
        cloud_final->points.push_back(pt_n);
    }
    for(int i=0;i<locations->points.size();i++)
    {
        auto [status,camera_location] = orientCameras({locations->points[i].x,locations->points[i].y,locations->points[i].z},{cloud_final->points[i].x,cloud_final->points[i].y,cloud_final->points[i].z});
        if(status)
            cameras.push_back(camera_location);
    }
    return cameras;
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
    // PointCloudProcessing::makePointCloudNormal(cloud,normals);
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    constexpr double PI = 3.141592653589793238462643383279502884197;
    VoxelVolume volume;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    cout<<min_pt.x<<" "<<max_pt.x<<" "<<min_pt.y<<" "<<max_pt.y<<" "<<min_pt.z<<" "<<max_pt.z<<endl;
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*125;
    double y_resolution = (max_pt.y-min_pt.y)*125;
    double z_resolution = (max_pt.z-min_pt.z)*125;
    cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    // volume.setVolumeSize(50,50,50);
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addSphere({volume.xcenter_,volume.ycenter_,volume.zcenter_},"origin");
#if 0
    //Visualizing the pointcloud.
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    viz.spinViewer();
    return 0;
#endif
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (argv[2], *locations) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[2]), *cloud);
#endif
    Camera cam(K);
    auto camera_locations = positionCameras(locations);
#if 1
    //Visualizing oriented cameras.
    for(int x=0;x<camera_locations.size();x++)
    {
        viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
    }
    // viz.addPointCloud<pcl::PointXYZRGBNormal>(locations,"InitialCloud");
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);
    // moveCloud(locations,cloud_final);
    // viz.addPointCloud<pcl::PointXYZRGB>(cloud_final,"FinalCloud");
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    viz.spinViewer();
    return 0;
#endif
    return 0;
}
