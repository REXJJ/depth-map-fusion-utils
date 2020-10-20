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

Eigen::Affine3f orientCameras(pcl::PointXYZRGBNormal pt, bool print=false)
{
    Vector3f t1(3);
    t1<<pt.x,pt.y,pt.z;
    Vector3f k(3);
    k<<0,0,1;
    k = k.normalized();
    Vector3f n(3);
    n<<pt.normal[0],pt.normal[1],-pt.normal[2];
    double theta = acos(k.dot(n));
    auto b = k.cross(n);
    Eigen::Affine3f Q = Eigen::Affine3f::Identity();
    if(fabs(theta-3.14159)<0.00001)
    {
        cout<<"Camera Directly Above."<<endl;
        Q = vectorToAffineMatrix({pt.x,pt.y,pt.z,0,0,-theta});
        return Q;
    }
    if(print)
    {
        cout<<"Printing Data: "<<endl;
        cout<<"Theta: "<<theta<<endl;
        cout<<"t1: "<<t1<<endl;
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
    Q(0,3) = pt.x;
    Q(1,3) = pt.y;
    Q(2,3) = pt.z;
    return Q;
}

vector<Affine3f> repositionCameras(vector<Affine3f> camera_locations,VoxelVolume& volume)
{
    vector<Affine3f> new_camera_locations;
    for(auto x:camera_locations)
        new_camera_locations.push_back(x);
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
        memcpy(pt_n.normal,pt.normal,3*sizeof(float));
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
    viz.addPointCloudNormals<pcl::PointXYZRGB>(cloud,normals);
    viz.spinViewer();
    return 0;
#endif
    Camera cam(K);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    Affine3f transformHemiSphere = vectorToAffineMatrix({volume.xcenter_,volume.ycenter_,volume.zcenter_,0,0,PI/2.0});
    Algorithms::generateSphere(0.3,sphere,transformHemiSphere);
#if 0
    //Visualizing the hemisphere
    viz.addPointCloud<pcl::PointXYZRGBNormal>(locations);
    viz.spinViewer();
    return 0;
#endif
    vector<Affine3f> camera_locations;
    for(auto pt:locations->points)
    {
        auto camera_location = orientCameras(pt);
        camera_locations.push_back(camera_location);
    }
#if 1
    //Visualizing oriented cameras.
    for(int x=0;x<camera_locations.size();x++)
        viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
    viz.spinViewer();
    return 0;
#endif
    double resolution = volume.voxel_size_;
    int resolution_single_dimension = int(round(cbrt(resolution*1e9)));
    cout<<resolution*1e9<<" Resolution"<<endl;
    cout<<"Resolution Single Dim: "<<resolution_single_dimension<<endl;
    camera_locations = repositionCameras(camera_locations,volume);
#if 0
    //Visualizing repositioned cameras.
#if 0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int x=0;x<camera_locations.size();x++)
    {
        pcl::PointXYZRGB pt;
        pt.x = camera_locations[x](0,3);
        pt.y = camera_locations[x](1,3);
        pt.z = camera_locations[x](2,3);
        pt.r = 0;
        pt.g = 0;
        pt.b = 0;
        temp_cloud->points.push_back(pt);
    }
    viz.addPointCloud<pcl::PointXYZRGB>(temp_cloud);
    temp_cloud->height = 1;
    temp_cloud->width = camera_locations.size();
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *temp_cloud);
#else
    for(int x=0;x<camera_locations.size();x++)
        viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
#endif
    viz.spinViewer();
    return 0;
#endif
    RayTracingEngine engine(cam);
#if 0
    //Visualize the object seen from all cameras.
    for(int x=0;x<camera_locations.size();x++)
    {
        viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
        engine.rayTrace(volume,camera_locations[x],resolution_single_dimension);
    }
    viz.addPointCloudInVolumeRayTraced(volume);
    viz.spinViewer();
    return 0;
#endif
    vector<vector<unsigned long long int>> regions_covered;
    cout<<"Printing Good Points"<<endl;
    for(int i=0;i<camera_locations.size();i++)
    {
        // auto[found,good_points] = engine.rayTraceAndGetGoodPoints(volume,camera_locations[i],resolution_single_dimension);
        auto[found,good_points] = engine.rayTraceAndGetPoints(volume,camera_locations[i],resolution_single_dimension);
        sort(good_points.begin(),good_points.end());//Very important for set difference.
        regions_covered.push_back(good_points);
        cout<<good_points.size()<<endl;
    }
    auto cameras_selected = Algorithms::greedySetCover(regions_covered,resolution);
#if 1
    for(auto x:cameras_selected)
        cout<<x<<" ";
    cout<<endl;
    for(auto x:cameras_selected)
    {
        viz.addCamera(cam,camera_locations[x],"camera"+to_string(x));
        // engine.rayTraceAndClassify(volume,camera_locations[x],resolution_single_dimension,false);
        engine.rayTrace(volume,camera_locations[x],resolution_single_dimension,false);
    }    
    // viz.addVolumeWithVoxelsClassified(volume);
    viz.addPointCloudInVolumeRayTraced(volume);
    viz.spinViewer();
    return 0;
#endif
    viz.spinViewer();
    return 0;
}
