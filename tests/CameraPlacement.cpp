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

void process(vector<string> filenames)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    readPointCloud(filenames[0],cloud_normal,cloud,normals);
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    cout<<"Pointcloud dimensions: "<<min_pt.x<<" "<<max_pt.x<<" "<<min_pt.y<<" "<<max_pt.y<<" "<<min_pt.z<<" "<<max_pt.z<<endl;
    //Setting up the volume.
    VoxelVolume volume;
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*125;
    double y_resolution = (max_pt.y-min_pt.y)*125;
    double z_resolution = (max_pt.z-min_pt.z)*125;
    cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    vector<double> center = {volume.xcenter_,volume.ycenter_,volume.zcenter_};
    cout<<"Volume Integrated"<<endl;
    //Setting up the camera locations
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    downsample<pcl::PointXYZRGBNormal>(cloud_normal,locations,0.3);
    auto camera_locations = positionCameras(locations);
    Camera cam(K);
    double resolution = volume.voxel_size_;
    int resolution_single_dimension = int(round(cbrt(resolution*1e9)));
    cout<<resolution*1e9<<" Resolution"<<endl;
    cout<<"Resolution Single Dim: "<<resolution_single_dimension<<endl;
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addPointCloudInVolume(volume);
    for(int i=0;i<camera_locations.size();i++)
    {
        viz.addCamera(cam,camera_locations[i],"camera"+to_string(i));
    }
    viz.spinViewer();
}

int main(int argc, char** argv)
{
    vector<string> filenames;
    filenames.push_back(argv[1]);
    process(filenames);
    return 0;
}
