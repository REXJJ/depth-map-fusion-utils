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

void usage(string program_name)
{
    std::cout<<program_name<<" <Pointcloud filename>"<<std::endl;
    exit(-1);
}

vector<vector<double>> lines;
// lines.push_back({pt.x,pt.y,pt.z,new_points[0],new_points[1],new_points[2]});

vector<string> filenames;

vector<Affine3f> remove_inside(vector<Affine3f> locations,PointXYZRGB min_pt, PointXYZRGB max_pt)
{
    vector<Affine3f> locations_filtered;
    for(int i=0;i<locations.size();i++)
    {
        float x = locations[i](0,3);
        float y = locations[i](1,3);
        float z = locations[i](2,3);
        if(x>min_pt.x&&x<max_pt.x&&y>min_pt.y&&y<max_pt.y&&z>min_pt.z&&z<max_pt.z)
            continue;
        locations_filtered.push_back(locations[i]);
    }
    return locations_filtered;
}

vector<unsigned long long int> setCover(RayTracingEngine engine, VoxelVolume &volume, vector<Affine3f> camera_locations,int resolution_single_dimension,bool sparse = true)
{
    vector<vector<unsigned long long int>> regions_covered;
    cout<<"Printing Good Points"<<endl;
    for(int i=0;i<camera_locations.size();i++)
    {
        std::cout<<"Location: "<<i<<endl;
        vector<unsigned long long int> good_points;
        bool found;
        tie(found,good_points) = engine.reverseRayTraceFast(volume,camera_locations[i],false,resolution_single_dimension/2);
        // tie(found,good_points) = engine.rayTraceAndGetPoints(volume,camera_locations[i],resolution_single_dimension,false);
        sort(good_points.begin(),good_points.end());//Very important for set difference.
        regions_covered.push_back(good_points);
        // cout<<good_points.size()<<endl;
    }
    for(auto x:regions_covered)
        std::cout<<"Sizes: "<<x.size()<<endl;

    auto cameras_selected = Algorithms::greedySetCover(regions_covered);
    std::cout<<"Total Cameras: "<<camera_locations.size()<<endl;
    std::cout<<"Cameras found: "<<cameras_selected.size()<<endl;
    return cameras_selected;
}

void process()
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
    cout<<"Volume Integrated"<<endl;
    //Setting up the camera locations
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr locations(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    downsample<pcl::PointXYZRGBNormal>(cloud_normal,locations,0.1);
    Camera cam(K);
    auto camera_locations = remove_inside(positionCameras(locations),min_pt,max_pt);
    // auto camera_locations = positionCameras(locations);
    double resolution = volume.voxel_size_;
    int resolution_single_dimension = int(round(cbrt(resolution*1e9)));
    cout<<resolution*1e9<<" Resolution"<<endl;
    cout<<"Resolution Single Dim: "<<resolution_single_dimension<<endl;
    std::cout<<"Cameras: "<<camera_locations.size()<<endl;
#if 0
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addVolumeWithVoxelsClassified(volume);
    viz.addCoordinateSystem();
    for(int i=0;i<camera_locations.size();i++)
    {
        viz.addCamera(cam,camera_locations[i],"camera"+to_string(i));
    }
    viz.spinViewer();
    return;
#endif
    /* Setting up the ray tracer.*/
    RayTracingEngine engine(cam);

    auto cameras_selected = setCover(engine,volume,camera_locations,resolution_single_dimension,false);

    /*Optimizing location of the selected cameras.*/
    vector<Affine3f> optimized_camera_locations;
    for(auto x:cameras_selected)
    {
        auto improved_position = optimizeCameraPosition(volume,engine,resolution_single_dimension,locations,x);
        optimized_camera_locations.push_back(improved_position);
    }

    /*Second run of set cover.*/
    cameras_selected = setCover(engine,volume,optimized_camera_locations,resolution_single_dimension,false);

    /*Saving the results.*/
    vector<Affine3f> locations_to_save;
    for(auto x:cameras_selected)
        locations_to_save.push_back(optimized_camera_locations[x]);

    string temp_file = "cameras.tmp";
    if(filenames.size()==3)
        temp_file = filenames[2];
    std::cout<<"Saving outputs in : "<<temp_file<<std::endl;
    writeCameraLocations(temp_file,locations_to_save);
}

int main(int argc, char** argv)
{
    if(argc<2)
        usage(string(argv[0]));
    filenames.push_back(string(argv[1]));
    process();
    return 0;
}
