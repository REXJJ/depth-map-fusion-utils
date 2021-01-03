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
#include <TSPSolver.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;

double euclideanDistance(Vector3f a, Vector3f b)
{
    return (a-b).norm();
}

vector<Affine3f> createCameraLocationsFromSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,Vector3f center)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    for(int i=0;i<sphere->points.size();i++)
    {
        pcl::PointXYZRGBNormal ptn;
        pcl::PointXYZRGB pt = sphere->points[i];
        ptn.x = pt.x;
        ptn.y = pt.y;
        ptn.z = pt.z;
        Vector3f loc = {pt.x,pt.y,pt.z};
        Vector3f normal = (loc-center).normalized();
        memcpy(ptn.normal,normal.data(),3*sizeof(float));
        cloud->points.push_back(ptn);
    }
    vector<Affine3f> locations;
    for(int i=0;i<cloud->points.size();i++)
    {
        auto location = positionCamera(cloud,i,10);
        locations.push_back(location);
    }
    return locations;
}

vector<Affine3f> repositionCamerasSampled(vector<Affine3f> cameras,VoxelVolume& volume,Camera cam)
{
    RayTracingEngine engine(cam);    
    /**
     * Algorithm:
     * foreach camera locations; do 
     *      get the point of intersection with the volume.
     *      move 30-40cm away from that point. 
     * return the points.
     */
    vector<Affine3f> new_locations;
    for(auto camera:cameras)
    {
        auto nearest_point = engine.rayTraceAndGetMinimum(volume,camera);  
        std::cout<<"The nearest point is: "<<nearest_point<<std::endl;
        if(nearest_point==-1)
        {
            std::cout<<"Skipping the processing of this points. Debug this."<<std::endl;
            new_locations.push_back(camera);
            continue;
        }
        Vector3f bz = {camera(0,2),camera(1,2),camera(2,2)};
        Vector3f current_pt = {camera(0,3),camera(1,3),camera(2,3)};
        Vector3f intersection_pt = current_pt + bz*(float(nearest_point)/1000.0);
        Vector3f new_point = intersection_pt - bz*(0.3);
        auto new_location = camera;
        new_location(0,3) = new_point(0);
        new_location(1,3) = new_point(1);
        new_location(2,3) = new_point(2);
        new_locations.push_back(new_location);
    }
    return new_locations;
}

bool willCollide(VoxelVolume& volume, Vector3f a, Vector3f b)
{
    double distance = (a-b).norm();
    std::cout<<"Distance: "<<distance<<std::endl;
    Vector3f v = (b-a).normalized();
    bool collided = false;
    for(int depth=1;collided==false;depth++){
        Vector3f pt = a + v*double(depth)/1000.0;
        double xx = pt(0);
        double yy = pt(1);
        double zz = pt(2);
        if(depth>distance*1000)
            break;
        if(volume.validPoints(xx,yy,zz)==false)
            continue;
        auto coords = volume.getVoxel(xx,yy,zz);
        int xidn = get<0>(coords);
        int yidn = get<1>(coords);
        int zidn = get<2>(coords);
        if( volume.voxels_[xidn][yidn][zidn]!=nullptr )
        {
            collided = true;
        }
    }
    return collided;
}

class Planner
{
    public:
        vector<Affine3f> camera_locations_;
        VoxelVolume volume;
        vector<string> filenames_;
        Planner(vector<string> filenames);
        void sample_locations();
        void run_tsp();
};

Planner::Planner(vector<string> filenames)
{
    filenames_ = filenames;
}

void Planner::sample_locations()
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filenames_[0], *cloud_temp) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        exit(-1);
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
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    constexpr double PI = 3.141592653589793238462643383279502884197;
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*cloud, min_pt, max_pt);
    volume.setDimensions(min_pt.x,max_pt.x,min_pt.y,max_pt.y,min_pt.z,max_pt.z);
    //The raycasting mechanism needs the surface to have no holes, so the resolution should be selected accordingly.
    double x_resolution = (max_pt.x-min_pt.x)*63;//TODO: Constexpr this.
    double y_resolution = (max_pt.y-min_pt.y)*63;
    double z_resolution = (max_pt.z-min_pt.z)*63;
    // cout<<x_resolution<<" "<<y_resolution<<" "<<z_resolution<<endl;
    volume.setVolumeSize(int(x_resolution),int(y_resolution),int(z_resolution));
    volume.constructVolume();
    volume.integratePointCloud(cloud,normals);
    Camera cam(K);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    Vector3f center;
    center(0) = volume.xcenter_;
    center(1) = volume.ycenter_;
    center(2) = volume.zcenter_;
    for(int i=0;i<3;i++)
        transformation(i,3) = center(i);
    auto radius = max(max((volume.xmax_-volume.xmin_),(volume.ymax_-volume.ymin_)),(volume.zmax_-volume.zmin_));
    std::cout<<"The Radius is : "<<radius<<std::endl;
    generateSphere(radius,sphere,transformation,center(2));
    vector<Affine3f> camera_locations = createCameraLocationsFromSphere(sphere,center);
    auto new_locations = repositionCamerasSampled(camera_locations,volume,cam);
    VisualizationUtilities::PCLVisualizerWrapper viz(255,255,255);
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    for(int i=0;i<new_locations .size();i++)
    {
        viz.addCamera(cam,new_locations [i],"camera_"+to_string(i),30);
    }
    viz.spinViewer();
    camera_locations_ = new_locations;
}

void Planner::run_tsp()
{
    VisualizationUtilities::PCLVisualizerWrapper viz(255,255,255);
    // viz.addCoordinateSystem();
    int V = camera_locations_.size();
    vector<vector<int>> map = vector<vector<int>>(V,vector<int>(V,0));
    // vector<Eigen::Affine3f> camera_locations;
    for(int i=0;i<V;i++)
    {
        for(int j=0;j<V;j++)
        {
            std::cout<<"I,J: "<<i<<" "<<j<<std::endl;
            Vector3f a = { camera_locations_[i](0,3),camera_locations_[i](1,3),camera_locations_[i](2,3) };
            Vector3f b = { camera_locations_[j](0,3),camera_locations_[j](1,3),camera_locations_[j](2,3) };
            if(willCollide(volume,a,b)==true)
            {
                std::cout<<"Collided"<<std::endl;
                map[i][j] = INT_MAX;
            }
            else
                map[i][j] =  euclideanDistance(a,b)*1000;
        }
    }

    TSP::NearestNeighborSearch nnsolver(map);
    nnsolver.solve();
    std::cout<<"Nearest Neighbor Path Length: "<<nnsolver.getPathLength()<<std::endl;
    TSP::TwoOptSearch twsolver(map);
    twsolver.path_ = nnsolver.path_;
    twsolver.solve();
    std::cout<<"Two Opt Path Length: "<<twsolver.getPathLength()<<std::endl;
	// auto best = TSPUtil(map); 
    int prev = -1;
    vector<int> path;
    std::cout<<"Size of Path: "<<twsolver.path_.size()<<std::endl;
    for(int i=0;i<V;i++)
    {
        // int id = best.gnome[i]-'0';
        int id = twsolver.path_[i];
        path.push_back(id);
        std::cout<<"Id: "<<id<<std::endl;
        if(prev>=0)
        {
            vector<double> start = { camera_locations_[prev](0,3),camera_locations_[prev](1,3), camera_locations_[prev](2,3) };
            vector<double> end = { camera_locations_[id](0,3),camera_locations_[id](1,3), camera_locations_[id](2,3) };
            viz.addLine(start,end,"line_path"+to_string(prev)+to_string(id));
        }
        prev = id;
    }

    viz.spinViewer();
    std::cout<<"Writing the path.."<<std::endl;
    ofstream file("rex_path.csv");
    for(auto x:path)
    {
    	auto t = camera_locations_[x];
        std::cout<<x<<std::endl;
    	vector<double> position;
    	position.push_back(t(0,3));
    	position.push_back(t(1,3));
    	position.push_back(t(2,3));

    	position.push_back(t(0,2));
    	position.push_back(t(1,2));
    	position.push_back(t(2,2));
	    for(auto pos: position)
	    	std::cout<<pos<<" ";
	    std::cout<<std::endl;
	    file<<position[0];
	    for(int i=1;i<position.size();i++)
	    	file<<", "<<position[i];
	    file<<endl;
    }
}

int main(int argc, char** argv)
{
    vector<string> filenames;
    filenames.push_back(argv[1]);
    Planner planner(filenames);
    planner.sample_locations();
    planner.run_tsp();
    return 0;
}
