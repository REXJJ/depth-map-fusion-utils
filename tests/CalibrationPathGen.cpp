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

void process(vector<string> filenames)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    Vector3f center;
    center(0) = 1.26;
    center(1) = -0.226;
    center(2) = 0.325;

    for(int i=0;i<3;i++)
        transformation(i,3) = center(i);

    generateSphere(0.6,sphere,transformation,center(2));
    VisualizationUtilities::PCLVisualizerWrapper viz(255,255,255);
    // viz.addCoordinateSystem();
    viz.addPointCloud<pcl::PointXYZRGB>(sphere);
    int V = sphere->points.size();
    vector<vector<int>> map = vector<vector<int>>(V,vector<int>(V,0));
    // vector<Eigen::Affine3f> camera_locations;
    for(int i=0;i<V;i++)
    {
        for(int j=0;j<V;j++)
        {
            std::cout<<"I,J: "<<i<<" "<<j<<std::endl;
            Vector3f a = { sphere->points[i].x,sphere->points[i].y,sphere->points[i].z };
            Vector3f b = { sphere->points[j].x,sphere->points[j].y,sphere->points[j].z };
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
            vector<double> start = { sphere->points[prev].x,sphere->points[prev].y,sphere->points[prev].z };
            vector<double> end = { sphere->points[id].x,sphere->points[id].y,sphere->points[id].z };
            viz.addLine(start,end,"line_path"+to_string(prev)+to_string(id));
        }
        prev = id;
    }

    viz.spinViewer();
    std::cout<<"Writing the path.."<<std::endl;
    ofstream file("rex_path.csv");
    for(auto x:path)
    {
    	auto t = sphere->points[x];
        std::cout<<x<<std::endl;
    	vector<double> position;
    	position.push_back(t.x);
    	position.push_back(t.y);
    	position.push_back(t.z);

        Vector3f loc = { t.x,t.y,t.z };
        Vector3f normal = (center-loc).normalized();

    	position.push_back(normal(0));
    	position.push_back(normal(1));
    	position.push_back(normal(2));
	    for(auto pos: position)
	    	std::cout<<pos<<" ";
	    std::cout<<std::endl;
	    file<<position[0];
	    for(int i=1;i<position.size();i++)
	    	file<<", "<<position[i];
	    file<<endl;
    }

    std::cout<<"Cloud Points: "<<std::endl;
    for(int i=0;i<sphere->points.size();i++)
    {
        auto pt = sphere->points[i];
        std::cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<std::endl;
    }

}

int main(int argc, char** argv)
{
    vector<string> filenames;
    // filenames.push_back(argv[1]);
    process(filenames);
    return 0;
}
