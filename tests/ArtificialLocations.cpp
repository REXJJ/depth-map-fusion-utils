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

void run_tsp(vector<Affine3f> camera_locations_)
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
    vector<double> prev_pos(12);
    for(auto x:path)
    {
    	auto t = camera_locations_[x];
        std::cout<<x<<std::endl;
    	vector<double> position;
    	position.push_back(t(0,3));
    	position.push_back(t(1,3));
    	position.push_back(t(2,3));


    	position.push_back(t(0,0));
    	position.push_back(t(1,0));
    	position.push_back(t(2,0));

    	position.push_back(t(0,1));
    	position.push_back(t(1,1));
    	position.push_back(t(2,1));

    	position.push_back(t(0,2));
    	position.push_back(t(1,2));
    	position.push_back(t(2,2));
        if(position==prev_pos)
        {
            prev_pos = position;
            continue;
        }
        prev_pos = position;
	    for(auto pos: position)
	    	std::cout<<pos<<" ";
	    std::cout<<std::endl;
        string cur="";
	    file<<position[0];
	    for(int i=1;i<position.size();i++)
	    	file<<", "<<position[i];
	    file<<endl;
    }
}

int main(int argc, char** argv)
{
    vector<vector<double>> locations={
        {1246.92,-190.203,783.429,1.61,0.85943,1.63687},
        {1246.89,128.101,783.487,1.60995,0.859381,1.63684},
        {1275.81,-597.882,566.684,1.60766,-0.0698675,1.6248},
        {1195.28,560.94,503.438,1.71148,1.77016,1.68834},
        {842.212,259.682,501.206,1.06822,0.829608,0.869383},
        {821.526,-110.391,684.042,1.06797,0.829867,0.869281},
        {850.24,-48.63,502.91,1.07935,0.81348,0.86898},
        {1621.68,-214.53,598.02,1.76191,0.73848,2.51930},
        {1621.67,73.47,598.04,1.76021,0.73853,2.51935}
    };

    for(int i = 0;i<locations.size();i++)
        for(int j=0;j<3;j++)
            locations[i][j]=locations[i][j]/1000.0;

    vector<vector<double>> transformation = {
        { 0.6749,    0.0000,    0.7379,-0.04514},
        {0.7379,   -0.0000,   -0.6749, -0.01944},
        {0,    1.0000,   -0.0000, 0.13323}
    };

    vector<Affine3f> locations_affine;

    MatrixXd transformation_matrix = MatrixXd::Identity(4,4);
    for(int i=0;i<3;i++)
        for(int j=0;j<4;j++)
            transformation_matrix(i,j) = transformation[i][j];

    for(auto x:locations)
    {
        auto y = vectorToTransformationMatrix(x);
        auto z = y*transformation_matrix;
        Affine3f temp = Affine3f::Identity();
        for(int i=0;i<3;i++)
            for(int j=0;j<4;j++)
                temp(i,j) = z(i,j);
        locations_affine.push_back(temp);
    }
    run_tsp(locations_affine);
    return 0;
}
