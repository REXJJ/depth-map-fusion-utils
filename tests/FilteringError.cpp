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
#include "nabo/nabo.h"

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Nabo;
using namespace Eigen;
using namespace PointCloudProcessing;
using namespace TransformationUtilities;
using namespace Algorithms;


class Optimizer
{
    private:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scan;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled;
        std::vector<double> transformation;
    public:
        Optimizer();
        void getInputs(string,string);
        pair<double,double> getError();

};

Optimizer::Optimizer()
{
    cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_scan.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_downsampled.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    transformation = std::vector<double>(6,0);
}

void Optimizer::getInputs(string scan_file,string object_file)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (scan_file, *cloud_scan) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (object_file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }
    downsample<pcl::PointXYZRGB>(cloud,cloud_downsampled,0.01);
    pcl::io::savePCDFileASCII ("/home/rex/REX_WS/part.pcd",*cloud);
    pcl::io::savePCDFileASCII ("/home/rex/REX_WS/part_scan.pcd",*cloud_scan);

    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    viz.addPointCloud<pcl::PointXYZRGB>(cloud_scan,"scan");
    viz.spinViewer();
    std::cout<<"Scan and object cloud read..."<<std::endl;
}

pair<double,double> Optimizer::getError()
{
    int K = 1;
    MatrixXf M = MatrixXf::Zero(3, cloud_scan->points.size());
    for(int i=0;i<cloud_scan->points.size();i++)
    {
        auto pt = cloud_scan->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    //Setting up the structures.
    MatrixXf N = MatrixXf::Zero(3, cloud_downsampled->points.size());
    MatrixXi indices;
    MatrixXf dists;
    indices.resize(1, N.cols());
    dists.resize(1, N.cols());
    double maximum = -1e9,average=0.0;
    int counter = 0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    Eigen::Affine3f trans = vectorToAffineMatrix(transformation);
    for(int i=0;i<cloud_downsampled->points.size();i++)
    {
        auto point = cloud_downsampled->points[i];
        Vector3f pt = {point.x,point.y,point.x};
        Vector3f pt_trans = trans*pt;
        N(0,i) = pt_trans(0);
        N(1,i) = pt_trans(1);
        N(2,i) = pt_trans(2);
        counter++;
    }
    nns->knn(N, indices,dists, 1, 0.1, NNSearchF::SORT_RESULTS);
    for(int i=0;i<counter;i++)
    {
        double distance = sqrt(dists(0,i));
        if(maximum<distance)
            maximum=distance;
        average+=distance;
    }
    average=average/counter;

    return make_pair(average,maximum);
}

int main(int argc, char** argv)
{
    Optimizer opti;
    opti.getInputs(argv[1],argv[2]);
    auto t = opti.getError();
    std::cout<<t.first<<" "<<t.second<<std::endl;
    return 0;
}

