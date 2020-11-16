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

constexpr double ball_radius = 0.01;
constexpr double cylinder_radius = 0.002;
constexpr double downsample_radius = 0.005;

Eigen::Vector4f fitPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  Eigen::MatrixXd lhs (cloud->size(), 3);
  Eigen::VectorXd rhs (cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    const auto& pt = cloud->points[i];
    lhs(i, 0) = pt.x;
    lhs(i, 1) = pt.y;
    lhs(i, 2) = 1.0;

    rhs(i) = -1.0 * pt.z;
  }
  Eigen::Vector3d params = lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);
  Eigen::Vector3d normal (params(0), params(1), 1.0);
  auto length = normal.norm();
  normal /= length;
  params(2) /= length;
  return {normal(0), normal(1), normal(2), params(2)};
}

double pointToPlaneDistance(Eigen::Vector4f plane, vector<double> pt)
{
    return fabs(plane(0)*pt[0]+plane(1)*pt[1]+plane(2)*pt[2]+plane(3))/(sqrt(pow(plane(0),2)+pow(plane(1),2)+pow(plane(2),2)));
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return 0;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    std::cout<<"Pointcloud Read.."<<std::endl;
    auto plane = fitPlane(cloud);
    std::cout<<"Plane Fitted.."<<std::endl;
    double avg_error = 0.0, max_error = -1000.0;
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        auto err = fabs(pointToPlaneDistance(plane,{pt.x,pt.y,pt.z}));
        if(err>max_error)
            max_error = err;
        avg_error = avg_error + err;
    }
    avg_error = avg_error/cloud->points.size();
    std::cout<<"Average Error: "<<avg_error<<" Max Error: "<<max_error<<std::endl;
    std::cout<<"Plane Equation: "<<std::endl;
    std::cout<<plane<<std::endl;
    return 0;
}
