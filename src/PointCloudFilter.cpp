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

constexpr double ball_radius = 0.02;
constexpr double cylinder_radius = 0.002;
constexpr double downsample_radius = 0.005;

void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addPointCloud<pcl::PointXYZRGB>(cloud);
    viz.spinViewer();
}

template <typename PointT> void downsample(typename pcl::PointCloud<PointT>::Ptr cloud,typename pcl::PointCloud<PointT>::Ptr cloud_filtered, double leaf)
{
    pcl::VoxelGrid<PointT> sor;
    sor.setLeafSize (leaf, leaf, leaf);
    sor.setInputCloud (cloud);
    sor.filter (*cloud_filtered);
}

Eigen::Vector3f getNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
  return {normal(0), normal(1), normal(2)};
}

double euclideanNorm(Vector3f a,Vector3f b)
{
    return (a-b).norm();
}

double pointToVectorDistance(Vector3f pt, Vector3f v, Vector3f t)
{
   Vector3f d_xyz = v*ball_radius*2;
   Vector3f v1 = t - d_xyz;
   Vector3f v2 = t + d_xyz;
    Vector3f a = v1 -v2;
    Vector3f b = pt - v2;
    if(a.norm()==0.0)
        return 0;
    return a.cross(b).norm()/a.norm();
}

double distanceAlongNormal(Vector3f pt, Vector3f norm_pt, Vector3f normal)
{
    double d = pointToVectorDistance(pt,normal,norm_pt);
    return sqrt(pow((pt-norm_pt).norm(),2) - d*d);
}

Eigen::Vector4f fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

double pointToPlaneDistance(Eigen::Vector4f plane, Vector3f pt)
{
    return fabs(plane(0)*pt(0)+plane(1)*pt(1)+plane(2)*pt(2)+plane(3))/(sqrt(pow(plane(0),2)+pow(plane(1),2)+pow(plane(2),2)));
}

void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroids,pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_bw);
    tree->setInputCloud (cloud_bw);
    for(int i=0;i<centroids->points.size();i++)
    {
        auto pt = centroids->points[i];
        // pcl::PointXYZ ptxyz = pcl::PointXYZ({pt.x,pt.y,pt.z});
        pcl::PointXYZ ptxyz;
        ptxyz.x = pt.x;
        ptxyz.y = pt.y;
        ptxyz.z = pt.z;
        vector<int> indices;
        vector<float> dist;
        auto t=tree->radiusSearch(ptxyz,ball_radius,indices,dist,0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr points( new pcl::PointCloud<pcl::PointXYZ> );
        for(auto ids:indices)
        {
            PointXYZ pt_temp;
            pt_temp.x = cloud->points[ids].x;
            pt_temp.y = cloud->points[ids].y;
            pt_temp.z = cloud->points[ids].z;
            points->points.push_back(pt_temp);
        }
        int good_points = 0;
#if 0
        if(indices.size()>10)
        {
            Vector3f normal = getNormal(points);
            Vector3f norm_pt;
            norm_pt<<pt.x,pt.y,pt.z;
            Vector3f pt_pro;
            pt_pro<<0,0,0;
            for(auto ids:indices)
            {
                PointXYZ pt_temp;
                pt_temp.x = cloud->points[ids].x;
                pt_temp.y = cloud->points[ids].y;
                pt_temp.z = cloud->points[ids].z;
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                auto dist = distanceAlongNormal(pt_loc,norm_pt,normal);
                if(dist>cylinder_radius)
                    continue;
                good_points++;
                pt_pro(0)+=pt_temp.x*(1-dist/ball_radius);
                pt_pro(1)+=pt_temp.y*(1-dist/ball_radius);
                pt_pro(2)+=pt_temp.z*(1-dist/ball_radius);
            }
            if(good_points==0)
            {
                cout<<"No Good Points.."<<endl;
            }
            pt_pro/=good_points;
            PointXYZRGB pt_processed;
            pt_processed.x = pt_pro(0);
            pt_processed.y = pt_pro(1);
            pt_processed.z = pt_pro(2);
            pt_processed.r = 0;
            pt_processed.g = 0;
            pt_processed.b = 0;
           processed->points.push_back(pt_processed); 
        }
#else
    if(indices.size()>10)
        {
            Vector4f plane = fitPlane(points);
            Vector3f pt_pro;
            pt_pro<<0,0,0;
            for(auto ids:indices)
            {
                PointXYZ pt_temp;
                pt_temp.x = cloud->points[ids].x;
                pt_temp.y = cloud->points[ids].y;
                pt_temp.z = cloud->points[ids].z;
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                auto dist = pointToPlaneDistance(plane,pt_loc);
                if(dist>cylinder_radius)
                    continue;
                good_points++;
                pt_pro(0)+=pt_temp.x*(1-dist/cylinder_radius);
                pt_pro(1)+=pt_temp.y*(1-dist/cylinder_radius);
                pt_pro(2)+=pt_temp.z*(1-dist/cylinder_radius);
            }
            if(good_points==0)
            {
                cout<<"No Good Points.."<<endl;
            }
            pt_pro/=good_points;
            PointXYZRGB pt_processed;
            pt_processed.x = pt.x;
            pt_processed.y = pt.y;
            pt_processed.z = pt_pro(2);
            pt_processed.r = 0;
            pt_processed.g = 0;
            pt_processed.b = 0;
           processed->points.push_back(pt_processed); 
        }
#endif
    }
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#if 0
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return;
    }
#else
    pcl::PLYReader Reader;
    Reader.read(string(argv[1]), *cloud);
#endif
    // visualize(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample<pcl::PointXYZRGB>(cloud,cloud_filtered,downsample_radius);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZRGB>);
    process(cloud,cloud_filtered,cloud_processed);
    visualize(cloud_filtered);
    visualize(cloud_processed);
    return 0;
}
