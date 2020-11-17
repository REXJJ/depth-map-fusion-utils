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

Vector3f projectPointToVector(Vector3f pt, Vector3f norm_pt, Vector3f n)
{
    Vector3f d_xyz = n*ball_radius;
    Vector3f a = norm_pt - d_xyz;
    Vector3f b = norm_pt + d_xyz;
    Vector3f ap = (a-pt);
    Vector3f ab = (a-b);
    Vector3f p = a - (ap.dot(ab)/ab.dot(ab))*ab;
    return p;
}

void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroids,pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_bw);
    tree->setInputCloud (cloud_bw);
    unsigned long long int good_centers = 0;
    for(int i=0;i<centroids->points.size();i++)
    {
        if(i%100==0)
        {
            std::cout<<i<<" out of "<<centroids->points.size()<<" done."<<std::endl;
        }
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
#if 1
        if(indices.size()>10)
        {
            Vector3f normal = getNormal(points);
            Vector3f norm_pt;
            norm_pt<<pt.x,pt.y,pt.z;
            Vector3f pt_pro;
            pt_pro<<0,0,0;
            double weights = 0.0;
            for(auto ids:indices)
            {
                PointXYZ pt_temp;
                pt_temp.x = cloud->points[ids].x;
                pt_temp.y = cloud->points[ids].y;
                pt_temp.z = cloud->points[ids].z;
                Vector3f pt_loc;
                pt_loc<<pt_temp.x,pt_temp.y,pt_temp.z;
                Vector3f projected_points = projectPointToVector(pt_loc, norm_pt,normal);
                double distance_to_normal = (pt_loc - projected_points).norm();
                // std::cout<<"Distance To Normal: "<<distance_to_normal<<" Distance to normal Pt: "<<(pt_loc-norm_pt).norm()<<std::endl;
                if(distance_to_normal<cylinder_radius)
                {
                    pt_pro+= (1.0-distance_to_normal/cylinder_radius) * projected_points;
                    weights += (1.0-distance_to_normal/cylinder_radius);
                    // cout<<"Points: "<<pt_loc<<" Projected Points: "<<projected_points<<" Normal: "<<normal<<endl;
                    good_points++;

                }
            }
            if(good_points==0)
            {
                cout<<"No Good Points.."<<endl;
                continue;
            }
            pt_pro/=weights;
            PointXYZRGB pt_processed;
            pt_processed.x = pt_pro(0);
            pt_processed.y = pt_pro(1);
            pt_processed.z = pt_pro(2);
            pt_processed.r = pt.r;
            pt_processed.g = pt.g;
            pt_processed.b = pt.b;
            good_centers++;
            processed->points.push_back(pt_processed); 
        }
#endif
    }
    processed->height = 1;
    processed->width = good_centers;
}


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#if 1
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return -1;
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
