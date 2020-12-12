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
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <omp.h>

/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <OccupancyGrid.hpp>
#include <PointCloudProcessing.hpp>
#include <DebuggingUtilities.hpp>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace PointCloudProcessing;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string filename = argv[1];
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return 0;
    }
    std::cout<<"File read."<<std::endl;
    tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    downsample<pcl::PointXYZ>(cloud,cloud_temp,0.01);
    std::cout<<"Downsampled.."<<std::endl;
    toc();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_temp);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud_temp);
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setRadiusSearch(0.02);
    normalEstimator.setViewPoint(0,0,1);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    normalEstimator.compute(*cloud_normals);
    std::cout<<"Normal Estimation done."<<std::endl;
    toc();

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_output(new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals_output->points.resize(cloud_normals->points.size());
    #pragma omp parallel for
    for(int i=0;i<cloud_normals->points.size();i++)
    {
        pcl::PointNormal pt;
        pcl::PointXYZ ptxyz = cloud_temp->points[i];
        pcl::Normal ptn = cloud_normals->points[i];
        pt.x = ptxyz.x;
        pt.y = ptxyz.y;
        pt.z = ptxyz.z;
        pt.normal[0] = ptn.normal[0];
        pt.normal[1] = ptn.normal[1];
        pt.normal[2] = ptn.normal[2];
        cloud_normals_output->points[i]=pt;
    }
    std::cout<<"Normals Written."<<std::endl;
    toc();
    OccupancyGrid grid;
    grid.setResolution(0.005,0.005,0.005);
    grid.setDimensions(0,2,-1,1,-1,1);
    grid.construct();
    grid.setK(2);
    std::cout<<"Construction done.."<<std::endl;
    toc();
    std::cout<<"Dimensions of grid: "<<grid.xdim_<<" "<<grid.ydim_<<" "<<grid.zdim_<<std::endl;
    std::cout<<"Size of Int: "<<sizeof(int)<<" Size of Voxel: "<<sizeof(Voxel)<<std::endl;
    std::cout<<"Updating the grids.."<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_clr->points.resize(cloud->points.size());
    int size = cloud->points.size();
    #pragma omp parallel for 
    for(int i=0;i<size;i++)
    {
        auto pt = cloud->points[i];
        pcl::PointXYZRGB point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        cloud_clr->points[i] = point;
    }
    toc();
    grid.updateStates(cloud_clr,cloud_normals_output);
    std::cout<<"States Updated..The slowest code..."<<std::endl;
    toc();
    std::cout<<"Downloading cloud."<<std::endl;
    // grid.downloadHQCloud(cloud_normal);
    grid.downloadReorganizedCloud(cloud_normal,true);
    toc();
    std::cout<<"Saving cloud."<<std::endl;
    pcl::io::savePCDFileASCII ("/home/rex/Desktop/test_processed.pcd",*cloud_normal);
    toc();
    return 0;
}

