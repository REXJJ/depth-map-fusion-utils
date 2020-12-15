#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <omp.h>

#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/search/search.h>

#include <iostream>
#include <DebuggingUtilities.hpp>
#include <PointCloudProcessing.hpp>

using namespace pcl;
using namespace std;
using namespace PointCloudProcessing;

int main(int argc, char**argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string filename = argv[1];
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return 0;
    }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr normals_root(new pcl::PointCloud<pcl::PointXYZ>);
    // downsample<pcl::PointXYZ>(cloud,normals_root,0.005);
    // std::cout<<"Downsampled and filtered points size : "<<normals_root->points.size()<<std::endl;
    std::cout<<"Number of points: "<<cloud->points.size()<<std::endl;
    std::cout<<"Width: "<<cloud->width<<std::endl;
    std::cout<<"Height: "<<cloud->height<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new pcl::PointCloud<pcl::PointXYZ>);
    
    downsample<pcl::PointXYZ>(cloud,cloud_bw,0.005);

    if(cloud->points.size()>0)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_bw);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator;
        normalEstimator.setInputCloud(cloud_bw);
        normalEstimator.setSearchMethod(tree);
        normalEstimator.setRadiusSearch(0.015);
        normalEstimator.useSensorOriginAsViewPoint();
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        normalEstimator.compute(*cloud_normals);
        std::cout<<"CPU processing done.."<<std::endl;
    }
    tic();
    pcl::gpu::NormalEstimation::PointCloud cloud_device;
    toc();
    cloud_device.upload(cloud_bw->points);
    toc();

    //pcl::gpu::NormalEstimation::PointCloud surface_device;
    //surface_device.upload(source.surface->points);

    //pcl::gpu::NormalEstimation::Indices indices_device;
    //indices_device.upload(source.indices);

    pcl::gpu::NormalEstimation ne_device;
    toc();
    ne_device.setInputCloud(cloud_device);
    toc();
    ne_device.setRadiusSearch(0.05,10000);
    toc();
    //ne_device.setSearchSurface(surface_device);
    //ne_device.setIndices(indices_device);

    pcl::gpu::NormalEstimation::Normals normals_device;
    toc();
    ne_device.compute(normals_device);
    toc();

    std::vector<PointXYZ> downloaded;
    toc();
    normals_device.download(downloaded);
    toc();
    std::cout<<"GPU Processing done.."<<std::endl;
    return 0;

}
