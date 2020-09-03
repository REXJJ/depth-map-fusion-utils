#pragma once
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

/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>

#include <Volume.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace VisualizationUtilities
{
    class PCLVisualizerWrapper
    {
        pcl::visualization::PCLVisualizer::Ptr viewer_;
        public:
        PCLVisualizerWrapper()
        {
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer_=viewer;
            viewer_->setBackgroundColor (0, 0, 0);
            viewer_->initCameraParameters ();
        }
        template<typename PointT> void addPointCloud(typename PointCloud<PointT>::Ptr cloud);
        template<typename PointT> void addPointCloudNormals(typename PointCloud<PointT>::Ptr cloud,PointCloud<Normal>::Ptr normals);
        bool spinViewerOnce();
        void spinViewer();
        void clear();
        void addSphere(PointXYZ pt);
        void addCoordinateSystem();
        void addVolume(VoxelVolume &volume);
    };

    template<typename PointT> void PCLVisualizerWrapper::addPointCloud(typename PointCloud<PointT>::Ptr cloud)
    {
        viewer_->addPointCloud<PointT>(cloud,"sample cloud");
    }
    bool PCLVisualizerWrapper::spinViewerOnce()
    {
        if(viewer_->wasStopped())
            return false;
        viewer_->spinOnce(100);
        return true;
    }
    void PCLVisualizerWrapper::spinViewer()
    {
        while(!viewer_->wasStopped())
            viewer_->spinOnce(100);
    }

    void PCLVisualizerWrapper::clear()
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes(); 
    }

    template<typename PointT> void PCLVisualizerWrapper::addPointCloudNormals(typename PointCloud<PointT>::Ptr cloud,PointCloud<Normal>::Ptr normals)
    {
        viewer_->addPointCloudNormals<PointT,pcl::Normal>(cloud, normals);
    }

    void PCLVisualizerWrapper::addSphere(PointXYZ pt)
    {
        viewer_->addSphere (pt, 0.01, 0.5, 0.5, 0.0, "sphere");
    }
    void PCLVisualizerWrapper::addCoordinateSystem()
    {
        viewer_->addCoordinateSystem (1.0);
    }

    void PCLVisualizerWrapper::addVolume(VoxelVolume &volume)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
            for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
                for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
                    cloud->points.push_back({x,y,z});
        viewer_->addPointCloud(cloud);
    }
}


