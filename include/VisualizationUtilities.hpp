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
#include <Camera.hpp>

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
            viewer_->setBackgroundColor (1, 1, 1);
            // viewer_->setBackgroundColor (0, 0, 0);
            viewer_->initCameraParameters ();
        }
        template<typename PointT> void addPointCloud(typename PointCloud<PointT>::Ptr cloud);
        template<typename PointT> void addPointCloudNormals(typename PointCloud<PointT>::Ptr cloud,PointCloud<Normal>::Ptr normals);
        bool spinViewerOnce();
        void spinViewer();
        void clear();
        void addSphere(PointXYZ pt,string id);
        void addCoordinateSystem();
        void addVolume(VoxelVolume &volume);
        void addPointCloudInVolume(VoxelVolume &volume);
        void addLine(vector<double>& start,vector<double>& end,string id);
        void addPolygon(vector<vector<double>>& points,string id);
        void addPyramid(vector<vector<double>>& polygon,vector<double> origin,string id);
        void addNewCoordinateAxes(Eigen::Affine3f& transformation,string id);
        void addCamera(vector<float>& K,int height,int width,Eigen::Affine3f& transformation,string id,int zdepth);
        void addCamera(Camera& cam,Eigen::Affine3f& transformation,string id,int zdepth);
        void addPointCloudInVolumeRayTraced(VoxelVolume &volume);
        void addVolumeWithVoxelsClassified(VoxelVolume &volume);
    };

    template<typename PointT> void PCLVisualizerWrapper::addPointCloud(typename PointCloud<PointT>::Ptr cloud)
    {
        viewer_->addPointCloud<PointT>(cloud);
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

    void PCLVisualizerWrapper::addSphere(PointXYZ pt,string id="sphere")
    {
        viewer_->addSphere (pt, 0.01, 0.5, 0.5, 0.0, id);
    }
    void PCLVisualizerWrapper::addCoordinateSystem()
    {
        viewer_->addCoordinateSystem (1.0);
    }

    void PCLVisualizerWrapper::addVolume(VoxelVolume &volume)
    {
        int counter = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
            for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
                for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
                {
                    auto coords = volume.getVoxel(x,y,z);
                    pcl::PointXYZRGB pt;
                    pt.x=x;
                    pt.y=y;
                    pt.z=z;
                    pt.r=255;
                    pt.g=255;
                    pt.b=255;
                    if(volume.validPoints(x,y,z)==false)
                        continue;
                    if(volume.voxels_[get<0>(coords)][get<1>(coords)][get<2>(coords)]!=nullptr)
                    {
                        pt.r=0;
                        pt.b=0;
                    }
                    cloud->points.push_back(pt);
                }
        viewer_->addPointCloud<pcl::PointXYZRGB>(cloud,"volume");
    }
    
    void PCLVisualizerWrapper::addPointCloudInVolume(VoxelVolume &volume)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
            for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
                for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
                {
                    auto coords = volume.getVoxel(x,y,z);
                    pcl::PointXYZRGB pt;
                    pt.x=x;
                    pt.y=y;
                    pt.z=z;
                    pt.r=255;
                    pt.g=255;
                    pt.b=255;
                    if(volume.voxels_[get<0>(coords)][get<1>(coords)][get<2>(coords)]==nullptr)
                        continue;
                    cloud->points.push_back(pt);
                }
        viewer_->addPointCloud<pcl::PointXYZRGB>(cloud,"volume");
    }
    
    void PCLVisualizerWrapper::addLine(vector<double>& start,vector<double>& end,string id="line")
    {
        pcl::PointXYZ pt1,pt2;
        pt1.x = start[0];
        pt1.y = start[1];
        pt1.z = start[2];
        pt2.x = end[0];
        pt2.y = end[1];
        pt2.z = end[2];
        viewer_->addLine<pcl::PointXYZ> (pt1,pt2,id);
    }

    void PCLVisualizerWrapper::addPolygon(vector<vector<double>>& points,string id="polygon")
    {
        for(int i=0;i<points.size()-1;i++)
            addLine(points[i],points[i+1],id+"line"+to_string(i)+to_string(i+1));
        addLine(points[points.size()-1],points[0],id+"linelast0");
    }

    void PCLVisualizerWrapper::addPyramid(vector<vector<double>>& polygon,vector<double> origin,string id="pyramid")
    {
        addPolygon(polygon,id);
        for(int i=0;i<polygon.size();i++)
            addLine(polygon[i],origin,id+"originLine"+to_string(i));
    }

    void PCLVisualizerWrapper::addNewCoordinateAxes(Eigen::Affine3f& transformation,string id="new_reference")
    {
        viewer_->addCoordinateSystem (0.05,transformation,id);
    }

    void PCLVisualizerWrapper::addCamera(vector<float>& K,int height,int width,Eigen::Affine3f& transformation,string camera_name="camera",int zdepth=30)
    {
        Camera cam(K);
        double x,y,z;
        vector<vector<double>> polygon;
        std::tie(x,y,z)=cam.getPoint(0,0,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.getPoint(0,width,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.getPoint(height,width,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.getPoint(height,0,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.transformPoints(0,0,0,transformation);
        addPyramid(polygon,{x,y,z},camera_name);
        addNewCoordinateAxes(transformation,camera_name);
    }
    
    void PCLVisualizerWrapper::addCamera(Camera& cam,Eigen::Affine3f& transformation,string camera_name="camera",int zdepth=30)
    {
        int width = cam.getWidth();
        int height = cam.getHeight();
        double x,y,z;
        vector<vector<double>> polygon;
        std::tie(x,y,z)=cam.getPoint(0,0,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.getPoint(0,width,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.getPoint(height,width,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.getPoint(height,0,zdepth);
        std::tie(x,y,z)=cam.transformPoints(x,y,z,transformation);
        polygon.push_back({x,y,z});
        std::tie(x,y,z)=cam.transformPoints(0,0,0,transformation);
        addPyramid(polygon,{x,y,z},camera_name);
        addNewCoordinateAxes(transformation,camera_name);
    }
    void PCLVisualizerWrapper::addPointCloudInVolumeRayTraced(VoxelVolume &volume)
    {
        unsigned long long int no_of_points = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
            for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
                for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
                {
                    auto coords = volume.getVoxel(x,y,z);
                    pcl::PointXYZRGB pt;
                    pt.x=x;
                    pt.y=y;
                    pt.z=z;
                    pt.r=255;
                    Voxel *voxel = volume.voxels_[get<0>(coords)][get<1>(coords)][get<2>(coords)];
                    if(voxel==nullptr)
                        continue;
                    if(voxel->view)
                    {
                        pt.r=0;
                        pt.g=255;
                    }
                    cloud->points.push_back(pt);
                    no_of_points++;
                }
        cloud->height = 1;
        cloud->width = no_of_points;
        pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
        viewer_->addPointCloud<pcl::PointXYZRGB>(cloud,"volume");
    }

    void PCLVisualizerWrapper::addVolumeWithVoxelsClassified(VoxelVolume &volume)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
            for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
                for(float z=volume.zmin_;z<volume.zmax_;z+=volume.zdelta_)
                {
                    auto coords = volume.getVoxel(x,y,z);
                    pcl::PointXYZRGB pt;
                    pt.x=x;
                    pt.y=y;
                    pt.z=z;
                    pt.r=255;
                    pt.g=255;
                    pt.b=255;
                    Voxel *voxel = volume.voxels_[get<0>(coords)][get<1>(coords)][get<2>(coords)];
                    if(voxel==nullptr)
                        continue;
                    if(voxel->view)
                    {
                        pt.g = 0;
                        pt.b = 0;
                        if(voxel->good)
                        {
                            pt.g = 255;
                            pt.r=0;
                            pt.b=0;
                        }
                    }
                    cloud->points.push_back(pt);
                }
        viewer_->addPointCloud<pcl::PointXYZRGB>(cloud,"volume");
    }
}


