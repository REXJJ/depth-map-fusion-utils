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

#include<Volume.hpp>
#include<VisualizationUtilities.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace DebuggingUtilities
{
    void print(){std::cout<<std::endl;}
    template<typename T,typename... Args>
        void print(T Contents, Args... args) 
        {
            std::cout<< (Contents) <<" ";print(args...);
        }
}

class RayTracingEngine
{
    public:
        int height_,width_;
        float fx_,fy_,cx_,cy_;
        RayTracingEngine(int height,int width,float fx,float fy,float cx,float cy);
        tuple<float,float,float> projectPoint(int r,int c,int z);
};

RayTracingEngine::RayTracingEngine(int height,int width,float fx,float fy,float cx,float cy)
    :height_(height),
    width_(width),
    fx_(fx),
    fy_(fy),
    cx_(cx),
    cy_(cy){}

tuple<float,float,float> RayTracingEngine::projectPoint(int r,int c,int depth_mm)
{
    float x,y,z;
    z = depth_mm * 0.001;
    x = z * ( (double)c - cx_ ) / (fx_);
    y = z * ((double)r - cy_ ) / (fy_);
    return {x,y,z};
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
#if 0
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("/home/rex/REX_WS/Catkin_WS/data/base.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
#else
    pcl::PLYReader Reader;
    Reader.read("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/empty_box_inside/"+string(argv[1])+".ply", *cloud);
#endif
    for(auto &pt:cloud->points)
        pt.z*=-1;
#if 0
    VoxelVolume volume;
    volume.setDimensions(-1,1,-1,1,0,1);
    volume.setVolumeSize(50,50,50);
    volume.constructVolume();
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addVolume(volume);
    viz.addPointCloud<PointXYZRGB>(cloud);
    PointXYZ ptorigin = PointXYZ(0,0,0);
    viz.addSphere(ptorigin);
    viz.spinViewer();
#else
    pcl::PointCloud<pcl::PointXYZ>::Ptr rays (new pcl::PointCloud<pcl::PointXYZ>);
    int height = 480,width = 640;
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    double fx=K[0],cx=K[2],fy=K[4],cy=K[5];
    RayTracingEngine engine(height,width,fx,fy,cx,cy);
    for(int i=0;i<100;i++)
        for(int r=0;r<height;r+=20)
        { 
            for(int c=0;c<width;c+=20)
            {   
                auto [x,y,z] = engine.projectPoint(r,c,10+i*10);
                rays->points.push_back({x,y,z});
            }
        }
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addPointCloud<pcl::PointXYZ>(rays);
    viz.addCoordinateSystem();
    viz.spinViewer();
#endif
    return 0;
}
