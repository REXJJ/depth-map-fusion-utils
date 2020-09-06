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
#include<RayTracingEngine.hpp>
#include<DebuggingUtilities.hpp>
#include<TransformationUtilities.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;


int main(int argc, char** argv)
{
    using namespace TransformationUtilities;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr rays (new pcl::PointCloud<pcl::PointXYZ>);
    int height = 480,width = 640;
    vector<float> K = {602.39306640625, 0.0, 314.6370849609375, 0.0, 602.39306640625, 245.04962158203125, 0.0, 0.0, 1.0};
    double fx=K[0],cx=K[2],fy=K[4],cy=K[5];
    RayTracingEngine engine(height,width,fx,fy,cx,cy);
    for(int r=0;r<height;r++)
    { 
        for(int c=0;c<width;c++)
        {   
            auto [x,y,z] = engine.projectPoint(r,c,1000);
            rays->points.push_back({x,y,z});
        }
    }
    double x,y,z;
    vector<vector<double>> polygon;
    std::tie(x,y,z)=engine.projectPoint(0,0,1000);
    polygon.push_back({x,y,z});
    std::tie(x,y,z)=engine.projectPoint(0,640,1000);
    polygon.push_back({x,y,z});
    std::tie(x,y,z)=engine.projectPoint(480,640,1000);
    polygon.push_back({x,y,z});
    std::tie(x,y,z)=engine.projectPoint(480,0,1000);
    polygon.push_back({x,y,z});

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (rays);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.005);
    ne.setViewPoint(0,0,0);
    ne.compute (*normals); 
    constexpr double PI = 3.141592;
    vector<double> two_T_one_static = {0,0.5,0,0,PI/4,0};
	Eigen::MatrixXd two_T_one = vectorToTransformationMatrix(two_T_one_static);

    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            transformation(i,j)=two_T_one(i,j);

    pcl::PointCloud<pcl::PointXYZ>::Ptr rays_transformed (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::Normal>::Ptr normals_transformed (new pcl::PointCloud<pcl::Normal>);
    transformPointCloud<pcl::PointXYZ>(rays,rays_transformed,two_T_one);
    transformPointCloudNormal(normals,normals_transformed,two_T_one);

    VisualizationUtilities::PCLVisualizerWrapper viz;
    // viz.addPointCloudNormals<pcl::PointXYZ>(rays,normals);
    // viz.addPointCloudNormals<pcl::PointXYZ>(rays_transformed,normals_transformed);
    // viz.addPolygon(polygon);
    // viz.addPyramid(polygon,{0,0,0});
    viz.addCoordinateSystem();
    // viz.addNewCoordinateAxes(transformation);
    viz.addCamera(K,480,640,transformation,"camera");
    viz.spinViewer();
    return 0;
}
