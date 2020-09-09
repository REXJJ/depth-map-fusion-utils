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
#include<DebuggingUtilities.hpp>
#include<TransformationUtilities.hpp>
#include<Camera.hpp>
#include<RayTracingEngine.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace PointCloudProcessing
{
    void makePointCloudNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,vector<double> viewpoint={0.0,0.0,0.0})
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw (new PointCloud<pcl::PointXYZ>);
        for(int i=0;i<cloud->points.size();i++)
        {
            PointXYZ ptxyz;
            PointXYZRGB ptxyzrgb = cloud->points[i];
            ptxyz.x=ptxyzrgb.x;
            ptxyz.y=ptxyzrgb.y;
            ptxyz.z=ptxyzrgb.z;
            cloud_bw->points.push_back(ptxyz);
        }
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_bw);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.005);
        ne.setViewPoint(viewpoint[0],viewpoint[1],viewpoint[2]);
        ne.compute (*normals); 
    }
}

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
    constexpr double PI = 3.141592;
    // vector<double> two_T_one_static = {0,0,0,0,0,PI/6};
    vector<double> two_T_one_static = {0,-0.6,+0.4,0,0,-PI/3};
    // vector<double> two_T_one_static = {0.6,0,0.5,0,-PI/2,0};
	Eigen::MatrixXd two_T_one = vectorToTransformationMatrix(two_T_one_static);
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            transformation(i,j)=two_T_one(i,j);
    Camera cam(K);

    RayTracingEngine engine(cam);
    //
    // VoxelVolume volume;
    // volume.setDimensions(-0.5,0.5,-0.5,0.5,0,1);
    // volume.setVolumeSize(50,50,50);
    // volume.constructVolume();
    // volume.integratePointCloud(cloud);
    //
    // engine.rayTrace(volume,transformation,false);
    //
    // VisualizationUtilities::PCLVisualizerWrapper viz;
    // viz.addCoordinateSystem();
    // viz.addPointCloudInVolumeRayTraced(volume);
    // viz.addCamera(cam,transformation,"camera",1000);
    // viz.spinViewer();

    /*****************************************************************************/


    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    PointCloudProcessing::makePointCloudNormal(cloud,normals);

    VoxelVolume volume2;
    volume2.setDimensions(-0.5,0.5,-0.5,0.5,0,1);
    volume2.setVolumeSize(50,50,50);
    volume2.constructVolume();
    volume2.integratePointCloud(cloud,normals);
    engine.rayTraceAndClassify(volume2,transformation,false);

    VisualizationUtilities::PCLVisualizerWrapper viz2;
    viz2.addCoordinateSystem();
    viz2.addVolumeWithVoxelsClassified(volume2);
    viz2.addCamera(cam,transformation,"camera",1000);
    viz2.spinViewer();

    return 0;
}
