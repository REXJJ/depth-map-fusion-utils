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

// std::pair<Vector3f, Vector3f> best_plane_from_points(const std::vector<Vector3f> & c)
// {
// 	// copy coordinates to  matrix in Eigen format
// 	size_t num_atoms = c.size();
// 	Eigen::Matrix< Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
// 	for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];
//
// 	// calculate centroid
// 	Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());
//
// 	// subtract centroid
// 	coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);
//
// 	// we only need the left-singular matrix here
// 	//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
// 	auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
// 	Vector3f plane_normal = svd.matrixU().rightCols<1>();
// 	return std::make_pair(centroid, plane_normal);
// }

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
    vector<Vector3f> pts;
    for(int i=0;i<10;i++)
    {
        pcl::PointXYZ ptn;
        ptn.x = cloud->points[i].x;
        ptn.y = cloud->points[i].y;
        ptn.z = cloud->points[i].z;
        rays->points.push_back(ptn);
        Vector3f pt(3);
        pt<<ptn.x,ptn.y,ptn.z;
        pts.push_back(pt);
    }
    
    auto[centroid,plane_normal] = best_plane_from_points(pts);

    pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

    center->points.push_back({centroid(0),centroid(1),centroid(2)});
    pcl::Normal n;
    n.normal[0] = plane_normal(0);
    n.normal[1] = plane_normal(1);
    n.normal[2] = plane_normal(2);
    normal->points.push_back(n);



    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(rays);
    viewer->addPointCloudNormals<PointXYZ,pcl::Normal>(center, normal);
    while(!viewer->wasStopped())
            viewer->spinOnce(100);
    return 0;
}
