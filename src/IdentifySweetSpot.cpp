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

#include <VisualizationUtilities.hpp>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

namespace PointCloudProcessing
{
    template <typename PointT> static pcl::PointCloud<PointT> downsample(pcl::PointCloud<PointT> cloud,double leaf)
    {
        pcl::VoxelGrid<PointT> sor;
        pcl::PointCloud<PointT> cloud_filtered;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.setInputCloud (cloud.makeShared());
        sor.filter (cloud_filtered);
        return cloud_filtered;
    }
    PointCloud<PointXYZRGBNormal> makePointCloudNormal(PointCloud<PointXYZRGB>& cloud,vector<double>& viewpoint)
    {
        pcl::PointCloud<PointXYZ> cloud_xyz;
        for(int i=0;i<cloud.points.size();i++)
        {
            cloud_xyz.points.push_back({cloud.points[i].x,cloud.points[i].y,cloud.points[i].z});
        }
        PointCloud<PointXYZ>::Ptr cloud_in = cloud_xyz.makeShared();
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_in);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (0.005);
        ne.setViewPoint(viewpoint[0],viewpoint[1],viewpoint[2]);
        ne.compute (*cloud_normals); 
        PointCloud<PointXYZRGBNormal> output_cloud;
        for(int i=0;i<cloud.points.size();i++)
        {
            PointXYZRGBNormal pt;
            pt.x=cloud.points[i].x;
            pt.y=cloud.points[i].y;
            pt.z=cloud.points[i].z;
            pt.rgb=cloud.points[i].rgb;
            pt.normal[0]=cloud_normals->points[i].normal[0];
            pt.normal[1]=cloud_normals->points[i].normal[1];
            pt.normal[2]=cloud_normals->points[i].normal[2];
            output_cloud.push_back(pt);
        }
        output_cloud.height=1;
        output_cloud.width=cloud.points.size();
        return output_cloud;
    }
}

namespace DebuggingUtilities
{
    void print(){std::cout<<std::endl;}
    template<typename T,typename... Args>
        void print(T Contents, Args... args) 
        {
            std::cout<< (Contents) <<" ";print(args...);
        }
}

constexpr int degree(double radian){return int((radian*180)/3.14159);};
constexpr double magnitude(float normal[3]){return normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];};
constexpr int angle(float normal[3]){return degree(acos(-normal[2]/magnitude(normal)));};

void computeNormalAnglesRange(PointCloud<Normal>::Ptr normals)
{
    int counter=0;
    using namespace DebuggingUtilities;
    int mn=INT_MAX,mx=INT_MIN;
    for(int i=0;i<normals->points.size();i++)
    {
        Normal pt = normals->points[i];
        int angle_z = angle(pt.normal);
        if(pt.normal[0]!=pt.normal[0])
            counter++;
        if(pt.normal[0]==pt.normal[0]&&pt.normal[1]==pt.normal[1]&&pt.normal[2]==pt.normal[2])
        {
            mn=min(mn,angle_z);
            mx=max(mx,angle_z);
        }
    }
    print(counter,mn,mx);
}

int main(int argc, char** argv)
{
    using namespace DebuggingUtilities;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
#if 0
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("/home/rex/REX_WS/Catkin_WS/data/base.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file for base. \n");
        return (-1);
    }
#else
    pcl::PLYReader Reader;
    Reader.read("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/reference_cam_calibration_outside_box/"+string(argv[1])+".ply", *cloud);
#endif
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);
    int counter = 0;
    for(auto pt:cloud->points)
    {
        if(pt.z*-1>1.0)
            continue;
        cloud_output->points.push_back(pt);
        counter++;
    }
    cloud_output->height=1;
    cloud_output->width = counter;
    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addPointCloud<PointXYZRGB>(cloud_output);
    PointXYZ ptorigin = PointXYZ(0,0,0);
    viz.addSphere(ptorigin);
    viz.addCoordinateSystem();
    viz.spinViewer();
    pcl::io::savePCDFileASCII ("cleaned"+string(argv[1])+".pcd", *cloud_output);
    // PointCloud<PointXYZ>::Ptr cloud_bw (new PointCloud<PointXYZ>);
    // for(int i=0;i<cloud->points.size();i++)
    // {
    //     PointXYZ ptxyz;
    //     PointXYZRGB ptxyzrgb = cloud->points[i];
    //     ptxyz.x=ptxyzrgb.x;
    //     ptxyz.y=ptxyzrgb.y;
    //     ptxyz.z=ptxyzrgb.z;
    //     cloud_bw->points.push_back(ptxyz);
    // }
    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // ne.setInputCloud (cloud_bw);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    // ne.setSearchMethod (tree);
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // ne.setRadiusSearch (0.005);
    // ne.setViewPoint(0,0,0);
    // ne.compute (*normals); 
    //
    // PointCloud<PointXYZRGB>::Ptr cloud_classified (new PointCloud<PointXYZRGB>);
    //
    // for(int i=0;i<normals->points.size();i++)
    // {
    //     Normal ptn = normals->points[i];
    //     int angle_z = angle(ptn.normal);
    //     PointXYZ pt = cloud_bw->points[i];
    //     PointXYZRGB pt_new;
    //     pt_new.x = pt.x;
    //     pt_new.y = pt.y;
    //     pt_new.z = pt.z;
    //     pt_new.g = 255;
    //     pt_new.r = 0;
    //     pt_new.b = 0;
    //     if(angle_z>=0&&angle_z<=45)
    //     {
    //         pt_new.b = 0;
    //         pt_new.r = 255;
    //         pt_new.g = 0;
    //     }
    //     cloud_classified->points.push_back(pt_new);
    // }
    //
    //
    // computeNormalAnglesRange(normals);

    // auto temp = minmax_element(cloud_bw->points.begin(),cloud_bw->points.end(),[](PointXYZ A,PointXYZ B){return A.z<B.z;});
    // cout<<*temp.first<<" "<<*temp.second<<endl;
    return 0;
}
