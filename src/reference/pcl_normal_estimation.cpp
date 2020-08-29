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

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

template <typename PointT> static void visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<PointT> (cloud.makeShared(), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	// viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);

	}
}

template <typename PointT> static pcl::PointCloud<PointT> downsample(pcl::PointCloud<PointT> cloud,double leaf)
{
	pcl::VoxelGrid<PointT> sor;
	pcl::PointCloud<PointT> cloud_filtered;
	sor.setLeafSize (leaf, leaf, leaf);
	sor.setInputCloud (cloud.makeShared());
	sor.filter (cloud_filtered);
	return cloud_filtered;
}

void print(){std::cout<<std::endl;}
	template<typename T,typename... Args>
void print(T Contents, Args... args) 
{
	std::cout<< (Contents) <<" ";print(args...);
}


void pclXYZRGBNormalToPly(const std::string filename,pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
{
	ofstream f(filename);
	string header="property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n";
	f<<"ply\nformat ascii 1.0\n";
	f<<"element vertex ";
	f<<(to_string(cloud.points.size())+"\n");
	f<<header;
	for(auto pt:cloud.points)
	{
		f<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<static_cast<int>(pt.r)<<" "<<static_cast<int>(pt.g)<<" "<<static_cast<int>(pt.b)<<" "<<pt.normal[0]<<" "<<pt.normal[1]<<" "<<pt.normal[2]<<"\n";
	}
	f.close();
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

PointCloud<PointXYZRGBNormal> flipNormals(PointCloud<PointXYZRGBNormal>& cloud)
{
	PointCloud<PointXYZRGBNormal> output;
	for(auto point:cloud.points)
	{
		PointXYZRGBNormal pt;
		pt.x=point.x;
		pt.y=point.y;
		pt.z=point.z;
		pt.normal[0]=-1*point.normal[0];
		pt.normal[1]=-1*point.normal[1];
		pt.normal[2]=-1*point.normal[2];
		output.points.push_back(pt);
	}
	output.height=1;
	output.width=cloud.points.size();
	return output;
}

PointCloud<PointXYZRGB> generateSphere(double radius)
{
	PointCloud<PointXYZRGB> output;
	for(double r=radius;r>=0;r-=0.001)
	{
		for(double x=-r;x<=r;x+=0.001)
		{
			double y=sqrt(r*r-(x*x));
			PointXYZRGB pt;
			pt.x=x;
			pt.y=y;
			pt.z=sqrt(radius*radius-(x*x)-(y*y));
			pt.r=0;
			pt.g=0;
			pt.b=255;
			output.points.push_back(pt);
			pt.y=-y;
			output.points.push_back(pt);
			pt.z=-pt.z;
			output.points.push_back(pt);
			pt.y=y;
			output.points.push_back(pt);
		}
	}
	PointCloud<PointXYZRGB> valid;
	int count=0;
	for(auto pt:output.points)
	{
		if(pt.x==pt.x&&pt.y==pt.y&&pt.z==pt.z)
		{
			valid.points.push_back(pt);
			count++;
		}
	}
	valid.height=1;
	valid.width=count;
	return valid;
}


int main(int argc, char** argv)
{

	// string file_name="/home/rex/REX_WS/Test_WS/OTHERS/outputs/merged_new_blurred.pcd";
	// string file_name="../after_gaussian.pcd";
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	// if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (file_name, *cloud) == -1) //* load the file
	// {
	// 	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	// 	return (-1);
	// }
	// pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
	// pcl::PointCloud<pcl::PointXYZ> cloud_bw;
	// for(auto pt:cloud->points)
	// {
	// 	pcl::PointXYZRGB ptr;
	// 	pcl::PointXYZ ptn;
	// 	ptr.x=ptn.x=pt.x;
	// 	ptr.y=ptn.y=pt.y;
	// 	ptr.z=ptn.z=pt.z;
	// 	ptr.rgb=pt.rgb;
	// 	cloud_bw.points.push_back(ptn);
	// 	cloud_color.points.push_back(ptr);
	// }
	// double x_min=1000,x_max=-1000.0,y_min=1000,y_max=-1000.0,z_min=1000,z_max=-1000;
	// for(auto pt:cloud_bw.points)
	// {
	// 	if(pt.x<x_min)
	// 		x_min=pt.x;
	// 	if(pt.y<y_min)
	// 		y_min=pt.y;
	// 	if(pt.x>x_max)
	// 		x_max=pt.x;
	// 	if(pt.y>y_max)
	// 		y_max=pt.y;
	// 	if(pt.z>z_max)
	// 		z_max=pt.z;
	// 	if(pt.z<z_min)
	// 		z_min=pt.z;
	// }
	// print(x_min,x_max,y_min,y_max,z_min,z_max);
    //
	// double vx=(x_min+x_max)/2.0,vy=(y_min+y_max)/2.0,vz=+10000.0;
    //
	// vector<double> viewpoint={vx,vy,vz};
    //
	// PointCloud<PointXYZRGBNormal> cloud_with_normals=makePointCloudNormal(cloud_color,viewpoint);

	// cloud_with_normals=flipNormals(cloud_with_normals);
	//
	PointCloud<PointXYZRGB> circle = generateSphere(0.2);

	vector<double> vp={0,0,0};
	auto cloud_circle=makePointCloudNormal(circle,vp);

	string output_pcd_file_name="/home/rex/REX_WS/Test_WS/OTHERS/outputs/after_gaussian_with_normals.pcd";

	// pcl::io::savePCDFileASCII(output_pcd_file_name,cloud_with_normals);

	string output_ply_file_name="/home/rex/REX_WS/Test_WS/OTHERS/outputs/after_gaussian_with_normals_new.ply";

	pclXYZRGBNormalToPly(output_ply_file_name,cloud_circle);

	return 0;
}
