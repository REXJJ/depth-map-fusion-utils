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
/*********************************************/
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
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
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/obj_io.h>

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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;


inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
{
	Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
	Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
	data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
	data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
	Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
	Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
	transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
	return transformed_data_mat.transpose();
}

pcl::PointCloud<pcl::PointXYZRGB> transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,Eigen::MatrixXd& a_T_b)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	for(auto point:point_cloud->points)
	{
		Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
		pts(0,0)=point.x;
		pts(0,1)=point.y;
		pts(0,2)=point.z;
		Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
		pcl::PointXYZRGB pt;
		pt.x=pts_trans(0,0);
		pt.y=pts_trans(0,1);
		pt.z=pts_trans(0,2);
		pt.rgb=point.rgb;
		cloud.push_back(pt);
	}
	return cloud;
}

MatrixXd parseTransforms(string filename)
{
	ifstream t(filename);
	MatrixXd mat=MatrixXd::Zero(4,4);
	string line;
	int count=0;
	while(getline(t,line))
	{
		vector<string> v;
		split(v,line,boost::is_any_of(","));
		for(int i=0;i<v.size();i++)
			mat(count,i)=stof(v[i]);
		count++;
	}
	return mat;
}

PointCloud<PointXYZRGB> mergeFromPointClouds(string directory,int positions)
{
	PointCloud<PointXYZRGB> cloud_merged;
	for(int i=0;i<positions;i+=1)
	{
		cout<<i<<endl;
		string temp="/home/rex/REX_WS/Test_WS/OTHERS/rex_data/pointclouds/cloud";
		string input_pcd=temp+to_string(i)+"0.pcd";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_pcd, *cloud_ptr) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return cloud_merged;
		}
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		for(int i=0;i<cloud_ptr->points.size();i++)
		{
			if(cloud_ptr->points[i].z>1.20)
				continue;
			auto pt = cloud_ptr->points[i];
			if(pt.x==pt.x&&pt.y==pt.y&&pt.z==pt.z)
				cloud.push_back(cloud_ptr->points[i]);
		}
		MatrixXd rob_T_cam_local;
		rob_T_cam_local.resize(4,4);
		rob_T_cam_local=parseTransforms("/home/rex/REX_WS/Test_WS/OTHERS/rex_data/transforms/pointcloud_transforms"+to_string(i)+".txt");
		cout<<rob_T_cam_local<<endl;
		PointCloud<PointXYZRGB> cloud_new = transformPointCloud(cloud.makeShared(),rob_T_cam_local);
		cloud_merged=cloud_merged+cloud_new;
	}

	PointCloud<PointXYZRGB> cloud_filtered;
	int count=0;
	for(auto pt:cloud_merged.points)
	{
		//	if(pt.x<-1.0||pt.x>-0.50||pt.y<-0.5||pt.y>0.1||pt.z<0||pt.z>0.2)
		//		continue;
		cloud_filtered.points.push_back(pt);
		count++;
	}
	cloud_filtered.height=1;
	cloud_filtered.width=count;
	return cloud_filtered;
}

int main(int argc, char** argv)
{
	string directory = "/home/rex/REX_WS/Test_WS/OTHERS/rex_data/pointclouds";
	string output_pcd_file_name="/home/rex/REX_WS/Test_WS/OTHERS/merged_point_cloud.pcd";
	PointCloud<PointXYZRGB> fused_point_cloud = mergeFromPointClouds(directory,5);
	pcl::io::savePCDFileASCII (output_pcd_file_name, fused_point_cloud);
	return 0;
}
