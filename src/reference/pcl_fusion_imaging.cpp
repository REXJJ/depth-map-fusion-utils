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


pcl::PointCloud<pcl::PointXYZRGB> makePointCloud(Mat& color_image, Mat& depth_image, Eigen::VectorXd& K, std::string frame_id)
{
	double fx=K[0],cx=K[2],fy=K[4],cy=K[5];
	pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
	Size s = depth_image.size();
	int width = s.width;
	int height = s.height;
	pcl_cloud.width = width*height;
	pcl_cloud.height = 1;
	pcl_cloud.is_dense = true;
	pcl_cloud.header.frame_id = frame_id;
	int i=0;
	for(int r=0;r<height;r+=1)
	{ 
		for(int c=0;c<width;c+=1)
		{   
			if(depth_image.at<unsigned short>(r,c)==0||depth_image.at<unsigned short>(r,c)!=depth_image.at<unsigned short>(r,c)) continue;
			pcl::PointXYZRGB point;
			point.r = color_image.at<Vec3b>(r,c)[2];
			point.g = color_image.at<Vec3b>(r,c)[1];
			point.b = color_image.at<Vec3b>(r,c)[0];
			point.z = depth_image.at<unsigned short>(r,c) * 0.001;
			point.x = point.z * ( (double)c - cx ) / (fx);
			point.y = point.z * ((double)r - cy ) / (fy);
			pcl_cloud.points.push_back(point);
			i++;
		}
	}
	pcl_cloud.width=i;
	pcl_cloud.resize(i);
	cout<<"Done..."<<endl;
	return pcl_cloud;
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


PointCloud<PointXYZRGB> mergeFromImagesAndTransforms(string directory_color_image,string directory_depth_image,string directory_transforms, int positions)
{
	PointCloud<PointXYZRGB> cloud_merged;
	//Reading camera intrinsic parameter...
	ifstream f(directory_transforms+"/camera.csv");
	string cam_intrinsic_str;
	getline(f,cam_intrinsic_str);
	std::vector<std::string> v;
	split(v, cam_intrinsic_str,boost::is_any_of(","));
	VectorXd K=VectorXd::Zero(9);
	for(int i=0;i<9;i++)
		K(i)=stof(v[i]);
	//Reading all the images and transforms and merging the point cloud by applying the appropriate transfromation.
	for(int i=0;i<positions;i+=1)
	{
		string color_image=directory_color_image+"/image"+to_string(i)+".jpg";
		string depth_image=directory_depth_image+"/image"+to_string(i)+"0.png";
		Mat color=imread(color_image);
		Mat depth=imread(depth_image,-1);
		//medianBlur(depth,depth,5);
		pcl::PointCloud<pcl::PointXYZRGB> cloud = makePointCloud(color,depth,K,"camera_color_optical_frame");//Hardcoding the frame, change it later.
		string file_name_out="/home/rex/REX_WS/Test_WS/OTHERS/pointcloud"+to_string(i)+".pcd";

		pcl::io::savePCDFileASCII (file_name_out, cloud );
		MatrixXd rob_T_cam_local=parseTransforms(directory_transforms+"/pointcloud_transforms"+to_string(i)+".txt");
		PointCloud<PointXYZRGB> cloud_new = transformPointCloud(cloud.makeShared(),rob_T_cam_local);
		cloud_merged=cloud_merged+cloud_new;
	}
	PointCloud<PointXYZRGB> cloud_filtered;
	int count=0;
	for(auto pt:cloud_merged.points)
	{
		if(pt.x<-1.0||pt.x>-0.50||pt.y<-0.5||pt.y>0.1||pt.z<0||pt.z>0.2)
			 continue;
		cloud_filtered.points.push_back(pt);
		count++;
	}
	cloud_filtered.height=1;
	cloud_filtered.width=count;
	cout<<count<<endl;
	return cloud_filtered;
}


int main(int argc, char** argv)
{
	string directory_color_image="/home/rex/REX_WS/Test_WS/OTHERS/rex_data/color_images";
	string directory_depth_image="/home/rex/REX_WS/Test_WS/OTHERS/rex_data/depth_image";
	string directory_transforms="/home/rex/REX_WS/Test_WS/OTHERS/rex_data/transforms";
	string output_pcd_file_name="/home/rex/REX_WS/Test_WS/OTHERS/stitched_point_cloud_new.pcd";
	PointCloud<PointXYZRGB> fused_point_cloud = mergeFromImagesAndTransforms(directory_color_image,directory_depth_image,directory_transforms,5);
	pcl::io::savePCDFileASCII (output_pcd_file_name, fused_point_cloud);
	return 0;
}
