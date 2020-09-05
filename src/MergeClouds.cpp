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

namespace VisualizationUtilities
{
template <typename PointT> void visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
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

template <typename PointT> void visualizePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<PointT> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	// viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);

	}
}
}

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

pcl::PointCloud<pcl::PointXYZRGB> transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud,Eigen::MatrixXd& a_T_b)
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
PointCloud<PointXYZRGB> mergePointClouds(vector<PointCloud<PointXYZRGBNormal>::Ptr> &points, vector<MatrixXd> &transformations)
{
    MatrixXd flange_T_cam(4,4);
    flange_T_cam<<-0.9999  , 0.0018 ,  0.0101 ,  0.0227,
                   0.0016  , 0.9998 , -0.0211 , -0.1139,
                  -0.0102  ,-0.0211 , -0.9997 ,  0.0126,
                   0       , 0      ,  0      ,  1.0000;
    PointCloud<PointXYZRGB> merged_cloud;
    // if(points.size()!=transformations.size())
    //     return merged_cloud;
    for(int i=0;i<points.size();i++)
    {
        MatrixXd base_T_flange = transformations[i];
        MatrixXd base_T_cam = base_T_flange*flange_T_cam;
        MatrixXd cam_T_base = base_T_cam.inverse();
        merged_cloud+=transformPointCloud(points[i],cam_T_base);
    }
    return merged_cloud;
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

namespace InputUtilities
{
vector<MatrixXd> readTransformations(string filename)
{
    vector<MatrixXd> transformations;
	ifstream file(filename);
	MatrixXd mat=MatrixXd::Zero(4,4);
	string line;
	int count=0;
	while(true)
	{
        for(int i=0;i<4;i++)
        {
            getline(file,line);
            if(line.size()==0)
                goto end;
            vector<string> v;
            cout<<line<<endl;
            split(v,line,boost::is_any_of(","));
            for(auto x:v)
                cout<<x<<" ";
            cout<<endl; for(int j=0;j<v.size();j++)
                mat(i,j)=stof(v[j]);
        }
        transformations.push_back(mat);
	}
end:
    return transformations;
}
}

int main(int argc, char** argv)
{
    vector<MatrixXd> trans = InputUtilities::readTransformations("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/transformations_1.txt");
    vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> points;
    for(int i=0;i<5;i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
#if 0
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> ("/home/rex/REX_WS/Catkin_WS/data/base.pcd", *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file for base. \n");
            return (-1);
        }
#else
        pcl::PLYReader Reader;
        Reader.read("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/reference_cam_calibration_outside_box/"+to_string(i+1)+".ply", *cloud);
#endif
        for(int i=0;i<cloud->points.size();i++)
            cloud->points[i].z*=-1;
        points.push_back(cloud);
    }
    auto merged_points = PointCloudProcessing::mergePointClouds(points,trans);
    VisualizationUtilities::visualizePointCloud<pcl::PointXYZRGB>(merged_points);
    return 0;
}
