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


namespace TransformationUtilities
{
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

    template<typename PointT> void transformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud_input,typename pcl::PointCloud<PointT>::Ptr cloud_output,Eigen::MatrixXd& a_T_b)
    {
        for(auto point:cloud_input->points)
        {
            Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
            auto pt = point;
            pt.x=pts_trans(0,0);
            pt.y=pts_trans(0,1);
            pt.z=pts_trans(0,2);
            cloud_output->points.push_back(pt);
        }
    }
    
    void transformPointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud_input,pcl::PointCloud<pcl::Normal>::Ptr cloud_output,Eigen::MatrixXd& a_T_b)
    {
        for(auto point:cloud_input->points)
        {
            Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
            pts(0,0)=point.normal[0];
            pts(0,1)=point.normal[1];
            pts(0,2)=point.normal[2];
            Eigen::MatrixXd rotation = a_T_b;
            rotation(3,0)=0;
            rotation(3,1)=0;
            rotation(3,2)=0;
            Eigen::MatrixXd pts_trans=apply_transformation(pts,rotation);
            auto pt = point;
            pt.normal[0]=pts_trans(0,0);
            pt.normal[1]=pts_trans(0,1);
            pt.normal[2]=pts_trans(0,2);
            cloud_output->points.push_back(pt);
        }
    }
    
    std::string validate_seq(std::string seq)
	{
		if(seq =="")
			seq = "ZYX";	
		bool invalid_flag = false;
		if(seq.size()!=3)
		{
			invalid_flag = true;
		}
		for (int i =0;i<3;++i)
			if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z' && seq[i]!='x' && seq[i]!='y' && seq[i]!='z')
			{
				invalid_flag = true; 
				break;
			}
		if(invalid_flag)
		{
			std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
			std::terminate();		
		}
		return seq;
	}

    //Default : ZYX
    Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq="")
    {
        seq = validate_seq(seq);
        Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
        for (int i=0; i<3; ++i)
        {
            if(seq[i]=='X' || seq[i]=='x')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitX());
            else if(seq[i]=='Y' || seq[i]=='y')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitY());			
            else if(seq[i]=='Z' || seq[i]=='z')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitZ());					
        }
        return rot_mat; 
    }

    Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static)
    {
        Eigen::MatrixXd a_T_b = Eigen::MatrixXd::Identity(4,4);
        a_T_b(0,3) = a_T_b_static[0];        
        a_T_b(1,3) = a_T_b_static[1];
        a_T_b(2,3) = a_T_b_static[2];
        Eigen::MatrixXd eul_ang(1,3);
        eul_ang<<a_T_b_static[3],a_T_b_static[4],a_T_b_static[5];
        a_T_b.block(0,0,3,3) = eul2rot(eul_ang);
        return a_T_b;
    }
};

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
