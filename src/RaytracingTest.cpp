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

using namespace boost::algorithm;
using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;

class RayTracingEngine
{
    public:
        Camera cam_;
        RayTracingEngine(Camera &cam);
        void rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse);
        void rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation);
};

RayTracingEngine::RayTracingEngine(Camera &cam):cam_(cam){}

void RayTracingEngine::rayTrace(VoxelVolume& volume,Eigen::Affine3f& transformation,bool sparse=true)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    bool found[cam_.getHeight()][cam_.getWidth()]={false};
    int zdelta = 1,rdelta = 1, cdelta = 1;
    if(sparse==true)
    {
        zdelta=5;
        rdelta=10;
        cdelta=10;
    }
    for(int z_depth=10;z_depth<1000;z_depth+=5)
    {
        for(int r=0;r<height;r+=10)
        { 
            for(int c=0;c<width;c+=10)
            {   
                if(found[r][c])
                    continue;
                auto [x,y,z] = cam_.projectPoint(r,c,z_depth);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                if(volume.validPoints(x,y,z)==false)
                    continue;
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel!=nullptr)
                {
                    found[r][c]=true;
                    voxel->view=1;//TODO: Change it to view number.
                }
            }
        }

    }
}

void RayTracingEngine::rayTraceVolume(VoxelVolume& volume,Eigen::Affine3f& transformation)
{
    int width = cam_.getWidth();
    int height = cam_.getHeight();
    int depth[cam_.getHeight()][cam_.getWidth()]={-1};
    Eigen::Affine3f inverseTransformation = transformation.inverse();
    for(float x=volume.xmin_;x<volume.xmax_;x+=volume.xdelta_)
        for(float y=volume.ymin_;y<volume.ymax_;y+=volume.ydelta_)
            for(float z=volume.zmax_-volume.zdelta_;z>=volume.zmin_;z-=volume.zdelta_)
            {
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                if(volume.voxels_[xid][yid][zid]==nullptr)
                    continue;
                auto[xx,yy,zz] = cam_.transformPoints(x,y,z,inverseTransformation);
                auto[r,c] = cam_.deProjectPoint(xx,yy,zz);
                if(cam_.validPixel(r,c)==false)
                    continue;
                depth[r][c] = z*1000;
            }
    cout<<"Depth Map Found"<<endl;
    for(int r=0;r<cam_.getHeight();r++)
        for(int c=0;c<cam_.getWidth();c++)
            if(depth[r][c]>0)
            {
                auto[x,y,z] = cam_.projectPoint(r,c,depth[r][c]);
                tie(x,y,z) = cam_.transformPoints(x,y,z,transformation);
                auto coords = volume.getVoxel(x,y,z);
                int xid = get<0>(coords);
                int yid = get<1>(coords);
                int zid = get<2>(coords);
                if(volume.validPoints(x,y,z)==false)
                        continue;
                Voxel *voxel = volume.voxels_[xid][yid][zid];
                if(voxel==nullptr)
                    continue;
                voxel->view=1;//TODO: Change it to view number.
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
    vector<double> two_T_one_static = {0.5,0,0.5,0,-PI/2,0};
	Eigen::MatrixXd two_T_one = vectorToTransformationMatrix(two_T_one_static);
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            transformation(i,j)=two_T_one(i,j);
    Camera cam(K);

    RayTracingEngine engine(cam);

    VoxelVolume volume;
    volume.setDimensions(-0.5,0.5,-0.5,0.5,0,1);
    volume.setVolumeSize(50,50,50);
    volume.constructVolume();
    volume.integratePointCloud(cloud);

    // engine.rayTrace(volume,transformation);
    engine.rayTrace(volume,transformation,false);

    VisualizationUtilities::PCLVisualizerWrapper viz;
    viz.addCoordinateSystem();
    viz.addPointCloudInVolumeRayTraced(volume);
    viz.addCamera(cam,transformation,"camera",1000);
    viz.spinViewer();
    return 0;
}
